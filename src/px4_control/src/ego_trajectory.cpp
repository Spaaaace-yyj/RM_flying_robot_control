#include "px4_control/ego_trajectory.hpp"
#include <algorithm>
#include <stdexcept>

namespace px4_control
{
    bool goalRejectionMatches(int64_t sent_stamp, const Vec3& sent_goal, const std::string& expected_frame,
                              int64_t reply_stamp, const Vec3& reply_goal, const std::string& reply_frame)
    {
        return sent_stamp > 0 && sent_stamp == reply_stamp && expected_frame == reply_frame &&
            sent_goal.allFinite() && reply_goal.allFinite() && (sent_goal - reply_goal).norm() < 1e-9;
    }

    EgoSpline::Curve EgoSpline::derivative(const Curve& c)
    {
        Curve d{c.degree - 1, {}, {c.knots.begin() + 1, c.knots.end() - 1}};
        for (size_t i = 0; i + 1 < c.points.size(); ++i)
        {
            const double dt = c.knots[i + c.degree + 1] - c.knots[i + 1];
            d.points.push_back(c.degree * (c.points[i + 1] - c.points[i]) / dt);
        }
        return d;
    }

    Vec3 EgoSpline::evaluate(const Curve& c, double u)
    {
        const int n = static_cast<int>(c.points.size());
        const int k = std::clamp(static_cast<int>(std::upper_bound(c.knots.begin(), c.knots.end(), u) -
                                                 c.knots.begin()) - 1, c.degree, n - 1);
        std::array<Vec3, 4> d;
        for (int j = 0; j <= c.degree; ++j) { d[j] = c.points[k - c.degree + j]; }
        for (int r = 1; r <= c.degree; ++r)
        {
            for (int j = c.degree; j >= r; --j)
            {
                const int i = k - c.degree + j;
                const double alpha = (u - c.knots[i]) / (c.knots[i + c.degree - r + 1] - c.knots[i]);
                d[j] = (1. - alpha) * d[j - 1] + alpha * d[j];
            }
        }
        return d[c.degree];
    }

    EgoSpline::EgoSpline(int order, std::vector<Vec3> points, std::vector<double> knots,
                         double max_velocity, double max_acceleration)
        : position_{order, std::move(points), std::move(knots)}
    {
        const auto& c = position_;
        if (order != 3 || c.points.size() < 4 || c.points.size() > 2048 ||
            c.knots.size() != c.points.size() + 4 || !std::isfinite(max_velocity) || max_velocity <= 0. ||
            !std::isfinite(max_acceleration) || max_acceleration <= 0.)
        { throw std::invalid_argument("invalid cubic spline shape/limits"); }
        for (const auto& p : c.points)
        { if (!p.allFinite()) { throw std::invalid_argument("non-finite spline control point"); } }
        for (size_t i = 0; i < c.knots.size(); ++i)
        {
            if (!std::isfinite(c.knots[i]) || (i && c.knots[i] - c.knots[i - 1] < 1e-6))
            { throw std::invalid_argument("knots must be finite and strictly increasing"); }
        }
        duration_ = c.knots[c.points.size()] - c.knots[3];
        if (!std::isfinite(duration_) || duration_ < .05 || duration_ > 30.)
        { throw std::invalid_argument("spline duration outside [0.05,30] seconds"); }
        velocity_ = derivative(c);
        acceleration_ = derivative(velocity_);
        // Derivative control polygons bound the entire continuous curve, not just sampled times.
        for (const auto& v : velocity_.points)
        { if (!v.allFinite() || v.norm() > max_velocity) { throw std::invalid_argument("spline velocity limit"); } }
        for (const auto& a : acceleration_.points)
        { if (!a.allFinite() || a.norm() > max_acceleration) { throw std::invalid_argument("spline acceleration limit"); } }
    }

    Reference EgoSpline::sample(double elapsed) const
    {
        if (!std::isfinite(elapsed)) { throw std::invalid_argument("non-finite sample time"); }
        const double u = position_.knots[3] + std::clamp(elapsed, 0., duration_);
        return {evaluate(position_, u), evaluate(velocity_, u), evaluate(acceleration_, u)};
    }

    bool EgoSpline::stationary() const
    {
        for (const auto& p : position_.points)
        { if ((p - position_.points.front()).norm() > 1e-5) { return false; } }
        return true;
    }

    bool EgoSpline::inside(const Vec3& origin, double radius, double min_z, double max_z) const
    {
        for (const auto& p : position_.points)
        {
            if ((p - origin).norm() > radius || p.z() < min_z || p.z() > max_z) { return false; }
        }
        return true;
    }

    void EgoConfig::validate() const
    {
        for (double value : {max_goal_distance, max_velocity, max_acceleration, planning_timeout, mission_timeout,
             settle_timeout, max_start_age, future_tolerance, continuity_position, continuity_velocity,
             tracking_error, goal_tolerance, arrival_speed, start_speed})
        { if (!std::isfinite(value) || value <= 0.) { throw std::invalid_argument("invalid EGO execution limit"); } }
        if (!std::isfinite(min_z) || !std::isfinite(max_z) || min_z >= max_z)
        { throw std::invalid_argument("invalid EGO odom altitude range"); }
        if (!std::isfinite(nearby_goal_radius) || nearby_goal_radius < 0. || nearby_goal_radius > 2.)
        { throw std::invalid_argument("EGO nearby goal radius must be in [0,2] metres"); }
    }

    EgoExecution::EgoExecution(EgoConfig config) : config_(config) { config_.validate(); }

    Check EgoExecution::begin(const Vec3& goal, const Vec3& p, const Vec3& v, double steady, double ros)
    {
        if (busy()) { return {false, "one goal at a time; hold to cancel first"}; }
        if (!goal.allFinite() || !p.allFinite() || !v.allFinite() || !std::isfinite(steady) || !std::isfinite(ros))
        { return {false, "non-finite goal/state/time"}; }
        const double distance = (goal - p).norm();
        if (distance > config_.max_goal_distance)
        { return {false, "goal too far: distance_m=" + jsonNumber(distance) +
                        " limit_m=" + jsonNumber(config_.max_goal_distance)}; }
        if (goal.z() < config_.min_z || goal.z() > config_.max_z)
        { return {false, "goal odom altitude outside limits: z_m=" + jsonNumber(goal.z()) +
                        " min_m=" + jsonNumber(config_.min_z) + " max_m=" + jsonNumber(config_.max_z)}; }
        if (v.norm() > config_.start_speed)
        { return {false, "initial speed too high: speed_mps=" + jsonNumber(v.norm()) +
                        " limit_mps=" + jsonNumber(config_.start_speed)}; }
        if (distance <= config_.goal_tolerance)
        {
            cancel("already within goal tolerance; holding current position");
            return {true, reason_};
        }
        goal_ = goal;
        requested_goal_ = goal;
        nearby_stop_ = false;
        origin_ = p;
        spline_.reset();
        id_ = -1;
        requested_ros_ = last_ros_ = ros;
        requested_steady_ = steady;
        settled_since_ = kNever;
        reason_ = "goal accepted; holding while EGO plans";
        return {true, reason_};
    }

    Check EgoExecution::receive(const EgoSpline& spline, int64_t id, double start,
                               const Vec3& p, const Vec3& v, double ros)
    {
        if (!busy()) { return {false, "no authorized goal"}; }
        if (!std::isfinite(start) || !std::isfinite(ros) || start < requested_ros_ ||
            start > ros + config_.future_tolerance || ros - start > config_.max_start_age ||
            id < 0 || (spline_ && (id <= id_ || start <= start_)))
        { return {false, "stale/future/out-of-order spline ignored"}; }
        if (spline.stationary())
        {
            cancel("EGO stationary/emergency-stop trajectory; holding, new goal required");
            return {false, reason_};
        }
        if (!spline.inside(origin_, config_.max_goal_distance + 2., config_.min_z, config_.max_z))
        {
            cancel("EGO trajectory leaves configured odom flight envelope; holding");
            return {false, reason_};
        }
        const auto end = spline.sample(spline.duration());
        // The first complete segment selects a stop within the authorized region. Freeze it
        // for later replans; never let a sequence of nearby endpoints drift away from the request.
        const Vec3 adjustment = end.position - *requested_goal_;
        const bool in_region = adjustment.norm() <= config_.goal_tolerance ||
            (config_.nearby_goal_radius > 0. && adjustment.head<2>().norm() <= config_.nearby_goal_radius + 1e-9 &&
             std::abs(adjustment.z()) <= config_.goal_tolerance);
        if (!in_region || (tracking() && (end.position - *goal_).norm() > config_.goal_tolerance) ||
            end.velocity.norm() > config_.arrival_speed)
        {
            const std::string detail = "spline endpoint does not match authorized fixed goal/stop speed: goal=" +
                jsonVector(*goal_) + " endpoint=" + jsonVector(end.position) +
                " position_error_m=" + jsonNumber((end.position - *goal_).norm()) +
                " terminal_speed_mps=" + jsonNumber(end.velocity.norm()) +
                " requested_goal=" + jsonVector(*requested_goal_) +
                " nearby_radius_m=" + jsonNumber(config_.nearby_goal_radius) +
                " position_tolerance_m=" + jsonNumber(config_.goal_tolerance) +
                " stop_speed_limit_mps=" + jsonNumber(config_.arrival_speed);
            if (tracking()) { cancel(detail); }
            return {false, detail};
        }
        const auto next = spline.sample(ros - start);
        const auto expected = spline_ ? spline_->sample(ros - start_) : Reference{p, v, Vec3::Zero()};
        if (!p.allFinite() || !v.allFinite() || ros >= start + spline.duration() ||
            (next.position - expected.position).norm() > config_.continuity_position ||
            (next.velocity - expected.velocity).norm() > config_.continuity_velocity ||
            (next.position - p).norm() > config_.tracking_error)
        {
            cancel("discontinuous/expired EGO trajectory; holding, new goal required");
            return {false, reason_};
        }
        if (!tracking())
        {
            goal_ = end.position;
            nearby_stop_ = adjustment.norm() > 1e-6;
        }
        spline_ = spline;
        start_ = start;
        id_ = id;
        reason_ = nearby_stop_ ? "following validated EGO spline to nearby free stop" : "following validated EGO spline";
        return {true, reason_};
    }

    void EgoExecution::cancel(const std::string& reason)
    {
        goal_.reset();
        requested_goal_.reset();
        spline_.reset();
        reason_ = reason;
    }

    std::optional<Reference> EgoExecution::tick(double steady, double ros, const Vec3& p, const Vec3& v,
                                               bool perception_fresh, bool planner_fresh)
    {
        if (!busy()) { return std::nullopt; }
        auto stop = [this](const char* reason) -> std::optional<Reference> { cancel(reason); return std::nullopt; };
        if (!std::isfinite(steady) || !std::isfinite(ros) || ros < last_ros_ || !p.allFinite() || !v.allFinite())
        { return stop("invalid execution time/state; goal cancelled"); }
        last_ros_ = ros;
        if (!perception_fresh) { return stop("depth pose stale; holding, new goal required"); }
        if (steady - requested_steady_ > config_.mission_timeout) { return stop("EGO mission timeout; holding"); }
        if (!spline_)
        {
            if (steady - requested_steady_ > config_.planning_timeout) { return stop("EGO planning timeout; holding"); }
            return std::nullopt;
        }
        const double t = ros - start_;
        if (t < spline_->duration() - .1 && !planner_fresh) { return stop("EGO planner heartbeat stale; holding"); }
        auto reference = spline_->sample(t);
        if ((reference.position - p).norm() > config_.tracking_error) { return stop("EGO tracking error; holding"); }
        if (t >= spline_->duration())
        {
            reference.velocity.setZero();
            reference.acceleration.setZero();
            if (!std::isfinite(settled_since_)) { settled_since_ = steady; }
            if ((p - *goal_).norm() <= config_.goal_tolerance && v.norm() <= config_.arrival_speed)
            { return stop(nearby_stop_ ? "nearby goal reached; holding, ready for next goal" :
                                        "goal reached; holding, ready for next goal"); }
            if (steady - settled_since_ > config_.settle_timeout) { return stop("arrival timeout; holding"); }
        }
        return reference;
    }
}
