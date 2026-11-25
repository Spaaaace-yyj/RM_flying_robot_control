#pragma once
#include "px4_control/core.hpp"
#include <vector>

namespace px4_control
{
    struct Reference
    {
        Vec3 position = Vec3::Zero(), velocity = Vec3::Zero(), acceleration = Vec3::Zero();
    };

    // EGO's cubic, strictly increasing knot convention; time zero is knots[3].
    // No ROS clocks/messages here: the caller supplies both clocks explicitly.
    class EgoSpline
    {
    public:
        EgoSpline(int order, std::vector<Vec3> points, std::vector<double> knots,
                  double max_velocity, double max_acceleration);
        Reference sample(double elapsed) const;
        double duration() const { return duration_; }
        bool stationary() const;
        bool inside(const Vec3& origin, double radius, double min_z, double max_z) const;
    private:
        struct Curve { int degree; std::vector<Vec3> points; std::vector<double> knots; };
        static Curve derivative(const Curve& curve);
        static Vec3 evaluate(const Curve& curve, double u);
        Curve position_, velocity_, acceleration_;
        double duration_ = 0.;
    };

    struct EgoConfig
    {
        double max_goal_distance = 3., min_z = .3, max_z = 2.2;
        double max_velocity = .8, max_acceleration = 1.5;
        double planning_timeout = 5., mission_timeout = 45., settle_timeout = 5.;
        double max_start_age = .5, future_tolerance = .05;
        double continuity_position = .35, continuity_velocity = .5, tracking_error = .6;
        double goal_tolerance = .15, arrival_speed = .15, start_speed = .35;
        void validate() const;
    };

    class EgoExecution
    {
    public:
        explicit EgoExecution(EgoConfig config = {});
        Check begin(const Vec3& goal, const Vec3& position, const Vec3& velocity,
                    double steady_now, double ros_now);
        Check receive(const EgoSpline& spline, int64_t id, double start,
                      const Vec3& position, const Vec3& velocity, double ros_now);
        std::optional<Reference> tick(double steady_now, double ros_now,
                                     const Vec3& position, const Vec3& velocity,
                                     bool perception_fresh, bool planner_fresh);
        void cancel(const std::string& reason);
        bool busy() const { return goal_.has_value(); }
        bool tracking() const { return busy() && spline_.has_value(); }
        const char* stateName() const { return tracking() ? "TRACKING" : busy() ? "WAIT_PLAN" : "IDLE"; }
        const std::string& reason() const { return reason_; }
        const std::optional<Vec3>& goal() const { return goal_; }
        int64_t trajectoryId() const { return id_; }
    private:
        EgoConfig config_;
        std::optional<Vec3> goal_;
        std::optional<EgoSpline> spline_;
        double requested_ros_ = 0., requested_steady_ = 0., start_ = 0., last_ros_ = 0.;
        double settled_since_ = kNever;
        Vec3 origin_ = Vec3::Zero();
        int64_t id_ = -1;
        std::string reason_ = "waiting for explicit goal after takeoff";
    };
}
