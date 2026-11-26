#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <vector>

namespace ego_depth_inputs
{
    using Vec3 = std::array<double, 3>;
    using Quat = std::array<double, 4>; // x, y, z, w

    struct Pose
    {
        int64_t stamp = 0;
        Vec3 position{};
        Quat orientation{0., 0., 0., 1.};
    };

    inline bool normalize(Quat& q)
    {
        double n = 0.;
        for (double x : q)
        {
            if (!std::isfinite(x)) { return false; }
            n += x * x;
        }
        if (!std::isfinite(n) || n < 1e-12) { return false; }
        n = std::sqrt(n);
        for (double& x : q) { x /= n; }
        return true;
    }

    inline bool validPose(Pose& p)
    {
        return p.stamp > 0 &&
            std::all_of(p.position.begin(), p.position.end(), [](double x) { return std::isfinite(x); }) &&
            normalize(p.orientation);
    }

    inline Quat slerp(const Quat& a, Quat b, double u)
    {
        double dot = 0.;
        for (size_t i = 0; i < 4; ++i) { dot += a[i] * b[i]; }
        if (dot < 0.)
        {
            for (double& x : b) { x = -x; }
            dot = -dot;
        }
        dot = std::clamp(dot, 0., 1.);
        double wa = 1. - u, wb = u;
        if (dot < .9995)
        {
            const double theta = std::acos(dot), denominator = std::sin(theta);
            wa = std::sin((1. - u) * theta) / denominator;
            wb = std::sin(u * theta) / denominator;
        }
        Quat out{};
        for (size_t i = 0; i < 4; ++i) { out[i] = wa * a[i] + wb * b[i]; }
        normalize(out);
        return out;
    }

    enum class Insert { Added, Duplicate, Reset, Invalid };

    enum class Match { Ready, Empty, TooOld, Future, Gap };

    struct Sample
    {
        Match state = Match::Empty;
        Pose pose;
        bool rounded = false;
    };

    // Integer nanoseconds end-to-end. No extrapolation: only bracketing samples,
    // or a bounded sub-microsecond endpoint clamp for the VINS double conversion.
    class PoseHistory
    {
    public:
        PoseHistory(size_t capacity, int64_t span, int64_t gap, int64_t tolerance)
            : capacity_(capacity), span_(span), gap_(gap), tolerance_(tolerance)
        {
            if (capacity < 2 || span <= 0 || gap <= 0 || tolerance < 0 || tolerance > 1000)
            {
                throw std::invalid_argument("invalid pose history limits");
            }
        }

        Insert add(Pose pose)
        {
            if (!validPose(pose)) { return Insert::Invalid; }
            Insert result = Insert::Added;
            if (!poses_.empty())
            {
                if (pose.stamp == poses_.back().stamp) { return Insert::Duplicate; }
                if (pose.stamp < poses_.back().stamp)
                {
                    clear();
                    result = Insert::Reset;
                }
            }
            poses_.push_back(pose);
            while (poses_.size() > capacity_ ||
                (poses_.size() > 2 && poses_.back().stamp - poses_[1].stamp > span_))
            {
                poses_.pop_front();
            }
            return result;
        }

        Sample sample(int64_t stamp) const
        {
            if (stamp <= 0) { return {Match::TooOld, {}, false}; }
            if (poses_.empty()) { return {}; }
            if (stamp <= poses_.front().stamp)
            {
                if (poses_.front().stamp - stamp > tolerance_) { return {Match::TooOld, {}, false}; }
                return endpoint(poses_.front(), stamp);
            }
            if (stamp >= poses_.back().stamp)
            {
                if (stamp - poses_.back().stamp > tolerance_) { return {Match::Future, {}, false}; }
                return endpoint(poses_.back(), stamp);
            }
            const auto upper = std::lower_bound(
                poses_.begin(), poses_.end(), stamp,
                [](const Pose& p, int64_t t) { return p.stamp < t; });
            if (upper->stamp == stamp) { return endpoint(*upper, stamp); }
            const auto &a = *(upper - 1), &b = *upper;
            const int64_t dt = b.stamp - a.stamp;
            if (dt > gap_) { return {Match::Gap, {}, false}; }
            const double u = static_cast<double>(stamp - a.stamp) / static_cast<double>(dt);
            Pose pose;
            pose.stamp = stamp;
            for (size_t i = 0; i < 3; ++i)
            {
                pose.position[i] = a.position[i] + u * (b.position[i] - a.position[i]);
            }
            pose.orientation = slerp(a.orientation, b.orientation, u);
            return {Match::Ready, pose, false};
        }

        void clear() { poses_.clear(); }
        size_t size() const { return poses_.size(); }
        int64_t latest() const { return poses_.empty() ? 0 : poses_.back().stamp; }

    private:
        static Sample endpoint(Pose pose, int64_t stamp)
        {
            const bool rounded = pose.stamp != stamp;
            pose.stamp = stamp;
            return {Match::Ready, pose, rounded};
        }

        size_t capacity_;
        int64_t span_, gap_, tolerance_;
        std::deque<Pose> poses_;
    };

    inline bool validDepthLayout(uint32_t width, uint32_t height, uint32_t step, size_t size)
    {
        return width > 0 && height > 0 &&
            static_cast<uint64_t>(width) * 2 <= step &&
            static_cast<uint64_t>(step) * height == size &&
            static_cast<uint64_t>(width) * height <= std::numeric_limits<size_t>::max() / 2;
    }

    // Rotate byte pairs, preserving 16UC1 values on both endian variants and
    // discarding row padding. This never modifies the received message.
    inline std::vector<uint8_t> rotateDepth180(
        const std::vector<uint8_t>& data, uint32_t width, uint32_t height, uint32_t step)
    {
        if (!validDepthLayout(width, height, step, data.size()))
        {
            throw std::invalid_argument("invalid 16UC1 image layout");
        }
        std::vector<uint8_t> out(static_cast<size_t>(width) * height * 2);
        for (uint32_t y = 0; y < height; ++y)
        {
            for (uint32_t x = 0; x < width; ++x)
            {
                const size_t from = static_cast<size_t>(height - 1 - y) * step +
                    static_cast<size_t>(width - 1 - x) * 2;
                const size_t to = (static_cast<size_t>(y) * width + x) * 2;
                out[to] = data[from];
                out[to + 1] = data[from + 1];
            }
        }
        return out;
    }
} // namespace ego_depth_inputs
