#include "ego_depth_inputs/core.hpp"

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

using namespace ego_depth_inputs;
static int checks = 0;
static void require(bool condition, const std::string &message)
{
  ++checks;
  if (!condition) {throw std::runtime_error(message);}
}
static bool close(double a, double b) {return std::abs(a - b) < 1e-10;}
static Pose pose(int64_t t, double x, Quat q = {0., 0., 0., 1.})
{
  return {t, {x, 0., 0.}, q};
}

int main()
{
  try {
    const int64_t base = 1795575886022829000LL;
    PoseHistory h(200, 2000000000, 100000000, 1000);
    require(h.sample(base).state == Match::Empty, "empty history");
    require(h.add(pose(base, 0.)) == Insert::Added, "first sample");
    require(h.add(pose(base + 20000000, 2., {0., 0., std::sqrt(.5), std::sqrt(.5)})) == Insert::Added,
      "second sample");
    const auto mid = h.sample(base + 10000000);
    require(mid.state == Match::Ready && close(mid.pose.position[0], 1.), "position at exposure");
    require(close(mid.pose.orientation[2], std::sin(std::acos(-1.) / 8.)), "SLERP z angle");
    require(close(mid.pose.orientation[3], std::cos(std::acos(-1.) / 8.)), "SLERP w angle");
    require(!mid.rounded, "interpolation is not a rounding clamp");
    require(h.sample(base + 20000000).state == Match::Ready, "exact upper sample");
    require(h.sample(base + 20000001).rounded, "one nanosecond rounding after latest");
    require(h.sample(base - 1).rounded, "one nanosecond rounding before oldest");
    require(h.sample(base + 20001001).state == Match::Future, "never extrapolate beyond tolerance");
    require(h.sample(base - 1001).state == Match::TooOld, "reject unavailable old pose");
    require(h.sample(0).state == Match::TooOld, "zero stamp");
    require(h.sample(std::numeric_limits<int64_t>::min()).state == Match::TooOld, "negative stamp no overflow");
    require(h.add(pose(base + 20000000, 999.)) == Insert::Duplicate, "ignore duplicate timestamps");
    require(close(h.sample(base + 20000000).pose.position[0], 2.), "duplicate cannot overwrite state");
    require(h.add(pose(base + 10000000, 3.)) == Insert::Reset, "clock rollback resets history");
    require(h.size() == 1 && h.sample(base).state == Match::TooOld, "do not mix clock epochs");
    require(h.add(pose(base + 11000000, 0., {0., 0., 0., 0.})) == Insert::Invalid, "zero quaternion");
    require(h.add(pose(base + 11000000, std::numeric_limits<double>::quiet_NaN())) == Insert::Invalid,
      "nonfinite position");

    PoseHistory gap(5, 2000000000, 100000000, 0);
    gap.add(pose(base, 0.));
    gap.add(pose(base + 200000000, 1.));
    require(gap.sample(base + 100000000).state == Match::Gap, "refuse interpolation across a pose outage");
    require(gap.sample(base + 1).state == Match::Gap, "gap detection uses full bracketing interval");
    require(gap.sample(base + 200000000).state == Match::Ready, "exact pose does not need interpolation");

    PoseHistory anti(5, 2000000000, 100000000, 0);
    anti.add(pose(base, 0.));
    anti.add(pose(base + 20, 1., {0., 0., 0., -2.}));
    const auto antipodal = anti.sample(base + 10);
    require(close(std::abs(antipodal.pose.orientation[3]), 1.), "antipodal quaternions follow shortest arc");
    for (int64_t i = 0; i <= 20; ++i) {
      const auto sample = anti.sample(base + i);
      require(sample.state == Match::Ready && sample.pose.stamp == base + i, "preserve epoch nanoseconds");
    }
    PoseHistory bounded(3, 100, 100, 0);
    for (int64_t i = 0; i < 30; ++i) {bounded.add(pose(base + i * 10, static_cast<double>(i)));}
    require(bounded.size() == 3 && bounded.sample(base + 10).state == Match::TooOld, "bounded pose memory");

    // Distinct bytes represent either endian convention without decoding values.
    const std::vector<uint8_t> padded{1, 11, 2, 12, 99, 99, 3, 13, 4, 14, 88, 88};
    const auto rotated = rotateDepth180(padded, 2, 2, 6);
    require(rotated == std::vector<uint8_t>({4, 14, 3, 13, 2, 12, 1, 11}), "rotate pixels and remove padding");
    require(padded[4] == 99 && padded[10] == 88, "input message remains untouched");
    require(!validDepthLayout(0, 2, 4, 8), "zero width");
    require(!validDepthLayout(4, 2, 6, 12), "short row");
    require(!validDepthLayout(2, 2, 4, 7), "short buffer");
    require(!validDepthLayout(UINT32_MAX, UINT32_MAX, UINT32_MAX, 0), "overflow layout");
    bool threw = false;
    try {rotateDepth180({}, 2, 2, 4);} catch (const std::invalid_argument &) {threw = true;}
    require(threw, "malformed image rejected before access");

    std::mt19937 random(20261125);
    for (int run = 0; run < 200; ++run) {
      const uint32_t width = 1 + random() % 61, height = 1 + random() % 51;
      const uint32_t step = width * 2 + random() % 9;
      std::vector<uint8_t> source(static_cast<size_t>(step) * height);
      for (auto &x : source) {x = static_cast<uint8_t>(random());}
      const auto actual = rotateDepth180(source, width, height, step);
      bool correct = actual.size() == static_cast<size_t>(width) * height * 2;
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          for (size_t byte = 0; byte < 2; ++byte) {
            correct = correct && actual[(static_cast<size_t>(y) * width + x) * 2 + byte] ==
              source[static_cast<size_t>(height - 1 - y) * step + (width - 1 - x) * 2 + byte];
          }
        }
      }
      require(correct, "random padded image preserves pixel byte pairs");
    }
    std::cout << checks << " depth geometry/timing checks passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
