#ifndef QD_KINEMATICS__STAMP_MONITOR_HPP_
#define QD_KINEMATICS__STAMP_MONITOR_HPP_

#include <cstdint>

namespace qd_kinematics
{

// Classifies the measurement stamps of a feedback stream (joint states) before they are used.
// Stamps are never altered: a sample whose stamp cannot be ordered is dropped instead of being
// given a made-up time.
//   OK            newer than the last accepted stamp, within max_gap
//   FIRST         first sample, or first after reset()
//   GAP           newer, but more than max_gap after the last one: the motion in between is unknown,
//                 so odometry restarts its integration from this sample (nothing is integrated across)
//   DUPLICATE     same stamp as the last accepted sample: dropped
//   OUT_OF_ORDER  older than the last accepted stamp (late, reordered or replayed), however much
//                 older: dropped. A stamp alone cannot tell a delayed message from a restarted clock;
//                 the owner watches its own clock and calls reset() when that clock jumps back.
//   INVALID       not finite or not positive: dropped
class StampMonitor
{
  public:
    enum class Result
    {
        FIRST,
        OK,
        GAP,
        DUPLICATE,
        OUT_OF_ORDER,
        INVALID
    };

    explicit StampMonitor(double max_gap = 0.1) : max_gap_(max_gap)
    {
    }

    Result check(double stamp);

    // The sample is usable (its stamp becomes the last accepted one).
    static bool accepted(Result r)
    {
        return r == Result::FIRST || r == Result::OK || r == Result::GAP;
    }

    static const char *name(Result r);

    void reset()
    {
        has_last_ = false;
    }

    double last() const
    {
        return last_;
    }

    // Dropped samples since construction, by reason (diagnostics).
    std::uint64_t duplicates{0};
    std::uint64_t out_of_order{0};
    std::uint64_t invalid{0};
    std::uint64_t gaps{0};

  private:
    double max_gap_;
    double last_{0.0};
    bool has_last_{false};
};

} // namespace qd_kinematics

#endif // QD_KINEMATICS__STAMP_MONITOR_HPP_
