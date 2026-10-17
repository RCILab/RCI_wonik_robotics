#include "qd_kinematics/stamp_monitor.hpp"

#include <cmath>

namespace qd_kinematics
{

StampMonitor::Result StampMonitor::check(double stamp)
{
    if (!std::isfinite(stamp) || stamp <= 0.0)
    {
        ++invalid;
        return Result::INVALID;
    }
    if (!has_last_)
    {
        last_ = stamp;
        has_last_ = true;
        return Result::FIRST;
    }
    const double dt = stamp - last_;
    if (dt == 0.0)
    {
        ++duplicates;
        return Result::DUPLICATE;
    }
    if (dt < 0.0)
    {
        ++out_of_order;
        return Result::OUT_OF_ORDER;
    }
    last_ = stamp;
    if (dt > max_gap_)
    {
        ++gaps;
        return Result::GAP;
    }
    return Result::OK;
}

const char *StampMonitor::name(Result r)
{
    switch (r)
    {
    case Result::FIRST:
        return "FIRST";
    case Result::OK:
        return "OK";
    case Result::GAP:
        return "GAP";
    case Result::DUPLICATE:
        return "DUPLICATE";
    case Result::OUT_OF_ORDER:
        return "OUT_OF_ORDER";
    case Result::INVALID:
        return "INVALID";
    }
    return "UNKNOWN";
}

} // namespace qd_kinematics
