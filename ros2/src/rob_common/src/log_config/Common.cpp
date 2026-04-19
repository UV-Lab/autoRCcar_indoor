#include "Common.h"

#include <cmath>

using namespace rob_common;

std::uint64_t TimestampRT::s_nanos_per_sec = 1'000'000'000;

rob_common::TimestampRT::TimestampRT()
    : sec(0), nano(0) {
}

rob_common::TimestampRT::TimestampRT(
        const std::int64_t &time_sec,
        const std::uint64_t &time_nano)
    : sec(time_sec), nano(time_nano) {
}

void rob_common::TimestampRT::getCurrentTime() {
    timespec time_spec;
    clock_gettime(CLOCK_MONOTONIC, &time_spec);
    sec = time_spec.tv_sec;
    nano = time_spec.tv_nsec;
}

double rob_common::TimestampRT::toSeconds() const {
    return double(sec) + double(nano) / double(s_nanos_per_sec);
}

std::uint64_t rob_common::TimestampRT::toNanoSecs() const {
    return sec * s_nanos_per_sec + nano;
}

double rob_common::TimestampRT::diffTimeSecs(const TimestampRT &timestamp_end) {
    return timestamp_end.toSeconds() - this->toSeconds();
}

std::int64_t rob_common::TimestampRT::diffTimeNS(const TimestampRT &timestamp_end) {
    return s_nanos_per_sec * diffTimeSecs(timestamp_end);
}

void rob_common::TimestampRT::updateBySecs(const double &time_secs) {
    sec = std::floor(time_secs);
    nano = std::round((time_secs - sec) * s_nanos_per_sec);
}

void rob_common::TimestampRT::updateByNs(const std::uint64_t &time_ns) {
    sec = time_ns / s_nanos_per_sec;
    nano = time_ns % s_nanos_per_sec;
}
