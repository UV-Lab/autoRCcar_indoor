#ifndef C69610D9_10CB_4393_9500_03D01BF25CCC
#define C69610D9_10CB_4393_9500_03D01BF25CCC

#include <chrono>

#include <time.h>

namespace rob_common {
class TimestampRT {
public:
    std::uint64_t sec;
    std::uint64_t nano;

    static std::uint64_t s_nanos_per_sec;

    TimestampRT();
    TimestampRT(const std::int64_t &time_sec,
                const std::uint64_t &time_nano);

    void getCurrentTime();
    double toSeconds() const;
    std::uint64_t toNanoSecs() const;
    double diffTimeSecs(const TimestampRT &timestamp_end);
    std::int64_t diffTimeNS(const TimestampRT &timestamp_end);
    void updateBySecs(const double &time_secs);
    void updateByNs(const std::uint64_t &time_ns);
};


}// namespace rob_common


#endif /* C69610D9_10CB_4393_9500_03D01BF25CCC */
