#ifndef B6606037_DC5B_43C9_AA45_79E44D84BC8C
#define B6606037_DC5B_43C9_AA45_79E44D84BC8C

#include <string>
namespace robot_bridge {

enum class ProtocolType {
    // Communication between threads in a process.
    INPROC = 0,
    // Communication between process in a single host machine.
    IPC = 1,
    // Communication between different machines.
    TCP = 2,
    SHM = 3
};

class TopicSubPubInfo {
public:
    std::string topic_name;
    ProtocolType proto_type;
    std::string ip_addr;
};
}// namespace robot_bridge


#endif /* B6606037_DC5B_43C9_AA45_79E44D84BC8C */
