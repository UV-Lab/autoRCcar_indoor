#include <iostream>

#include "robot_bridge/SrvClient.h"

// #include "proto_test/robot_service.pb.h"
#include <example_msg/example_msg.pb.h>
#include <robot_base/geometry_msgs.pb.h>

namespace rb = robot_bridge;

int main(int argc, char const *argv[]) {
    std::cout << "The cmd format is: robot_client_test url_protocol(inproc, ipc, tcp) url_addr service_name"
              << std::endl;
    if (4 != argc) {
        std::cout << "The cmd format is wrong, please check the cmd format." << std::endl;
        return -1;
    }

    using RequestMsg = example_msgs::RobotStateRequest;
    using ResponseMsg = example_msgs::RobotStateResponse;
    using ClientType = rb::SrvClient<RequestMsg, ResponseMsg>;

    std::shared_ptr<ClientType> client_ptr = std::make_shared<ClientType>();
    std::string srv_name(argv[3]);
    std::string srv_ip_addr(argv[2]);
    std::string srv_proto_type_str(argv[1]);
    std::map<std::string, ProtocolType> map_str_to_proto_type{
            {"inproc", ProtocolType::INPROC},
            {"ipc", ProtocolType::IPC},
            {"tcp", ProtocolType::TCP}};
    auto srv_proto_type = map_str_to_proto_type.at(srv_proto_type_str);
    std::cout << "Create the client: url proto: " << srv_proto_type_str
              << ", ip_addr: " << srv_ip_addr
              << ", server_name: " << srv_name << std::endl;
    client_ptr->init(srv_name, srv_proto_type, srv_ip_addr);
    std::cout << "Go into the loop for requesting." << std::endl;
    int request_idx = 0;
    for (int idx = 0, cnt = 10; idx != cnt; ++idx) {
        std::shared_ptr<RequestMsg> request_ptr = std::make_shared<RequestMsg>();
        std::string robot_name = std::string("mobile_robot_") + std::to_string(request_idx++);
        request_ptr->mutable_header()->set_frame_id(robot_name);
        std::cout << "Request the server." << std::endl;
        auto response_ptr = client_ptr->requestSync(request_ptr, 30);

        if (nullptr != response_ptr) {
            const auto &header = response_ptr->header();
            // std::uint64_t response_stamp = header.stamp().secs() * 1'000'000'000 + header.stamp().nanos();
            std::cout << "Response stamp, sec: " << header.stamp().secs()
                      << ", nano: " << header.stamp().nanos() << std::endl;
            std::cout << "Robot state: " << response_ptr->robot_state() << std::endl;
        } else {
            std::cout << "Fail to grab the response from server." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}
