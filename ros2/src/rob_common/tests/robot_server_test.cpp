#include <iostream>

#include "robot_bridge/SrvServer.h"

// #include "proto_test/robot_service.pb.h"
#include <example_msg/example_msg.pb.h>
#include <robot_base/geometry_msgs.pb.h>

namespace rb = robot_bridge;

int main(int argc, char const *argv[]) {
    std::cout << "The cmd format is: robot_server_test url_protocol(inproc, ipc, tcp) url_addr service_name"
              << std::endl;
    if (4 != argc) {
        std::cout << "The cmd format is wrong, please check the cmd format." << std::endl;
        return -1;
    }

    using RequestMsg = example_msgs::RobotStateRequest;
    using ResponseMsg = example_msgs::RobotStateResponse;
    using ServerType = rb::SrvServer<RequestMsg, ResponseMsg>;

    std::shared_ptr<ServerType> server_ptr = std::make_shared<ServerType>();
    std::string srv_name(argv[3]);
    std::string srv_ip_addr(argv[2]);
    std::string srv_proto_type_str(argv[1]);
    std::map<std::string, ProtocolType> map_str_to_proto_type{
            {"inproc", ProtocolType::INPROC},
            {"ipc", ProtocolType::IPC},
            {"tcp", ProtocolType::TCP}};
    auto cur_proto_type = map_str_to_proto_type.at(srv_proto_type_str);
    std::cout << "Create the server: server name = " << srv_name
              << ", server proto_type: " << srv_proto_type_str
              << ", ip addr :" << srv_ip_addr << std::endl;
    server_ptr->init(srv_name, cur_proto_type, srv_ip_addr);
    int pose_idx = 1;
    auto srv_process_func = [&pose_idx](const ServerType::RequestPtr &request_ptr,
                                        const ServerType::ResponsePtr &response_ptr) -> void {
        std::cout << "The robot name = " << request_ptr->header().frame_id() << std::endl;

        auto cur_time_ns = std::chrono::system_clock::now().time_since_epoch().count();
        auto header_ptr = response_ptr->mutable_header();
        header_ptr->set_frame_id(request_ptr->header().frame_id());
        header_ptr->mutable_stamp()->set_nanos(cur_time_ns % 1'000'000'000);
        header_ptr->mutable_stamp()->set_secs(cur_time_ns / 1'000'000'000);
        response_ptr->set_robot_state(example_msgs::RobotStateResponse_RobotState_RUNNING);
        std::cout << "Current timestamp is " << cur_time_ns
                  << ", robot state: " << response_ptr->robot_state() << std::endl;
    };
    std::cout << "Setup the server." << std::endl;
    server_ptr->setupServer(srv_process_func);
    std::cout << "Start the server." << std::endl;
    server_ptr->startServer();


    // while (true) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // }

    for (int idx = 0, cnt = 20; idx != cnt; ++idx) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    server_ptr->stopServer();


    return 0;
}
