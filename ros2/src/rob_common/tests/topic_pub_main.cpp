#include <iostream>
#include <thread>

#include "robot_bridge/TopicPublisher.h"

#include "robot_base/geometry_msgs.pb.h"
#include "robot_base/std_msgs.pb.h"

namespace rb = robot_bridge;

int main(int argc, char const *argv[]) {
    std::cout << "The cmd format is: robot_pub_test url_protocol(inproc, ipc, tcp) url_addr topic_name" << std::endl;
    if (4 != argc) {
        std::cout << "The CMD format is wrong, please check the CMD format." << std::endl;
        return -1;
    }

    using PoseMsg = gemoetry_msgs::PoseStamped;
    using PosePublisher = rb::TopicPublisher<PoseMsg>;
    std::shared_ptr<PosePublisher> pose_pub_ptr = std::make_shared<PosePublisher>();
    std::string topic_name(argv[3]);
    std::string url_protocol_str = std::string(argv[1]);
    std::string url_addr = std::string(argv[2]);
    std::map<std::string, ProtocolType> map_str_to_proto_type{
            {"inproc", ProtocolType::INPROC},
            {"ipc", ProtocolType::IPC},
            {"tcp", ProtocolType::TCP}};
    auto rc = pose_pub_ptr->init(topic_name,
                                 map_str_to_proto_type.at(url_protocol_str),
                                 url_addr);
    std::uint64_t msg_idx = 0;
    for (int idx = 0, cnt = 20; idx != cnt; ++idx) {
        // PoseMsg tmp_pose;
        // tmp_pose.mutable_pose()->mutable_position()->set_x(msg_idx++);
        // pose_pub_ptr->publishMsg(tmp_pose);
        // std::cout << "The pose msg pos_x = " << tmp_pose.pose().position().x() << std::endl;

        std::shared_ptr<PoseMsg> tmp_pose_ptr = std::make_shared<PoseMsg>();
        tmp_pose_ptr->mutable_pose()->mutable_position()->set_x(msg_idx++);
        pose_pub_ptr->publishMsg(tmp_pose_ptr);
        std::cout << "The pose msg pos_x = " << tmp_pose_ptr->pose().position().x() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }


    return 0;
}
