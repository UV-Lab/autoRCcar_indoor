#include <iostream>
#include <map>
#include <thread>

#include "robot_base/geometry_msgs.pb.h"
#include "robot_bridge/TopicSubscriber.h"

namespace rb = robot_bridge;

int main(int argc, char const *argv[]) {

    std::cout << "The CMD format: robot_sub_test url_protocol url_addr topic_name" << std::endl;
    if (4 != argc) {
        std::cout << "The CMD format is wrong, please check the CMD format." << std::endl;
        return -1;
    }

    using PoseMsg = gemoetry_msgs::PoseStamped;
    using PoseSubscriber = rb::TopicSubscriber<PoseMsg>;


    std::shared_ptr<PoseSubscriber> pose_sub_ptr =
            std::make_shared<PoseSubscriber>();

    std::string topic_name(argv[3]);
    std::string url_protocol_str(argv[1]);
    std::string url_addr(argv[2]);
    std::map<std::string, ProtocolType> map_str_to_proto_type{
            {"inproc", ProtocolType::INPROC},
            {"ipc", ProtocolType::IPC},
            {"tcp", ProtocolType::TCP}};
    pose_sub_ptr->init(topic_name,
                       map_str_to_proto_type.at(url_protocol_str),
                       url_addr);

    auto cb = [](PoseSubscriber::MsgSharedPtr msg_ptr) -> void {
        std::cout << "pose pos_x = " << msg_ptr->pose().position().x() << std::endl;
    };
    pose_sub_ptr->subscribe(cb);
    std::cout << "Start the subscription:" << std::endl;
    std::thread sub_thread([pose_sub_ptr]() {
        pose_sub_ptr->startSubscription();
    });

    for (int idx = 0, cnt = 10; idx != cnt; ++idx) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    pose_sub_ptr->stopSubscription();
    std::cout << "End the subscription." << std::endl;
    pose_sub_ptr.reset();
    std::cout << "Release the pose sub." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Exit the app." << std::endl;
    return 0;
}
