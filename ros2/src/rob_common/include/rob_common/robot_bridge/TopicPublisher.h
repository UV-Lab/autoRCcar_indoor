#ifndef B00BAE8F_268D_49A5_8164_BC42719B0390
#define B00BAE8F_268D_49A5_8164_BC42719B0390

#include <iostream>
#include <memory>
#include <string>
#include <vector>

// #include <nngpp/nngpp.h>
// #include <nngpp/protocol/pub0.h>
// #include <nngpp/protocol/sub0.h>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>

#include "Common.h"

namespace robot_bridge {

template<typename TopicType>
class TopicPublisher {
public:
    using MsgSharedPtr = std::shared_ptr<TopicType>;

public:
    TopicPublisher(/* args */);
    ~TopicPublisher();

    /**
     * @brief Initialize the topic publisher.
     * 
     * @param topic_name The topic name.
     * @param protocol_type The protocol type in the URL. Protocol type: inproc, ipc, tcp
     * @param ip_addr The IP addr for TCP protocol. In inproc and ipc, the topic_name is as the ip_addr.
     * @return int 0 -> normal.
     */
    int init(const std::string &topic_name,
             const ProtocolType &protocol_type = ProtocolType::IPC,
             const std::string &ip_addr = "");

    /**
     * @brief Publish the topic msg. The msg should be the protobuf msg.
     * 
     * @param msg The topic msg,
     * @return int 0->normal.
     */
    int publishMsg(const TopicType &msg);

    /**
     * @brief Publisher the topic msg.
     * 
     * @param msg_ptr The pointer of topic msg.
     * @return int 0->normal.
     */
    int publishMsg(MsgSharedPtr msg_ptr);

    TopicSubPubInfo getPublisherInfo() const;

private:
    // nng::socket m_pub_socket;
    int m_socket_id;
    std::string m_topic_name;
    TopicSubPubInfo m_pub_info;
};
}// namespace robot_bridge

#include "TopicPublisher.inl"


#endif /* B00BAE8F_268D_49A5_8164_BC42719B0390 */
