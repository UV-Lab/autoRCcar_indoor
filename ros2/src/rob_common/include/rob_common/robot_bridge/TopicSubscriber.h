#ifndef A927B8D1_A614_4E35_B6F7_809A7FBBDDC7
#define A927B8D1_A614_4E35_B6F7_809A7FBBDDC7

#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>


// #include <nngpp/nngpp.h>
// #include <nngpp/protocol/pub0.h>
// #include <nngpp/protocol/sub0.h>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>

#include "Common.h"

namespace robot_bridge {
template<typename TopicType>
class TopicSubscriber {
public:
    using MsgSharedPtr = std::shared_ptr<TopicType>;
    using CallbackSharedType = std::function<void(MsgSharedPtr)>;

public:
    TopicSubscriber(/* args */);
    ~TopicSubscriber();

    /**
     * @brief Initialize the topic subscriber.
     * 
     * @param topic_name The topic name.
     * @param protocol_type The protocol type in the URL. Protocol type: inproc, ipc, tcp
     * @param ip_addr The IP addr for TCP protocol. In inproc and ipc, the topic_name is as the ip_addr.
     * @return int 0->normal.
     */
    int init(const std::string &topic_name,
             const ProtocolType &protocol_type = ProtocolType::IPC,
             const std::string &ip_addr = "");

    /**
     * @brief Set the callback function for subscription.
     * 
     * @param callback The callback function.
     * @return int 0->normal.
     */
    int subscribe(CallbackSharedType callback);

    /**
     * @brief Start to subscribe the topic msg and execute the callback function.
     * 
     * @return int 0->normal.
     */
    int startSubscription();

    /**
     * @brief Stop the subscription of topic msg.
     * 
     * @return int 0->normal.
     */
    int stopSubscription();

    TopicSubPubInfo getSubscriptionInfo() const;

private:
    void subscriptionThread();

private:
    // nng::socket m_sub_socket;
    int m_socket_id;
    std::string m_topic_name;
    CallbackSharedType m_shared_cb;
    std::atomic_bool m_is_sub_flag;
    TopicSubPubInfo m_sub_info;
};
}// namespace robot_bridge

#include "TopicSubscriber.inl"

#endif /* A927B8D1_A614_4E35_B6F7_809A7FBBDDC7 */
