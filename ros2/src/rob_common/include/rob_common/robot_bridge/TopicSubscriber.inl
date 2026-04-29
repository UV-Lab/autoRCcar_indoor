#include "TopicSubscriber.h"

using namespace robot_bridge;

template<typename TopicType>
inline TopicSubscriber<TopicType>::TopicSubscriber()
    : m_socket_id(-1) {
    m_shared_cb = nullptr;
    // m_cb = nullptr;
    m_is_sub_flag = false;
}
template<typename TopicType>
inline TopicSubscriber<TopicType>::~TopicSubscriber() {
    m_is_sub_flag = false;

    if (0 <= m_socket_id) {
        nn_close(m_socket_id);
    }
}
template<typename TopicType>
inline int TopicSubscriber<TopicType>::init(const std::string &topic_name,
                                            const ProtocolType &protocol_type,
                                            const std::string &ip_addr) {
    std::string url_str;
    if (ProtocolType::INPROC == protocol_type) {
        url_str = std::string("inproc://") + topic_name;
    } else if (ProtocolType::IPC == protocol_type) {
        url_str = std::string("ipc://") + topic_name;
    } else if (ProtocolType::TCP == protocol_type) {
        url_str = std::string("tcp://") + ip_addr;
    }
    // std::cout << "sub url: " << url_str << std::endl;
    m_topic_name = topic_name + ":";

    // m_sub_socket = nng::sub::open();
    // nng::view sub_topic_name_view(topic_name.c_str(), topic_name.size());
    // nng::sub::set_opt_subscribe(m_sub_socket, sub_topic_name_view);

    m_socket_id = nn_socket(AF_SP, NN_SUB);
    if (0 > m_socket_id) {
        std::cout << "nn_socket error: " << nn_strerror(nn_errno()) << std::endl;
        return -1;
    }
    if (nn_connect(m_socket_id, url_str.c_str()) < 0) {
        std::cout << "nn_socket error: " << nn_strerror(nn_errno()) << std::endl;
        nn_close(m_socket_id);
        return -2;
    }
    if (nn_setsockopt(m_socket_id, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0) {
        std::cout << "nn_setsockopt error: " << nn_strerror(nn_errno()) << std::endl;
        nn_close(m_socket_id);
        return -3;
    }

    m_sub_info.topic_name = topic_name;
    m_sub_info.proto_type = protocol_type;
    m_sub_info.ip_addr = ip_addr;
    // while (true) {
    //     try {
    //         m_sub_socket.dial(url_str.c_str());
    //         break;
    //     } catch (const nng::exception &e) {
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //         continue;
    //     }
    // }
    return 0;
}

template<typename TopicType>
inline int TopicSubscriber<TopicType>::subscribe(CallbackSharedType callback) {
    // m_cb = nullptr;
    m_shared_cb = callback;
    return 0;
}

template<typename TopicType>
inline int robot_bridge::TopicSubscriber<TopicType>::startSubscription() {
    m_is_sub_flag = true;
    subscriptionThread();
    return 0;
}

template<typename TopicType>
inline int robot_bridge::TopicSubscriber<TopicType>::stopSubscription() {
    m_is_sub_flag = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return 0;
}

template<typename TopicType>
inline TopicSubPubInfo robot_bridge::TopicSubscriber<TopicType>::getSubscriptionInfo() const {
    return m_sub_info;
}

template<typename TopicType>
inline void robot_bridge::TopicSubscriber<TopicType>::subscriptionThread() {
    std::shared_ptr<TopicType> topic_msg_ptr = nullptr;
    while (true == m_is_sub_flag) {
        char *topic_buf_ptr(nullptr);
        auto rc = nn_recv(m_socket_id, &topic_buf_ptr, NN_MSG, 0);
        if (0 > rc) {
            std::cout << "nn_recv error: " << nn_strerror(nn_errno()) << std::endl;
            nn_freemsg(topic_buf_ptr);
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            continue;
        }

        // nng::buffer recv_buf = m_sub_socket.recv();
        // std::string recv_buf_str(recv_buf.data<char>(), recv_buf.size());

        std::string recv_buf_str(topic_buf_ptr, rc);
        nn_freemsg(topic_buf_ptr);
        std::string core_buf_str(recv_buf_str.begin() + m_topic_name.size(),
                                 recv_buf_str.end());
        topic_msg_ptr = std::make_shared<TopicType>();
        bool is_ok = topic_msg_ptr->ParseFromString(core_buf_str);
        if (false == is_ok) {
            std::cout << "Fail to parse the string buf." << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            continue;
        }
        if (nullptr != m_shared_cb) {
            m_shared_cb(topic_msg_ptr);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));
        // recv_buf.release();
    }
}
