#include "TopicPublisher.h"

using namespace robot_bridge;

template<typename TopicType>
inline TopicPublisher<TopicType>::TopicPublisher()
    : m_socket_id(-1) {
}

template<typename TopicType>
inline TopicPublisher<TopicType>::~TopicPublisher() {
    if (0 <= m_socket_id) {
        nn_close(m_socket_id);
    }
}

template<typename TopicType>
inline int TopicPublisher<TopicType>::init(const std::string &topic_name,
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
    // std::cout << "url_str = " << url_str << std::endl;
    // m_pub_socket = nng::pub::open();
    // m_pub_socket.listen(url_str.c_str());

    m_socket_id = nn_socket(AF_SP, NN_PUB);
    if (0 > m_socket_id) {
        std::cout << "nn_socket error: " << nn_strerror(nn_errno()) << std::endl;
        return -1;
    }
    if (nn_bind(m_socket_id, url_str.c_str()) < 0) {
        std::cout << "nn_bind error: " << nn_strerror(nn_errno()) << std::endl;
        nn_close(m_socket_id);
        return -2;
    }

    m_topic_name = topic_name + ":";

    m_pub_info.topic_name = topic_name;
    m_pub_info.proto_type = protocol_type;
    m_pub_info.ip_addr = ip_addr;

    return 0;
}

template<typename TopicType>
inline int TopicPublisher<TopicType>::publishMsg(const TopicType &msg) {
    std::string buf_str;
    msg.SerializeToString(&buf_str);
    // std::cout << "buf_str size = " << buf_str.size() << std::endl;
    std::string topic_str = m_topic_name + buf_str;
    // std::cout << "topic_str = " << topic_str
    //           << ", topic_str size = " << topic_str.size()
    //           << ", buf_str = " << buf_str << std::endl;

    // nng::buffer topic_buf(topic_str.data(), topic_str.size());
    // m_pub_socket.send(topic_buf);
    // topic_buf.release();

    auto rc = nn_send(m_socket_id, topic_str.c_str(), topic_str.size(), 0);
    if (0 > rc) {
        std::cout << "nn_send error: " << nn_strerror(nn_errno()) << std::endl;
        return -1;
    } else {
        return 0;
    }
}

template<typename TopicType>
inline int TopicPublisher<TopicType>::publishMsg(MsgSharedPtr msg_ptr) {
    return publishMsg(*msg_ptr);
}

template<typename TopicType>
inline TopicSubPubInfo robot_bridge::TopicPublisher<TopicType>::getPublisherInfo() const {
    return m_pub_info;
}
