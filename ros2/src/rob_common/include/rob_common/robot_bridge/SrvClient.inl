#include "SrvClient.h"

using namespace robot_bridge;

template<typename RequestType, typename ResponseType>
inline SrvClient<RequestType, ResponseType>::SrvClient()
    : m_socket_id(-1) {
}

template<typename RequestType, typename ResponseType>
inline robot_bridge::SrvClient<RequestType, ResponseType>::~SrvClient() {
    if (0 <= m_socket_id) {
        nn_close(m_socket_id);
    }
}

template<typename RequestType, typename ResponseType>
inline int robot_bridge::SrvClient<RequestType, ResponseType>::init(
        const std::string &service_name,
        const ProtocolType &protocol_type, const std::string &ip_addr) {
    std::string url_str;
    m_srv_name = service_name;
    if (ProtocolType::INPROC == protocol_type) {
        url_str = std::string("inproc://") + m_srv_name;
    } else if (ProtocolType::IPC == protocol_type) {
        url_str = std::string("ipc://") + m_srv_name;
    } else if (ProtocolType::TCP == protocol_type) {
        url_str = std::string("tcp://") + ip_addr;
    }
    // m_client_socket = nng::req::open();
    // while (true) {
    //     try {
    //         m_client_socket.dial(url_str.c_str());
    //         break;
    //     } catch (const nng::exception &e) {
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //         continue;
    //     }
    // }

    // Create and connect the socket.
    m_socket_id = nn_socket(AF_SP, NN_REQ);
    if (0 > m_socket_id) {
        std::cout << "nn_socket, error: " << nn_strerror(nn_errno()) << std::endl;
        return -1;
    }
    if (0 > nn_connect(m_socket_id, url_str.c_str())) {
        std::cout << "nn_connect, error: " << nn_strerror(nn_errno()) << std::endl;
        nn_close(m_socket_id);
        return -2;
    }

    m_srv_name += ":";
    return 0;
}

template<typename RequestType, typename ResponseType>
inline typename robot_bridge::SrvClient<RequestType, ResponseType>::ResponsePtr
robot_bridge::SrvClient<RequestType, ResponseType>::requestSync(RequestPtr req_ptr,
                                                                int timeout_ms) {
    // if (0 < timeout_ms) {
    //     m_client_socket.set_opt_ms(nng::to_name(nng::option::recv_timeout),
    //                                nng_duration(timeout_ms));
    //     m_client_socket.set_opt_ms(nng::to_name(nng::option::send_timeout),
    //                                nng_duration(timeout_ms));
    // }

    bool is_ok = true;
    // Send the request msg to server.
    std::string buf_str;
    req_ptr->SerializeToString(&buf_str);
    std::string req_msg_str = m_srv_name + buf_str;
    auto rc = nn_send(m_socket_id, req_msg_str.c_str(), req_msg_str.size(), 0);
    if (0 > rc) {
        std::cout << "nn_send, error: " << nn_strerror(nn_errno()) << std::endl;
        is_ok = false;
    }

    // nng::buffer req_msg_buf(req_msg_str.data(), req_msg_str.size());
    // try {
    //     m_client_socket.send(req_msg_buf);
    // } catch (const nng::exception &e) {
    //     std::cout << "Exception occures in sending request. " << std::endl
    //               << " Who = " << e.who() << std::endl
    //               << "What() = " << e.what() << std::endl;
    //     is_ok = false;
    //     // std::cout<<"Exception type: "<<e.who()<<std::co
    // }
    // req_msg_buf.release();

    if (false == is_ok) {
        return nullptr;
    }

    // Try to request the response msg from server.
    char *response_buf_ptr(nullptr);
    rc = nn_recv(m_socket_id, &response_buf_ptr, NN_MSG, 0);
    if (0 > rc) {
        std::cout << "nn_recv, error: " << nn_strerror(nn_errno()) << std::endl;
        is_ok = false;
    }

    // nng::buffer recv_buf;
    // try {
    //     recv_buf = m_client_socket.recv();
    // } catch (const nng::exception &e) {
    //     std::cout << "Exception occures in receiving response." << std::endl
    //               << "Who = " << e.who() << std::endl
    //               << "What = " << e.what() << std::endl;
    //     is_ok = false;
    // }
    if (false == is_ok) {
        nn_freemsg(response_buf_ptr);
        return nullptr;
    }
    // std::string recv_buf_str(recv_buf.data<char>(), recv_buf.size());
    std::string recv_buf_str(response_buf_ptr, rc);
    nn_freemsg(response_buf_ptr);
    // recv_buf.release();
    ResponsePtr response_msg_ptr = std::make_shared<ResponseType>();
    response_msg_ptr->ParseFromString(recv_buf_str);
    return response_msg_ptr;
}


// template<typename RequestType, typename ResponseType>
// inline typename robot_bridge::SrvClient<RequestType, ResponseType>::ResponsePtr
// robot_bridge::SrvClient<RequestType, ResponseType>::requestSync(RequestPtr req_ptr,
//                                                                 int timeout_ms) {
//     if (0 < timeout_ms) {
//         m_client_socket.set_opt_ms(nng::to_name(nng::option::recv_timeout),
//                                    nng_duration(timeout_ms));
//         m_client_socket.set_opt_ms(nng::to_name(nng::option::send_timeout),
//                                    nng_duration(timeout_ms));
//     }
//     bool is_ok = true;
//     // Send the request msg to server.
//     std::string buf_str;
//     req_ptr->SerializeToString(&buf_str);
//     std::string req_msg_str = m_srv_name + buf_str;
//     nng::buffer req_msg_buf(req_msg_str.data(), req_msg_str.size());
//     try {
//         m_client_socket.send(req_msg_buf);
//     } catch (const nng::exception &e) {
//         std::cout << "Exception occures in sending request. " << std::endl
//                   << " Who = " << e.who() << std::endl
//                   << "What() = " << e.what() << std::endl;
//         is_ok = false;
//         // std::cout<<"Exception type: "<<e.who()<<std::co
//     }
//     req_msg_buf.release();
//     if (false == is_ok) {
//         return nullptr;
//     }

//     // Try to request the response msg from server.
//     nng::buffer recv_buf;
//     try {
//         recv_buf = m_client_socket.recv();
//     } catch (const nng::exception &e) {
//         std::cout << "Exception occures in receiving response." << std::endl
//                   << "Who = " << e.who() << std::endl
//                   << "What = " << e.what() << std::endl;
//         is_ok = false;
//     }
//     if (false == is_ok) {
//         return nullptr;
//     }
//     std::string recv_buf_str(recv_buf.data<char>(), recv_buf.size());
//     // recv_buf.release();
//     ResponsePtr response_msg_ptr = std::make_shared<ResponseType>();
//     response_msg_ptr->ParseFromString(recv_buf_str);
//     return response_msg_ptr;
// }
