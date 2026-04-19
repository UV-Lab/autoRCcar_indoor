#include "SrvServer.h"

using namespace robot_bridge;

template<typename RequestType, typename ResponseType>
inline SrvServer<RequestType, ResponseType>::SrvServer()
    : m_socket_id(-1),
      m_cb(nullptr),
      m_is_start_flag(false) {
}
template<typename RequestType, typename ResponseType>
inline SrvServer<RequestType, ResponseType>::~SrvServer() {
    m_is_start_flag = false;
    if (0 <= m_socket_id) {
        nn_close(m_socket_id);
    }
}

template<typename RequestType, typename ResponseType>
inline int robot_bridge::SrvServer<RequestType, ResponseType>::init(
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

    // m_server_socket = nng::rep::open();
    // m_server_socket.listen(url_str.c_str());
    m_socket_id = nn_socket(AF_SP, NN_REP);
    if (0 > m_socket_id) {
        std::cout << "Fail to create the socket. Error: "
                  << nn_strerror(nn_errno()) << std::endl;
        return -1;
    }
    // bind to the URL.
    if (0 > nn_bind(m_socket_id, url_str.c_str())) {
        std::cout << "Fail to bind the URL, Error: "
                  << nn_strerror(nn_errno()) << std::endl;
        nn_close(m_socket_id);
        return -2;
    }
    m_srv_name = service_name + ":";


    return 0;
}

template<typename RequestType, typename ResponseType>
inline int robot_bridge::SrvServer<RequestType, ResponseType>::setupServer(CallbackFunc cb_func) {
    m_cb = cb_func;
    return 0;
}

template<typename RequestType, typename ResponseType>
inline int robot_bridge::SrvServer<RequestType, ResponseType>::startServer() {


    m_is_start_flag = true;
    std::thread srv_thread(std::bind(&SrvServer<RequestType, ResponseType>::serviceThread, this));
    srv_thread.detach();

    return 0;
}

template<typename RequestType, typename ResponseType>
inline int robot_bridge::SrvServer<RequestType, ResponseType>::stopServer() {
    m_is_start_flag = false;
    return 0;
}

template<typename RequestType, typename ResponseType>
inline void robot_bridge::SrvServer<RequestType, ResponseType>::serviceThread() {

    while (true == m_is_start_flag) {
        char *req_buf_ptr(nullptr);
        auto rc = nn_recv(m_socket_id, &req_buf_ptr, NN_MSG, 0);
        if (0 > rc) {
            std::cout << "nn_recv error: " << nn_strerror(nn_errno()) << std::endl;
            nn_freemsg(req_buf_ptr);
            std::this_thread::sleep_for(std::chrono::microseconds(1));
            continue;
        }

        // nng::buffer recv_buf = m_server_socket.recv();
        // std::string recv_buf_str(recv_buf.data<char>(), recv_buf.size());
        std::string recv_buf_str(req_buf_ptr, rc);
        nn_freemsg(req_buf_ptr);
        // recv_buf.release();
        // Check the srv name.
        std::string recv_srv_name(recv_buf_str.begin(), recv_buf_str.begin() + m_srv_name.size());
        if (recv_srv_name != m_srv_name) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // Extract the request msg.
        RequestPtr req_msg_ptr = std::make_shared<RequestType>();
        std::string buf_str(recv_buf_str.begin() + m_srv_name.size(), recv_buf_str.end());
        // TODO: parse from string
        req_msg_ptr->ParseFromString(buf_str);

        // Process the request msg and product the response msg.
        if (nullptr == m_cb) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        ResponsePtr response_msg_ptr = std::make_shared<ResponseType>();
        m_cb(req_msg_ptr, response_msg_ptr);
        std::string response_msg_str;
        response_msg_ptr->SerializeToString(&response_msg_str);
        rc = nn_send(m_socket_id, response_msg_str.c_str(), response_msg_str.size(), 0);
        if (0 > rc) {
            std::cout << "nn_send, error: " << nn_strerror(nn_errno()) << std::endl;
            continue;
        }


        // nng::buffer response_msg_buf(response_msg_str.data(), response_msg_str.size());
        // try {
        //     m_server_socket.send(response_msg_buf);
        // } catch (const nng::exception &e) {
        //     std::cout << "Exception occurs in sending response of " << m_srv_name << std::endl
        //               << "Who: " << e.who() << std::endl
        //               << "What: " << e.what() << std::endl;
        //     continue;
        // }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // response_msg_buf.release();
    }
}

// inline void robot_bridge::SrvServer<RequestType, ResponseType>::serviceThread() {

//     while (true == m_is_start_flag) {
//         nng::buffer recv_buf = m_server_socket.recv();
//         std::string recv_buf_str(recv_buf.data<char>(), recv_buf.size());
//         // recv_buf.release();
//         // Check the srv name.
//         std::string recv_srv_name(recv_buf_str.begin(), recv_buf_str.begin() + m_srv_name.size());
//         if (recv_srv_name != m_srv_name) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(1));
//             continue;
//         }
//         // Extract the request msg.
//         RequestPtr req_msg_ptr = std::make_shared<RequestType>();
//         std::string buf_str(recv_buf_str.begin() + m_srv_name.size(), recv_buf_str.end());
//         // TODO: parse from string
//         req_msg_ptr->ParseFromString(buf_str);

//         // Process the request msg and product the response msg.
//         if (nullptr == m_cb) {
//             std::this_thread::sleep_for(std::chrono::milliseconds(1));
//             continue;
//         }

//         ResponsePtr response_msg_ptr = std::make_shared<ResponseType>();
//         m_cb(req_msg_ptr, response_msg_ptr);
//         std::string response_msg_str;
//         response_msg_ptr->SerializeToString(&response_msg_str);
//         nng::buffer response_msg_buf(response_msg_str.data(), response_msg_str.size());
//         try {
//             m_server_socket.send(response_msg_buf);
//         } catch (const nng::exception &e) {
//             std::cout << "Exception occurs in sending response of " << m_srv_name << std::endl
//                       << "Who: " << e.who() << std::endl
//                       << "What: " << e.what() << std::endl;
//             continue;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//         response_msg_buf.release();
//     }
// }
