#ifndef BF9DABFC_31FA_48DD_AAB4_6B1E263CA2C7
#define BF9DABFC_31FA_48DD_AAB4_6B1E263CA2C7

#include <atomic>
#include <functional>
#include <iostream>
#include <iostream>
#include <memory>
#include <thread>

// #include <nngpp/nngpp.h>
// #include <nngpp/protocol/rep0.h>
// #include <nngpp/protocol/req0.h>

#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>


#include "Common.h"

namespace robot_bridge {

template<typename RequestType, typename ResponseType>
class SrvServer {
public:
    using RequestPtr = std::shared_ptr<RequestType>;
    using ResponsePtr = std::shared_ptr<ResponseType>;
    using CallbackFunc = std::function<void(const RequestPtr &, const ResponsePtr &)>;

public:
    SrvServer(/* args */);
    ~SrvServer();

    int init(const std::string &service_name,
             const ProtocolType &protocol_type = ProtocolType::IPC,
             const std::string &ip_addr = "");
    int setupServer(CallbackFunc cb_func);
    int startServer();
    int stopServer();

private:
    void serviceThread();

private:
    // nng::socket m_server_socket;
    int m_socket_id;
    CallbackFunc m_cb;
    std::atomic_bool m_is_start_flag;
    std::string m_srv_name;
};
}// namespace robot_bridge

#include "SrvServer.inl"

#endif /* BF9DABFC_31FA_48DD_AAB4_6B1E263CA2C7 */
