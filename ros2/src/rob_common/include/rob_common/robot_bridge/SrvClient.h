#ifndef FAE9BE01_4864_4877_82B2_2C228DE51EDE
#define FAE9BE01_4864_4877_82B2_2C228DE51EDE

#include <atomic>
#include <functional>
#include <future>
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
class SrvClient {
public:
    using RequestPtr = std::shared_ptr<RequestType>;
    using ResponsePtr = std::shared_ptr<ResponseType>;

public:
    SrvClient(/* args */);
    ~SrvClient();

    int init(const std::string &service_name,
             const ProtocolType &protocol_type = ProtocolType::IPC,
             const std::string &ip_addr = "");
    ResponsePtr requestSync(RequestPtr req_ptr, int timeout_ms = -1);
    // TODO: may remove it later.
    // int requestAsnyc(RequestPtr req_ptr, int timeout_ms = -1);
    // ResponsePtr spingUntilRequestComplete(int timeout_ms = 0);


private:
    // nng::socket m_client_socket;
    int m_socket_id;
    std::string m_srv_name;
    ResponsePtr m_response_ptr;
    std::future<ResponsePtr> m_future_response;
};

}// namespace robot_bridge

#include "SrvClient.inl"

#endif /* FAE9BE01_4864_4877_82B2_2C228DE51EDE */
