#ifndef E114781E_6265_4BAE_B8D4_65C12D001C6F
#define E114781E_6265_4BAE_B8D4_65C12D001C6F

#include <memory>
#include <string>

#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_posh/popo/publisher.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iox/signal_watcher.hpp>

namespace robot_bridge {

class ShmServerParams {
public:
    std::string runtime_name;
    std::string shm_msg_name = "RobotSharedMemory";
    // size_t shm_msg_size = 65536;// 共享内存大小
};

template<std::size_t DataSize>
class ShmServer {
public:
    struct StringData {
        // uint64_t timestamp;
        // uint32_t length;
        char data[DataSize];// 预留102400字节空间(100KB)
    };

public:
    ShmServer(std::shared_ptr<ShmServerParams> params_ptr) {
        m_params_ptr = params_ptr;
    }
    virtual ~ShmServer() {
    }

    int init() {
        iox::cxx::string<MAX_NAME_LENGTH> runtime_name(iox::cxx::TruncateToCapacity,
                                                       m_params_ptr->runtime_name.c_str());
        // iox::cxx::string<255> runtime_name(iox::cxx::TruncateToCapacity,
        //                                    "shm_server_runtime_1");
        iox::runtime::PoshRuntime::initRuntime(runtime_name);
        iox::cxx::string<MAX_NAME_LENGTH> event_name(iox::cxx::TruncateToCapacity,
                                                     m_params_ptr->shm_msg_name.c_str());
        // iox::cxx::string<255> event_name(iox::cxx::TruncateToCapacity,
        //                                  "shm_server_event_1");
        // m_publisher_ptr = std::make_shared<iox::popo::Publisher<StringData>>({"Robot", "Msg", event_name});
        m_publisher_ptr.reset(new iox::popo::Publisher<StringData>({"Robot", "Msg", event_name}));
        return 0;
    }

    int write(const std::string &buf_str) {
        if (m_publisher_ptr == nullptr) {
            return -1;
        }
        auto &publisher = *m_publisher_ptr;
        publisher.loan().and_then([&](auto &sample) {
                            memcpy(sample->data, buf_str.c_str(), buf_str.size());
                            sample.publish();
                        })
                .or_else([](auto &error) {
                    // do something with error
                    std::cerr << "Unable to loan sample, error code: " << error << std::endl;
                });

        return 0;
    }

private:
    std::shared_ptr<ShmServerParams> m_params_ptr = nullptr;
    // struct Impl;
    // std::unique_ptr<Impl> m_impl_ptr;
    std::shared_ptr<iox::popo::Publisher<StringData>> m_publisher_ptr = nullptr;

    static constexpr uint32_t MAX_NAME_LENGTH = 87;
};
}// namespace robot_bridge
#endif /* E114781E_6265_4BAE_B8D4_65C12D001C6F */
