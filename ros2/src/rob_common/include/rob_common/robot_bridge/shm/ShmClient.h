#ifndef B7E52B34_772E_41EC_BCDA_B018E6A48AFF
#define B7E52B34_772E_41EC_BCDA_B018E6A48AFF

#include <memory>
#include <string>

#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iox/signal_watcher.hpp>


namespace robot_bridge {

class ShmClientParams {
public:
    std::string runtime_name;
    std::string shm_msg_name = "RobotSharedMemory";
};


template<std::size_t DataSize>
class ShmClient {
public:
    struct StringData {
        // uint64_t timestamp;
        // uint32_t length;
        char data[DataSize];// 预留102400字节空间(100KB)
    };

public:
    ShmClient(std::shared_ptr<ShmClientParams> params_ptr) {
        m_params_ptr = params_ptr;
    }
    virtual ~ShmClient() {
    }

    int init() {
        iox::cxx::string<MAX_NAME_LENGTH> runtime_name(iox::cxx::TruncateToCapacity,
                                                       m_params_ptr->runtime_name.c_str());
        iox::runtime::PoshRuntime::initRuntime(runtime_name);
        iox::cxx::string<MAX_NAME_LENGTH> event_name(iox::cxx::TruncateToCapacity,
                                                     m_params_ptr->shm_msg_name.c_str());
        m_subscriber_ptr.reset(new iox::popo::Subscriber<StringData>({"Robot", "Msg", event_name}));
        m_subscriber_ptr->subscribe();
        return 0;
    }
    int read(std::string &buf_str) {
        if (nullptr == m_subscriber_ptr) {
            return -1;
        }
        auto &subscriber = *m_subscriber_ptr;
        buf_str.resize(DataSize);
        bool is_receive_msg = false;

        subscriber.take().and_then([&](auto &sample) {
                             std::memcpy(buf_str.data(), sample->data, DataSize);
                             is_receive_msg = true;
                         })
                .or_else([&](auto &result) {
                    if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                        // std::cout << "Error receiving chunk." << std::endl;
                        is_receive_msg = false;
                    }
                });

        if (true == is_receive_msg) {
            return 0;
        } else {
            return -2;
        }
    }

private:
    std::shared_ptr<ShmClientParams> m_params_ptr = nullptr;
    std::shared_ptr<iox::popo::Subscriber<StringData>> m_subscriber_ptr = nullptr;

    static constexpr uint32_t MAX_NAME_LENGTH = 87;
};

}// namespace robot_bridge

#endif /* B7E52B34_772E_41EC_BCDA_B018E6A48AFF */
