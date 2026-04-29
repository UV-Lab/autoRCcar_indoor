#include "shm/ShmClient.h"
#include <iostream>

using namespace robot_bridge;
const size_t k_shm_size = 1000;

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <runtime_name>\n";
        return -1;
    }
    auto params = std::make_shared<ShmClientParams>();
    params->runtime_name = argv[1];
    params->shm_msg_name = "robot_msg";
    ShmClient<k_shm_size> client(params);

    if (client.init() != 0) {
        std::cerr << "Client initialization failed" << std::endl;
        return 1;
    }

    std::string message;
    while (true) {
        if (client.read(message) != 0) {
            // std::cerr << "Failed to read message" << std::endl;
            // break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        std::cout << "Received message: " << message << std::endl;
    }

    return 0;
}