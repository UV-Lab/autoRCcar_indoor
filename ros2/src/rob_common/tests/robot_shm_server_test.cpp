#include "shm/ShmServer.h"
#include <iostream>
#include <thread>

const size_t k_shm_size = 1000;

int main(int argc, char *argv[]) {

    std::cout << "cmd: robot_shm_server_test msg_size\n";

    if (argc != 2) {
        std::cout << "please input the correct cmd.\n";
        return 1;
    }

    // int msg_size = std::atoi(argv[1]);

    using namespace robot_bridge;
    auto params = std::make_shared<ShmServerParams>();
    params->runtime_name = "robot_msg_pub_runtime_1";
    params->shm_msg_name = "robot_msg";
    ShmServer<k_shm_size> server(params);

    if (server.init() != 0) {
        std::cerr << "Server initialization failed" << std::endl;
        return 1;
    }

    std::string message;
    while (true) {
        std::cout << "Enter message to send (or 'quit' to exit): ";
        std::getline(std::cin, message);
        if (message == "quit") {
            break;
        }
        std::cout << "msg size = " << message.size() << std::endl;
        if (server.write(message) != 0) {
            std::cerr << "Failed to write message" << std::endl;
        } else {
            std::cout << "Message sent: " << message << std::endl;
        }
    }

    return 0;
}