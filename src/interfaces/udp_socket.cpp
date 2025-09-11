#include <dyno/interfaces/udp_socket.hpp>

int main() {
    try {
        boost::asio::io_service io_service;
        DYNO::Interfaces::HelloWorldServer server{io_service};
        io_service.run();
    } catch(const std::exception& ex) { std::cerr << ex.what() << std::endl; }
    return 0;
}