#pragma once

#include <iostream>
#include <string>

#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::udp;

namespace DYNO {
namespace Interfaces {

class HelloWorldServer {
  public:
    HelloWorldServer(boost::asio::io_service& io_service)
        : socket_(io_service,
                  boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                                 1111)) {
        WaitForDatagram();
    }

  private:
    void WaitForDatagram() {
        socket_.async_receive_from(boost::asio::buffer(buffer_),
                                   endpoint_,
                                   std::bind(&HelloWorldServer::Receive,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2));
    }

    void Receive(const boost::system::error_code& error,
                 std::size_t bytes_transferred) {
        if(!error || error == boost::asio::error::message_size) {
            auto message = std::make_shared<std::string>("Hello, World\n");

            auto bufferChar = static_cast<char*>(buffer_.begin());
            auto js =
                nlohmann::json::parse(bufferChar,
                                      bufferChar + int(bytes_transferred));

            std::cout << js << std::endl;

            socket_.async_send_to(boost::asio::buffer(*message),
                                  endpoint_,
                                  std::bind(&HelloWorldServer::Send,
                                            this,
                                            message,
                                            std::placeholders::_1,
                                            std::placeholders::_2));
        }
    }

    void Send(std::shared_ptr<std::string> message,
              const boost::system::error_code& ec,
              std::size_t bytes_transferred) {
        WaitForDatagram();
    }

    udp::socket socket_;

    udp::endpoint endpoint_;

    std::array<char, 1024> buffer_;
};

} // namespace Interfaces
} // namespace DYNO