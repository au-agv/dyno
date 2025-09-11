/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( D ) ( Y ) ( N ) ( O )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +              DYNO: Ground Vehicle Dynamics Validation Toolkit             +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <dyno/interfaces/zmq_socket.hpp>

namespace DYNO {
namespace Interfaces {

ZMQSocket::ZMQSocket() {}

ZMQSocket::ZMQSocket(std::string protocol, std::string address, int port)
    : protocol_(protocol), address_(address), port_(port) {}

void ZMQSocket::Bind() {
    socket_.bind(protocol_ + "://" + address_ + ":" + std::to_string(port_));
}

void ZMQSocket::Connect() {
    socket_ = zmq::socket_t(context_, ZMQ_REQ);
    socket_.connect(protocol_ + "://" + address_ + ":" + std::to_string(port_));
}

void ZMQSocket::Close() {
    socket_.close();
    context_.close();
}

void ZMQSocket::SendJSON(nlohmann::json& data) {
    // Serialize the data to a JSON string.
    auto stream = data.dump();

    // Copy the JSON string contents to a ZeroMQ message.
    zmq::message_t message(stream.length());
    std::memcpy(message.data(), stream.c_str(), stream.size());

    // Send the JSON ZeroMQ message.
    socket_.send(message, zmq::send_flags::dontwait);
}

void ZMQSocket::SendCBOR(nlohmann::json& data) {
    // Serialize the data to CBOR message format.
    auto cbor_data = nlohmann::json::to_cbor(data);

    // Copy the CBOR data to a ZeroMQ message.
    zmq::message_t message(cbor_data.size());
    std::memcpy(message.data(), cbor_data.data(), cbor_data.size());

    // Send the CBOR ZeroMQ message.
    socket_.send(message, zmq::send_flags::none);
}

void ZMQSocket::SendMsgPack(nlohmann::json& data) {
    // Serialize the data to MsgPack message format.
    auto stream = nlohmann::json::to_msgpack(data);

    // Copy the MsgPack data to a ZeroMQ message.
    zmq::message_t message(stream.size());
    std::memcpy(message.data(), stream.data(), stream.size());

    // Send the MsgPack ZeroMQ message.
    socket_.send(message, zmq::send_flags::dontwait);
}

void ZMQSocket::ReceiveMsgPack(nlohmann::json& data) {
    // Receive the ZeroMQ message.
    zmq::message_t message;
    auto result = socket_.recv(message, zmq::recv_flags::none);

    if (result.has_value() && result.value() == EAGAIN) {
        throw std::runtime_error("");
    }

    // Copy the MsgPack ZeroMQ message data to a string.
    std::string msgpack_data =
        std::string(static_cast<char*>(message.data()), message.size());

    data = nlohmann::json::from_msgpack(msgpack_data);
}

void ZMQSocket::ReceiveEmpty() {
    // Receive the ZeroMQ message.
    zmq::message_t message;
    auto result = socket_.recv(message, zmq::recv_flags::none);

    if (result.has_value() && result.value() == EAGAIN) {
        throw std::runtime_error("");
    }
}

void ZMQSocket::SetProtocol(std::string protocol) {
    protocol_ = protocol;
}

std::string ZMQSocket::GetProtocol() {
    return protocol_;
}

void ZMQSocket::SetAddress(std::string address) {
    address_ = address;
}

std::string ZMQSocket::GetAddress() {
    return address_;
}

void ZMQSocket::SetPort(int port) {
    port_ = port;
}

int ZMQSocket::GetPort() {
    return port_;
}

}  // namespace Interfaces
}  // namespace DYNO
