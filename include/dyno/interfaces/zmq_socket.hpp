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

#pragma once

#include <nlohmann/json.hpp>
#include <zmq.hpp>

namespace DYNO {
namespace Interfaces {

class ZMQSocket {
  public:
    /**
     * @brief Construct a new ZeroMQ socket with the default protocol, address
     * and port.
     */
    ZMQSocket();

    /**
     * @brief Construct a new ZeroMQ socket with the provided protocol, address
     * and port.
     *
     * @param protocol ZeroMQ protocol.
     * @param address ZeroMQ address.
     * @param port ZeroMQ port.
     */
    ZMQSocket(std::string protocol, std::string address, int port);

    void Bind();

    void Connect();

    void Close();

    void SendEmpty();

    void SendJSON(nlohmann::json& data);

    void ReceiveJSON(nlohmann::json& data);

    void ReceiveEmpty();

    void SendCBOR(nlohmann::json& data);

    void SendMsgPack(nlohmann::json& data);

    void ReceiveMsgPack(nlohmann::json& data);

    /**
     * @brief Set the ZeroMQ socket protocol.
     *
     * @param protocol ZeroMQ socket protocol.
     */
    void SetProtocol(std::string protocol);

    /**
     * @brief Get the ZeroMQ socket protocol.
     *
     * @return std::string ZeroMQ socket protocol.
     */
    std::string GetProtocol();

    /**
     * @brief Set the ZeroMQ socket address.
     *
     * @param address ZeroMQ socket address.
     */
    void SetAddress(std::string address);

    /**
     * @brief Get the ZeroMQ socket address.
     *
     * @return std::string ZeroMQ socket address.
     */
    std::string GetAddress();

    /**
     * @brief Set the ZeroMQ socket port.
     *
     * @param port ZeroMQ socket port.
     */
    void SetPort(int port);

    /**
     * @brief Get the ZeroMQ socket port.
     *
     * @return int ZeroMQ socket port.
     */
    int GetPort();

  protected:
    /** @brief ZeroMQ context. */
    zmq::context_t context_;

    /** @brief ZeroMQ socket. */
    zmq::socket_t socket_;

    /** @brief ZeroMQ socket protocol. */
    std::string protocol_ = "tcp";

    /** @brief ZeroMQ socket address. */
    std::string address_ = "localhost";

    /** @brief ZeroMQ socket port (if applicable for the current protocol). */
    int port_ = 5555;
};

} // namespace Interfaces
} // namespace DYNO
