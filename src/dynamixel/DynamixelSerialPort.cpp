
#include "DynamixelSerialPort.h"
#include <iostream>

namespace edu
{

DynamixelSerialPort::DynamixelSerialPort(const std::string& device_name, int baud_rate, float protocol_version)
    : device_name_(device_name),
      baud_rate_(baud_rate),
      protocol_version_(protocol_version)
{
    port_handler_ = std::unique_ptr<dynamixel::PortHandler>(dynamixel::PortHandler::getPortHandler(device_name_.c_str()));
    packet_handler_ = std::unique_ptr<dynamixel::PacketHandler>(dynamixel::PacketHandler::getPacketHandler(protocol_version_));

    if (port_handler_ == nullptr) {
        std::cerr << "[DynamixelSerialPort] Failed to get port handler for device: " << device_name_ << std::endl;
    }
    if (packet_handler_ == nullptr) {
        std::cerr << "[DynamixelSerialPort] Failed to get packet handler for protocol version: " << protocol_version_ << std::endl;
    }
}

DynamixelSerialPort::~DynamixelSerialPort()
{
    closePort();
}

bool DynamixelSerialPort::openPort()
{
    if (port_handler_ == nullptr) {
        std::cerr << "[DynamixelSerialPort] Port handler is null. Cannot open port." << std::endl;
        return false;
    }

    if (port_handler_->openPort())
    {
        std::cout << "[DynamixelSerialPort] Succeeded to open port: " << device_name_ << std::endl;
    }
    else
    {
        std::cerr << "[DynamixelSerialPort] Failed to open port: " << device_name_ << std::endl;
        return false;
    }

    if (port_handler_->setBaudRate(baud_rate_))
    {
        std::cout << "[DynamixelSerialPort] Succeeded to set baudrate: " << baud_rate_ << std::endl;
    }
    else
    {
        std::cerr << "[DynamixelSerialPort] Failed to set baudrate: " << baud_rate_ << std::endl;
        return false;
    }
    return true;
}

void DynamixelSerialPort::closePort()
{
    if (port_handler_ != nullptr && port_handler_->is  OpenPort())
    {
        port_handler_->closePort();
        std::cout << "[DynamixelSerialPort] Port closed: " << device_name_ << std::endl;
    }
}

dynamixel::PortHandler* DynamixelSerialPort::getPortHandler() const
{
    return port_handler_.get();
}

dynamixel::PacketHandler* DynamixelSerialPort::getPacketHandler() const
{
    return packet_handler_.get();
}

} // namespace edu
