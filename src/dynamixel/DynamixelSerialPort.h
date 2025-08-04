
#ifndef DYNAMIXEL_SERIAL_PORT_H
#define DYNAMIXEL_SERIAL_PORT_H

#include <string>
#include <vector>
#include <memory>

// Dynamixel SDK includes
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/packet_handler.h"

namespace edu
{

/**
 * @class DynamixelSerialPort
 * @brief Manages serial communication with Dynamixel devices using Dynamixel SDK.
 *        This class replaces SocketCAN for USB-Serial communication.
 */
class DynamixelSerialPort
{
public:
    /**
     * @brief Constructor
     * @param device_name The serial port device name (e.g., "/dev/ttyACM0").
     * @param baud_rate The baud rate for serial communication (e.g., 57600, 1000000).
     * @param protocol_version The Dynamixel Protocol version (e.g., 1.0 or 2.0).
     */
    DynamixelSerialPort(const std::string& device_name, int baud_rate, float protocol_version);

    /**
     * @brief Destructor
     */
    ~DynamixelSerialPort();

    /**
     * @brief Open the serial port.
     * @return True if the port was successfully opened, false otherwise.
     */
    bool openPort();

    /**
     * @brief Close the serial port.
     */
    void closePort();

    /**
     * @brief Get the PortHandler instance.
     * @return A pointer to the PortHandler instance.
     */
    dynamixel::PortHandler* getPortHandler() const;

    /**
     * @brief Get the PacketHandler instance.
     * @return A pointer to the PacketHandler instance.
     */
    dynamixel::PacketHandler* getPacketHandler() const;

private:
    std::string device_name_;
    int baud_rate_;
    float protocol_version_;
    
    std::unique_ptr<dynamixel::PortHandler> port_handler_;
    std::unique_ptr<dynamixel::PacketHandler> packet_handler_;
};

} // namespace edu

#endif // DYNAMIXEL_SERIAL_PORT_H
