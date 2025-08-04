
#include "RPiExtensionBoard.h"
#include <iostream>
// Remove CAN protocol include
// #include "can/canprotocol.h"
#include <unistd.h>

namespace edu
{

RPiExtensionBoard::RPiExtensionBoard(DynamixelSerialPort* serial_port, bool verbosity)
{
  _verbosity = verbosity;
  _serial_port = serial_port;
  portHandler_ = _serial_port->getPortHandler();
  packetHandler_ = _serial_port->getPacketHandler();
  
  // With usb_to_dxl firmware, this board's functions are not directly accessible.
  // It only acts as a transparent bridge for Dynamixel communication.
}

RPiExtensionBoard::~RPiExtensionBoard()
{

}

bool RPiExtensionBoard::setServos(double angles[8])
{
  // This function cannot control servos via usb_to_dxl firmware.
  // The usb_to_dxl firmware only passes Dynamixel Protocol commands to Dynamixel motors.
  // If you have Dynamixel servos, you would control them directly via Dynamixel SDK.
  // If these are non-Dynamixel servos connected to OpenCR, the usb_to_dxl firmware does not expose them.
  if (_verbosity) {
    std::cerr << "[RPiExtensionBoard] Warning: setServos() called, but servo control is not available with usb_to_dxl firmware." << std::endl;
  }
  return false; // Indicate failure
}

bool RPiExtensionBoard::sendEnabledState(bool enabled)
{
  // This function cannot send enable state (e.g., to LEDs) via usb_to_dxl firmware.
  // The usb_to_dxl firmware only passes Dynamixel Protocol commands.
  if (_verbosity) {
    std::cerr << "[RPiExtensionBoard] Warning: sendEnabledState() called, but this function is not available with usb_to_dxl firmware." << std::endl;
  }
  return false; // Indicate failure
}

} // namespace
