
#include "PowerManagementBoard.h"
#include <iostream>
// Remove CAN protocol include
// #include "can/canprotocol.h"
#include <unistd.h>
#include <cstring>

namespace edu
{

PowerManagementBoard::PowerManagementBoard(DynamixelSerialPort* serial_port, bool verbosity)
{
  _init      = false;
  _verbosity = verbosity;
  _serial_port = serial_port;
  portHandler_ = _serial_port->getPortHandler();
  packetHandler_ = _serial_port->getPacketHandler();
  
  _voltage = 0.f;
  _current = 0.f;

  // With usb_to_dxl firmware, this board's functions are not directly accessible.
  // It only acts as a transparent bridge for Dynamixel communication.
  usleep(10000);
  _init = true; // Assume initialized for now, but no actual communication is happening here.
}

PowerManagementBoard::~PowerManagementBoard()
{

}

bool PowerManagementBoard::enable()
{
  // This function cannot send enable command via usb_to_dxl firmware.
  // The usb_to_dxl firmware only passes Dynamixel Protocol commands.
  if (_verbosity) {
    std::cerr << "[PowerManagementBoard] Warning: enable() called, but power management is not available with usb_to_dxl firmware." << std::endl;
  }
  return false; // Indicate failure
}

bool PowerManagementBoard::disable()
{
  // This function cannot send disable command via usb_to_dxl firmware.
  // The usb_to_dxl firmware only passes Dynamixel Protocol commands.
  if (_verbosity) {
    std::cerr << "[PowerManagementBoard] Warning: disable() called, but power management is not available with usb_to_dxl firmware." << std::endl;
  }
  return false; // Indicate failure
}

float PowerManagementBoard::getVoltage()
{
  // This function cannot read voltage data from OpenCR via usb_to_dxl firmware.
  // Returning dummy value.
  if (_verbosity) {
    std::cerr << "[PowerManagementBoard] Warning: getVoltage() called, but voltage data is not available with usb_to_dxl firmware." << std::endl;
  }
  return _voltage;
}

float PowerManagementBoard::getCurrent()
{
  // This function cannot read current data from OpenCR via usb_to_dxl firmware.
  // Returning dummy value.
  if (_verbosity) {
    std::cerr << "[PowerManagementBoard] Warning: getCurrent() called, but current data is not available with usb_to_dxl firmware." << std::endl;
  }
  return _current;
}

} // namespace
