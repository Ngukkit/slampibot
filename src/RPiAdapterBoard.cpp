
#include "RPiAdapterBoard.h"
#include <iostream>
// Remove CAN protocol include
// #include "can/canprotocol.h"
#include <unistd.h>

namespace edu
{

RPiAdapterBoard::RPiAdapterBoard(DynamixelSerialPort* serial_port, bool verbosity)
{
  _init = false;
  _verbosity = verbosity;
  _serial_port = serial_port;
  portHandler_ = _serial_port->getPortHandler();
  packetHandler_ = _serial_port->getPacketHandler();
  
  _q[0] = 1.0;
  _q[1] = 0.0;
  _q[2] = 0.0;
  _q[3] = 0.0;

  _temperature  = -273.0;
  _voltageSys   = 0.0;

  _acceleration[0] = 0.0;
  _acceleration[1] = 0.0;
  _acceleration[2] = 0.0;

  _angular_velocity[0] = 0.0;
  _angular_velocity[1] = 0.0;
  _angular_velocity[2] = 0.0;

  // With usb_to_dxl firmware, these boards are not directly accessible via Dynamixel Protocol
  // for their internal sensors. They act as a transparent bridge.
  // Therefore, we cannot initialize them by reading their status.
  // Just a small delay for initialization.
  usleep(10000);
  _init = true; // Assume initialized for now, but no actual communication is happening here.
}

RPiAdapterBoard::~RPiAdapterBoard()
{

}

void RPiAdapterBoard::getOrientation(double q[4])
{
  // This function cannot read IMU data from OpenCR via usb_to_dxl firmware.
  // It only acts as a transparent bridge for Dynamixel communication.
  // Returning dummy values.
  if (_verbosity) {
    std::cerr << "[RPiAdapterBoard] Warning: getOrientation() called, but IMU data is not available with usb_to_dxl firmware." << std::endl;
  }
  q[0] = _q[0];
  q[1] = _q[1];
  q[2] = _q[2];
  q[3] = _q[3];
}

double RPiAdapterBoard::getTemperature()
{
  // This function cannot read temperature data from OpenCR via usb_to_dxl firmware.
  // Returning dummy value.
  if (_verbosity) {
    std::cerr << "[RPiAdapterBoard] Warning: getTemperature() called, but temperature data is not available with usb_to_dxl firmware." << std::endl;
  }
  return _temperature;
}

double RPiAdapterBoard::getVoltageSys()
{
  // This function cannot read system voltage data from OpenCR via usb_to_dxl firmware.
  // Returning dummy value.
  if (_verbosity) {
    std::cerr << "[RPiAdapterBoard] Warning: getVoltageSys() called, but voltage data is not available with usb_to_dxl firmware." << std::endl;
  }
  return _voltageSys;
}

void RPiAdapterBoard::getAcceleration(double acc[3])
{
  // This function cannot read acceleration data from OpenCR via usb_to_dxl firmware.
  // Returning dummy values.
  if (_verbosity) {
    std::cerr << "[RPiAdapterBoard] Warning: getAcceleration() called, but acceleration data is not available with usb_to_dxl firmware." << std::endl;
  }
  acc[0] = _acceleration[0];
  acc[1] = _acceleration[1];
  acc[2] = _acceleration[2];
}

void RPiAdapterBoard::getAngularVelocity(double vel[3])
{
  // This function cannot read angular velocity data from OpenCR via usb_to_dxl firmware.
  // Returning dummy values.
  if (_verbosity) {
    std::cerr << "[RPiAdapterBoard] Warning: getAngularVelocity() called, but angular velocity data is not available with usb_to_dxl firmware." << std::endl;
  }
  vel[0] = _angular_velocity[0];
  vel[1] = _angular_velocity[1];
  vel[2] = _angular_velocity[2];
}

} // namespace
