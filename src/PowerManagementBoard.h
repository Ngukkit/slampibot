#ifndef _POWERMANAGEMENTBOARD_H_
#define _POWERMANAGEMENTBOARD_H_

// Remove SocketCAN include
// #include "can/SocketCAN.h"

// Include DynamixelSerialPort
#include "dynamixel/DynamixelSerialPort.h"

namespace edu
{

/**
 * @class PowerManagementBoard
 * @brief Interface to EduArt's robot power management board.
 * @author Hannes Duske
 * @date 01.02.2024
 */
class PowerManagementBoard // No longer inherits from SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] serial_port DynamixelSerialPort instance
   * @param[in] verbosity verbosity output flag
   */
  PowerManagementBoard(DynamixelSerialPort* serial_port, bool verbosity=false);

  /**
   * Destructor
   */
  ~PowerManagementBoard();

    /**
   * Set hardware enable signal
   * @return successful operation
   */
  bool enable();

  /**
   * Reset hardware enable signal
   * @return successful operation
   */
  bool disable();
  
  /**
   * @brief Get voltage of main power rail
   * @return voltage [V]
   */
  float getVoltage();
  
  /**
   * @brief Get current consumed by robot
   * @return current [A]
   */
  float getCurrent();

private:

    // notify function is removed as we no longer use SocketCANObserver
    // void notify(struct can_frame* frame);

    DynamixelSerialPort* _serial_port; // Pointer to DynamixelSerialPort instance
    dynamixel::PortHandler *portHandler_; // From DynamixelSerialPort
    dynamixel::PacketHandler *packetHandler_; // From DynamixelSerialPort

    // can_frame        _cf; // No longer needed

    // int32_t          _inputAddress;     // No longer needed
    // int32_t          _outputAddress;    // No longer needed
    // int32_t          _broadcastAddress; // No longer needed

    float            _voltage;          // Voltage of main power rail

    float            _current;          // Current consumed by Robot
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
    bool             _init;
};

} // namespace

#endif // _POWERMANAGEMENTBOARD_H_