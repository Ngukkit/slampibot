#ifndef _RPIEXTENSIONBOARD_H_
#define _RPIEXTENSIONBOARD_H_

// Remove SocketCAN include
// #include "can/SocketCAN.h"

// Include DynamixelSerialPort
#include "dynamixel/DynamixelSerialPort.h"

namespace edu
{

/**
 * @class RPiExtensionBoard
 * @brief Interface to EduArt's robot RPi extension board.
 * @author Stefan May
 * @date 09.03.2025
 */
class RPiExtensionBoard // No longer inherits from SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] serial_port DynamixelSerialPort instance
   * @param[in] verbosity verbosity output flag
   */
  RPiExtensionBoard(DynamixelSerialPort* serial_port, bool verbosity=false);

  /**
   * Destructor
   */
  ~RPiExtensionBoard();
  
  /**
   * @brief Set angle of servo motors
   * @param[in] angle Desired angle, valid values: [0, 270]
   * @return success state of operation
   */
  bool setServos(double angles[8]);
  
  /**
   * @brief Send enable state for visualization (LEDs)
   * @param[in] enabled joint enabled state of all motors
   * @return success state of operation
   */
  bool sendEnabledState(bool enabled);
  

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
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
};

} // namespace

#endif // _RPIEXTENSIONBOARD_H_