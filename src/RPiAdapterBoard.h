#ifndef _RPIADAPTERBOARD_H_
#define _RPIADAPTERBOARD_H_

// Remove SocketCAN include
// #include "can/SocketCAN.h"

// Include DynamixelSerialPort
#include "dynamixel/DynamixelSerialPort.h"

namespace edu
{

/**
 * @class RPiAdapterBoard
 * @brief Interface to EduArt's robot RPi adapter board.
 * @author Stefan May
 * @date 27.04.2022
 */
class RPiAdapterBoard // No longer inherits from SocketCANObserver
{
public:
  /**
   * Constructor
   * @param[in] serial_port DynamixelSerialPort instance
   * @param[in] verbosity verbosity output flag
   */
  RPiAdapterBoard(DynamixelSerialPort* serial_port, bool verbosity=false);

  /**
   * Destructor
   */
  ~RPiAdapterBoard();
  
  /**
   * @brief Get orientation as quaternion
   * @param[out] q Orientation quaternion (layout: [w x y z])
   */
  void getOrientation(double q[4]);
  
  /**
   * @brief Get temperature of carrier board
   * @return temperature in degree Celsius
   */
  double getTemperature();

  /**
   * @brief Get system voltage (provided by supply pins)
   * @return voltage [V]
   */
  double getVoltageSys();

  /**
   * @brief Get raw acceleration values of IMU
   * @param[out] acc Acceleration in x-, y- and z-dimension
   */
  void getAcceleration(double acc[3]);

    /**
   * @brief Get raw angular velocity values of IMU
   * @param[out] vel Angular velocity in x-, y- and z-dimension
   */
  void getAngularVelocity(double vel[3]);

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

    double           _q[4];             // Orientation data as quaternion (layout [w x y z])

    double           _temperature;      // Temperature of surface of carrier board

    double           _voltageSys;       // Voltage supply of adapter board

    double           _acceleration[3];  // Acceleration in x-, y-, z-dimension.

    double           _angular_velocity[3];  // Acceleration in x-, y-, z-dimension.
    
    bool             _verbosity;        // Set this flag to true via the Constructor to get information via cout
    
    bool             _init;
};

} // namespace

#endif // _RPIADAPTERBOARD_H_