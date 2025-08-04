#include "MotorController.h"
#include <iostream>
#include <cmath>
#include <string.h>
#include <iomanip>
#include <unistd.h>

// Remove CAN protocol include
// #include "can/canprotocol.h"

// Dynamixel SDK specific control table addresses for XL430-W250-T (Protocol 2.0)
#define ADDR_XL_TORQUE_ENABLE           64
#define ADDR_XL_GOAL_VELOCITY           104
#define ADDR_XL_PRESENT_VELOCITY        128
#define ADDR_XL_OPERATING_MODE          11
#define ADDR_XL_PRESENT_POSITION        132

// Dynamixel Protocol 2.0 values for Operating Mode
#define OPERATING_MODE_VELOCITY_CONTROL 1

namespace edu
{

MotorController::MotorController(DynamixelSerialPort* serial_port, ControllerParams params, bool verbosity) 
  : _params(params), _verbosity(verbosity)
{
  _isInit = false;
  _serial_port = serial_port;
  portHandler_ = _serial_port->getPortHandler();
  packetHandler_ = _serial_port->getPacketHandler();

  // Initialize Dynamixel specific data
  _present_rpm[0] = 0.f;
  _present_rpm[1] = 0.f;
  _present_position[0] = 0;
  _present_position[1] = 0;
  _torque_enabled[0] = false;
  _torque_enabled[1] = false;

  // Print parameters (for debugging, can be removed in production)
  if(_verbosity)
  {
    std::cout << "---------------------------" << std::endl << std::endl;
    std::cout << "Controller CAN ID (now Dynamixel ID): " << params.canID << std::endl;
    std::cout << "kp             = " << params.kp << std::endl;
    std::cout << "ki             = " << params.ki << std::endl;
    std::cout << "kd             = " << params.kd << std::endl;
    std::cout << "antiWindup     = " << params.antiWindup << std::endl;
    std::cout << "responseMode   = " << params.responseMode << std::endl;

    for(unsigned int i=0; i<params.motorParams.size(); i++)
    {
      std::cout << "   --- Motor " << i << std::endl;
      std::cout << "       channel (Dynamixel ID): " << params.motorParams[i].channel << std::endl;
      std::cout << "       kinematics: ";
      for(unsigned int j=0; j<params.motorParams[i].kinematics.size(); j++)
        std::cout << params.motorParams[i].kinematics[j] << " ";
      std::cout << std::endl;
      std::cout << "       gearRatio      = " << params.motorParams[i].gearRatio << std::endl;
      std::cout << "       encoderRatio   = " << params.motorParams[i].encoderRatio << std::endl;
      std::cout << "       rpmMax         = " << params.motorParams[i].rpmMax << std::endl;
      std::cout << "       invertEnc      = " << params.motorParams[i].invertEnc << std::endl;
    }
    std::cout << "---------------------------" << std::endl << std::endl;
  }

  init(); // Initialize Dynamixel motors
}

MotorController::~MotorController()
{
  // Disable torque on motors when destroying the object
  disable();
}

bool MotorController::isInitialized()
{
  return _isInit;
}

void MotorController::deinit()
{
  _isInit = false;
  // Additional de-initialization for Dynamixel if needed
}

void MotorController::reinit()
{
  std::cout << "#MotorController Reinitializing device " << _params.canID << std::endl;
  init();
}

bool MotorController::init()
{
  _isInit = false;
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Ping the Dynamixel motor to check connection
  // Assuming _params.canID is now the Dynamixel ID for this controller's primary motor
  dxl_comm_result = packetHandler_->ping(portHandler_, _params.canID, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::cerr << "[MotorController] Failed to ping Dynamixel ID: " << _params.canID << " - " << packetHandler_->get = TxRxResult(dxl_comm_result) << std::endl;
    return false;
  }
  else if (dxl_error != 0)
  {
    std::cerr << "[MotorController] Dynamixel ID: " << _params.canID << " error: " << packetHandler_->getRxPacketError(dxl_error) << std::endl;
    return false;
  }
  else
  {
    std::cout << "[MotorController] Dynamixel ID: " << _params.canID << " ping successful." << std::endl;
  }

  // Set operating mode to Velocity Control Mode (for XL430-W250-T, Address 11, Value 1)
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, _params.canID, ADDR_XL_OPERATING_MODE, OPERATING_MODE_VELOCITY_CONTROL, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::cerr << "[MotorController] Failed to set operating mode for ID: " << _params.canID << " - " << packetHandler_->getTxRxResult(dxl_comm_result) << std::endl;
    return false;
  }
  else if (dxl_error != 0)
  {
    std::cerr << "[MotorController] Dynamixel ID: " << _params.canID << " error: " << packetHandler_->getRxPacketError(dxl_error) << std::endl;
    return false;
  }
  else
  {
    std::cout << "[MotorController] Dynamixel ID: " << _params.canID << " operating mode set to Velocity Control." << std::endl;
  }

  // Torque Disable initially
  if (!disable())
  {
    std::cerr << "[MotorController] Failed to torque disable Dynamixel ID: " << _params.canID << std::endl;
    return false;
  }

  _isInit = true;
  return true;
}

bool MotorController::enable()
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Torque Enable (Address 64, Value 1)
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, _params.canID, ADDR_XL_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::cerr << "[MotorController] Failed to torque enable Dynamixel ID: " << _params.canID << " - " << packetHandler_->getTxRxResult(dxl_comm_result) << std::endl;
    return false;
  }
  else if (dxl_error != 0)
  {
    std::cerr << "[MotorController] Dynamixel ID: " << _params.canID << " error: " << packetHandler_->getRxPacketError(dxl_error) << std::endl;
    return false;
  }
  else
  {
    _torque_enabled[0] = true; // Assuming _params.canID refers to the first motor in this controller
    std::cout << "[MotorController] Dynamixel ID: " << _params.canID << " torque enabled." << std::endl;
    return true;
  }
}

bool MotorController::disable()
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Torque Disable (Address 64, Value 0)
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, _params.canID, ADDR_XL_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::cerr << "[MotorController] Failed to torque disable Dynamixel ID: " << _params.canID << " - " << packetHandler_->getTxRxResult(dxl_comm_result) << std::endl;
    return false;
  }
  else if (dxl_error != 0)
  {
    std::cerr << "[MotorController] Dynamixel ID: " << _params.canID << " error: " << packetHandler_->getRxPacketError(dxl_error) << std::endl;
    return false;
  }
  else
  {
    _torque_enabled[0] = false; // Assuming _params.canID refers to the first motor in this controller
    std::cout << "[MotorController] Dynamixel ID: " << _params.canID << " torque disabled." << std::endl;
    return true;
  }
}

bool MotorController::getEnableState()
{
  // Read torque enable status from Dynamixel
  uint8_t torque_enable_status = 0;
  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, _params.canID, ADDR_XL_TORQUE_ENABLE, &torque_enable_status, &dxl_error);
  
  if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
  {
    std::cerr << "[MotorController] Failed to read torque enable status for ID: " << _params.canID << std::endl;
    return false;
  }
  _torque_enabled[0] = (torque_enable_status == 1);
  return _torque_enabled[0];
}

std::vector<MotorParams> MotorController::getMotorParams()
{
  return _params.motorParams;
}

unsigned short MotorController::getDynamixelId()
{
  return _params.canID; // _params.canID now represents Dynamixel ID
}

bool MotorController::setRPM(float rpm[2])
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // XL430 Goal Velocity is 4 bytes (signed int32_t)
  // Convert RPM to Dynamixel's Goal Velocity unit (0.229 rev/min per unit)
  // Dynamixel SDK uses raw values, so convert RPM to raw units
  // Assuming rpm[0] is for the motor with ID _params.canID
  // And rpm[1] is for _params.motorParams[1].channel (if applicable)

  // For XL430, 1 unit of Goal Velocity is 0.229 RPM
  // So, raw_value = RPM / 0.229
  // Or, raw_value = RPM * (1 / 0.229) = RPM * 4.366812227
  int32_t goal_velocity_raw_0 = static_cast<int32_t>(rpm[0] / 0.229);
  // int32_t goal_velocity_raw_1 = static_cast<int32_t>(rpm[1] / 0.229); // For the second motor if controlled by this instance

  // Write Goal Velocity for the first motor (ID: _params.canID)
  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, _params.canID, ADDR_XL_GOAL_VELOCITY, goal_velocity_raw_0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::cerr << "[MotorController] Failed to set RPM for ID: " << _params.canID << " - " << packetHandler_->getTxRxResult(dxl_comm_result) << std::endl;
    return false;
  }
  else if (dxl_error != 0)
  {
    std::cerr << "[MotorController] Dynamixel ID: " << _params.canID << " error: " << packetHandler_->getRxPacketError(dxl_error) << std::endl;
    return false;
  }
  return true;
}

void MotorController::getWheelResponse(float response[2])
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Read Present Velocity (Address 128, 4 bytes) for the first motor
  int32_t present_velocity_raw_0 = 0;
  dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, _params.canID, ADDR_XL_PRESENT_VELOCITY, (uint32_t*)&present_velocity_raw_0, &dxl_error);
  
  if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
  {
    std::cerr << "[MotorController] Failed to read present velocity for ID: " << _params.canID << std::endl;
    response[0] = 0.0; // Set to 0 on error
  }
  else
  {
    // Convert raw value to RPM (0.229 rev/min per unit)
    _present_rpm[0] = static_cast<float>(present_velocity_raw_0 * 0.229);
    response[0] = _present_rpm[0];
  }

  // If there's a second motor controlled by this instance, read its response as well
  // This assumes _params.motorParams[1].channel is the ID of the second motor
  if (_params.motorParams.size() > 1) {
      int32_t present_velocity_raw_1 = 0;
      dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, _params.motorParams[1].channel, ADDR_XL_PRESENT_VELOCITY, (uint32_t*)&present_velocity_raw_1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
      {
        std::cerr << "[MotorController] Failed to read present velocity for ID: " << _params.motorParams[1].channel << std::endl;
        response[1] = 0.0; // Set to 0 on error
      }
      else
      {
        _present_rpm[1] = static_cast<float>(present_velocity_raw_1 * 0.229);
        response[1] = _present_rpm[1];
      }
  }
  else
  {
      response[1] = 0.0; // No second motor
  }
}

void MotorController::stop()
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Set Goal Velocity to 0 for the first motor
  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, _params.canID, ADDR_XL_GOAL_VELOCITY, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
  {
    std::cerr << "[MotorController] Failed to stop motor ID: " << _params.canID << std::endl;
  }

  // If there's a second motor, stop it as well
  if (_params.motorParams.size() > 1) {
      dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, _params.motorParams[1].channel, ADDR_XL_GOAL_VELOCITY, 0, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
      {
        std::cerr << "[MotorController] Failed to stop motor ID: " << _params.motorParams[1].channel << std::endl;
      }
  }
}

} // namespace