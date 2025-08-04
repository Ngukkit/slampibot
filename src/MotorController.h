#ifndef _MOTORCONTROLLERCAN_H_
#define _MOTORCONTROLLERCAN_H_

#include <vector>

// Dynamixel SDK includes
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/packet_handler.h"

// Include the new DynamixelSerialPort
#include "dynamixel/DynamixelSerialPort.h"

// Remove SocketCANObserver include
// #include "can/SocketCANObserver.h"

namespace edu
{

  enum CanResponse // This enum might be renamed or removed if not applicable to Dynamixel
  {
    CAN_RESPONSE_RPM = 0,
    CAN_RESPONSE_POS = 1
  };

  struct MotorParams
  {
    // Channel of motor controller interface (might be Dynamixel ID)
    int channel; // This will now represent Dynamixel ID

    // Kinematic vector
    std::vector<double> kinematics;

    enum CanResponse responseMode; // Might be replaced by Dynamixel read modes
    
    int invertEnc;
    float gearRatio;
    float encoderRatio;
    float rpmMax;
    
    MotorParams()
    {
      channel       = 0;
      invertEnc     = 0;
      gearRatio     = 0.0f;
      encoderRatio  = 0.0f;
      rpmMax        = 0.0f;
      kinematics.clear();
      responseMode = CAN_RESPONSE_RPM;
    }

    /**
     * Copy constructor
     * @param[in] p parameter instance to be copied
     */
    MotorParams(const MotorParams &p)
    {
      channel       = p.channel;
      kinematics    = p.kinematics;
      responseMode  = p.responseMode;
      gearRatio     = p.gearRatio;
      encoderRatio  = p.encoderRatio;
      rpmMax        = p.rpmMax;
      invertEnc     = p.invertEnc;
    }
  };

  struct ControllerParams
  {
    // CAN interface ID (now Dynamixel ID for a group of motors or a single motor)
    int canID; // This will now represent a Dynamixel ID or a group ID

    // Control parameters
    unsigned short frequencyScale; // Might not be directly applicable to Dynamixel
    float inputWeight; // Might not be directly applicable to Dynamixel
    unsigned char maxPulseWidth; // Might not be directly applicable to Dynamixel
    unsigned short timeout; // Dynamixel has its own timeout mechanisms
    float kp;
    float ki;
    float kd;
    int antiWindup;

    CanResponse responseMode; // Might be replaced by Dynamixel read modes
    std::vector<MotorParams> motorParams; // Assuming each ControllerParams handles a group of motors

    /**
     * Standard constructor assigns default parameters
     */
    ControllerParams()
    {
      canID = 0;
      frequencyScale = 32;
      inputWeight = 0.8f;
      maxPulseWidth = 50;
      timeout = 300;
      kp = 0.f;
      ki = 0.f;
      kd = 0.f;
      antiWindup = 1;
      responseMode = CAN_RESPONSE_RPM;

      motorParams.resize(2); // Assuming 2 motors per controller for now
      std::vector<double> zeroKinematic{0.f, 0.f, 0.f};
      motorParams[0].channel = 0;
      motorParams[0].kinematics = zeroKinematic;
      motorParams[1].channel = 1;
      motorParams[1].kinematics = zeroKinematic;
    }

    /**
     * Copy constructor
     * @param[in] p parameter instance to be copied
     */
    ControllerParams(const ControllerParams &p)
    {
      canID = p.canID;
      frequencyScale = p.frequencyScale;
      inputWeight = p.inputWeight;
      maxPulseWidth = p.maxPulseWidth;
      timeout = p.timeout;
      kp = p.kp;
      ki = p.ki;
      kd = p.kd;
      antiWindup = p.antiWindup;
      responseMode = p.responseMode;
      motorParams = p.motorParams;
    }
  };

  /**
   * @class MotorControllerDynamixel
   * @brief Dynamixel interface for robot motor control.
   * @author Stefan May (adapted by Gemini)
   * @date 27.04.2022 (adapted 2025)
   */
  class MotorController // No longer inherits from SocketCANObserver
  {
  public:
    /**
     * Constructor
     * @param[in] serial_port DynamixelSerialPort instance
     * @param[in] params controller parameters (now including Dynamixel IDs)
     * @param[in] verbosity verbosity output flag
     */
    MotorController(DynamixelSerialPort *serial_port, ControllerParams params, bool verbosity = 0);

    /**
     * Destructor
     */
    ~MotorController();

    /**
     * Check whether device is initialized after powering on
     */
    bool isInitialized();

    /**
     * Revert initialization state
     */
    void deinit();

    /**
     * Reinitialize device
     */
    void reinit();

    /**
     * Enable device (set torque enable for Dynamixel)
     * @return successful operation
     */
    bool enable();

    /**
     * Disable device (set torque disable for Dynamixel)
     * @return successful operation
     */
    bool disable();

    /**
     * Get state of motor controller (enabled / disabled)
     * @return enable state
     */
    bool getEnableState();

    // broadcastExternalSync is not directly applicable to Dynamixel
    // bool broadcastExternalSync();

    /**
     * @brief Get the parameters of connected motors
     *
     * @return MotorParams vector
     */
    std::vector<MotorParams> getMotorParams();

    // configureResponse is not directly applicable to Dynamixel in the same way
    // bool configureResponse(enum CanResponse mode);

    // invertEncoderPolarity is not directly applicable to Dynamixel in the same way
    // bool invertEncoderPolarity(bool invert[2]);

    // getCanId is replaced by getDynamixelId
    unsigned short getDynamixelId();

    // Timeout is handled by Dynamixel SDK
    // bool setTimeout(unsigned short timeoutInMillis);
    // unsigned short getTimeout();

    // Gear ratio and encoder ticks are handled by Dynamixel SDK or in software
    // bool setGearRatio(float gearRatio[2]);
    // float getGearRatio(size_t motor_num);
    // bool setEncoderTicksPerRev(float encoderTicksPerRev[2]);

    // Frequency scale, max pulse width, PWM are not directly applicable to Dynamixel in the same way
    // bool setFrequencyScale(unsigned short scale);
    // unsigned short getFrequencyScale();
    // bool setMaxPulseWidth(unsigned char pulse);
    // bool setPWM(int pwm[2]);

    /**
     * Set motor revolutions per minute (target velocity for Dynamixel)
     * @param[in] rpm set point value, this device supports 2 channels
     * @return success
     */
    bool setRPM(float rpm[2]);

    /**
     * Get either motor revolutions per minute or motor position (encoder ticks).
     * This will read present velocity/position from Dynamixel.
     * @param[out] response revolutions per minute for motor 1 and 2 / position of motor 1 and 2.
     */
    void getWheelResponse(float response[2]);

    // PID gains are set in Dynamixel control table, not via separate commands
    // bool setKp(float kp);
    // float getKp();
    // bool setKi(float ki);
    // float getKi();
    // bool setKd(float kd);
    // float getKd();
    // bool setAntiWindup(bool activate);
    // bool getAntiWindup();
    // bool setInputWeight(float weight);
    // float getInputWeight();

    /**
     * Stop motors (set target velocity to 0 for Dynamixel)
     */
    void stop();

  protected:
  private:
  
    /**
     * Initialize motor controllers (adjust parameters)
     **/
    void init();
    
    // sendFloat is not applicable to Dynamixel
    // bool sendFloat(int cmd, float f);

    // notify is not applicable as we don't use SocketCANObserver
    // void notify(struct can_frame *frame);

    DynamixelSerialPort *_serial_port; // Pointer to DynamixelSerialPort instance
    dynamixel::PortHandler *portHandler_; // From DynamixelSerialPort
    dynamixel::PacketHandler *packetHandler_; // From DynamixelSerialPort

    ControllerParams _params; // Contains Dynamixel IDs and other motor parameters

    bool _isInit;

    // Dynamixel specific data
    float _present_rpm[2];
    int _present_position[2];
    bool _torque_enabled[2];

    bool _verbosity;
  };

} // namespace

#endif /* _MOTORCONTROLLERCAN_H_ */