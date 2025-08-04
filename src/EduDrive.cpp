
#include "EduDrive.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <fcntl.h>
#include <geometry_msgs/msg/detail/accel_stamped__struct.hpp>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Remove SocketCAN includes
// #include "can/SocketCAN.h"

namespace edu
{

    EduDrive::EduDrive() : Node("edu_drive_node")
    {

    }

    EduDrive::~EduDrive()
    {
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            delete *it;
        }
        delete _pwr_mgmt;
        delete _adapter;
        delete _extension;
        delete _odometry;

    }

    void EduDrive::initDrive(std::vector<ControllerParams> cp, DynamixelSerialPort &serial_port, bool using_pwr_mgmt, bool verbosity)
    {
        _serial_port = &serial_port; // Assign the DynamixelSerialPort instance

        _using_pwr_mgmt = using_pwr_mgmt;
        _verbosity = verbosity;
        _enabled = false;
        
        _subJoy     = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&EduDrive::joyCallback, this, std::placeholders::_1));
        _subVel     = this->create_subscription<geometry_msgs::msg::Twist>("vel/teleop", 10, std::bind(&EduDrive::velocityCallback, this, std::placeholders::_1));
        _srvEnable  = this->create_service<std_srvs::srv::SetBool>("enable", std::bind(&EduDrive::enableCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publisher of motor shields
        _pubEnabled = this->create_publisher<std_msgs::msg::ByteMultiArray>("enabled", 1);
        _pubRPM     = this->create_publisher<std_msgs::msg::Float32MultiArray>("rpm", 1);

        // Publisher of carrier shield
        _pubTemp             = this->create_publisher<std_msgs::msg::Float32>("temperature", 1);
        _pubVoltageAdapter   = this->create_publisher<std_msgs::msg::Float32>("voltageAdapter", 1);
        _pubImu              = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
		
        // Publisher of power management shield
        _pubVoltagePwrMgmt = this->create_publisher<std_msgs::msg::Float32>("voltagePwrMgmt", 1);
        _pubCurrentPwrMgmt = this->create_publisher<std_msgs::msg::Float32>("currentPwrMgmt", 1);

        // Broadcaster for odometry data
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // CAN devices (now using DynamixelSerialPort for communication)
        // These classes will need to be adapted to use DynamixelSerialPort instead of SocketCAN
        _adapter   = new RPiAdapterBoard(_serial_port, verbosity);
        _extension = new RPiExtensionBoard(_serial_port, verbosity);
        _pwr_mgmt  = new PowerManagementBoard(_serial_port, verbosity);

        _vMax = 0.f;

        bool isKinematicsValid = true;
        for (unsigned int i = 0; i < cp.size(); ++i)
        {
            std::vector<MotorParams> motorParams = cp[i].motorParams;

            for (unsigned int j = 0; j < motorParams.size(); ++j)
            {
                isKinematicsValid &= (motorParams[j].kinematics.size()==3);
				}
        }
        if(!isKinematicsValid)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Kinematic vectors does not fit to drive concept. Vectors of lenght==3 are expected.");

            exit(1);
        }
        
        std::vector<std::vector<double>> kinematicModel;
        for (unsigned int i = 0; i < cp.size(); ++i)
        {
            _mc.push_back(new MotorController(_serial_port, cp[i], verbosity)); // Pass DynamixelSerialPort
            
            for(unsigned int j=0; j<_mc[i]->getMotorParams().size(); j++)
            {
            	std::vector<double> kinematics = _mc[i]->getMotorParams()[j].kinematics;
                kinematicModel.push_back(kinematics);
	         	double kx = kinematics[0];
	         	double kw = kinematics[2];
	         	float rpmMax = std::min(cp[i].motorParams[0].rpmMax, cp[i].motorParams[1].rpmMax); // the slowest motor determines the maximum speed of the system
                if(fabs(kx)>1e-3)
	         	{
	            	float vMax = fabs(rpmMax / 60.f * (2.f * M_PI) / kx);
	            	if(vMax > _vMax) _vMax = vMax;
	            }
	            if(fabs(kw)>1e-3)
	            {
	            	float omegaMax = fabs(rpmMax / 60.f * (2.f * M_PI) / kw);
	            	if(omegaMax > _omegaMax) _omegaMax = omegaMax;
		         }
            }
        }
        edu::Matrix K(kinematicModel);
        edu::Matrix Kinv = K.pseudoInverse();

        _odometry = new Odometry(ODOMETRY_ABSOLUTE_MODE, Kinv);
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Instanciated robot with vMax: " << _vMax << " m/s and omegaMax: " << _omegaMax << " rad/s");
    }

    void EduDrive::run()
    {
        _lastCmd = this->get_clock()->now();

        // Renamed from timerReceiveCAN to timerReceiveDynamixelData
        rclcpp::TimerBase::SharedPtr timerReceiveDynamixelData = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&EduDrive::receiveDynamixelData, this));
        rclcpp::TimerBase::SharedPtr timerCheckLaggyConnection = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EduDrive::checkLaggyConnection, this));

        rclcpp::spin(shared_from_this());

        // No clearObservers for DynamixelSerialPort as it doesn't use observer pattern like SocketCAN
        // _serial_port->clearObservers(); 
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            (*it)->stop();
            (*it)->disable();
        }

        rclcpp::shutdown();
    }

    void EduDrive::enable()
    {
        RCLCPP_INFO(this->get_logger(), "Enabling robot");

        if(_using_pwr_mgmt){
            // Let power management board set hardware enable
            // if the power management board is not used, the user needs to take care about this pin.
            // The adapter board is designed to treat this pin as a input pin
            _pwr_mgmt->enable();
        }

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            if(!(*it)->isInitialized())
                (*it)->reinit();
            (*it)->enable();
        }
    }

    void EduDrive::disable()
    {
        RCLCPP_INFO(this->get_logger(), "Disabling robot");

        if(_using_pwr_mgmt){
            // Let power management board reset hardware enable
            _pwr_mgmt->disable();
        }

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
            (*it)->disable();
    }

    void EduDrive::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        // Assignment of joystick axes to motor commands
        float fwd      = joy->axes[1];               // Range of values [-1:1]
        float left     = joy->axes[0];               // Range of values [-1:1]
        float turn     = joy->axes[2];               // Range of values [-1:1]
        float throttle = (joy->axes[3] + 1.0) / 2.0; // Range of values  [0:1]

        // Enable movement in the direction of the y-axis only when the button 12 is pressed
        if (!joy->buttons[11])
            left = 0;

        // Forward / Backward basic orientation
        double servoPos = _servoPos;
        if(joy->buttons[2])
        {
            _servoPos = 45.0;
        }
        else if(joy->buttons[3])
        {
            _servoPos = 225.0;
        }

        // Coolie hat fine positioning
        if(joy->axes[4]==1)
            _servoPos += 1.0;
        if(joy->axes[4]==-1)
	        _servoPos -= 1.0;
	    
        if(_servoPos < 0.0)
            _servoPos = 0.0;
        if(_servoPos > 270.0)
            _servoPos = 270.0;

        // Avoid sending CAN messages, if servos keep their position
        if(servoPos != _servoPos)
        {
            double angles[8];
            angles[0] = _servoPos;
            angles[1] = _servoPos;
            // these values mean, to not change servo position (values > 270 are ignored)
            angles[2] = 275;
            angles[3] = 275;
            angles[4] = 275;
            angles[5] = 275;
            angles[6] = 275;
            angles[7] = 275;
            _extension->setServos(angles);
	}
        static int32_t btn9Prev  = joy->buttons[9];
        static int32_t btn10Prev = joy->buttons[10];

        if (joy->buttons[9] && !btn9Prev)
        {
            disable();
        }
        else if (joy->buttons[10] && !btn10Prev)
        {
            enable();
        }

        btn9Prev    = joy->buttons[9];
        btn10Prev   = joy->buttons[10];

        float vFwd  = throttle * fwd  * _vMax;
        float vLeft = throttle * left * _vMax;
        float omega = throttle * turn * _omegaMax;

        controlMotors(vFwd, vLeft, omega);
    }

    void EduDrive::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
    }

    bool EduDrive::enableCallback(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<std_srvs::srv::SetBool_Request> request, const std::shared_ptr<std_srvs::srv::SetBool_Response> response)
    {
        // suppress warning about unused variable header
        (void)header;

        if(request->data==true)
        {
            RCLCPP_INFO(this->get_logger(), "%s", "Enabling robot");
            enable();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),  "%s", "Disabling robot");
            disable();
        }
        response->success = true;
        return true;
    }

    void EduDrive::controlMotors(float vFwd, float vLeft, float omega)
    {
        _lastCmd = this->get_clock()->now();
            
        for (unsigned int i = 0; i < _mc.size(); ++i)
        {
            std::vector<double> kinematics0 = _mc[i]->getMotorParams()[0].kinematics;
            std::vector<double> kinematics1 = _mc[i]->getMotorParams()[1].kinematics;
            float w[2];
            w[0] = kinematics0[0] * vFwd + kinematics0[1] * vLeft + kinematics0[2] * omega;
            w[1] = kinematics1[0] * vFwd + kinematics1[1] * vLeft + kinematics1[2] * omega;

            // Convert from rad/s to rpm
            w[0] *= 60.f / (2.f * M_PI);
            w[1] *= 60.f / (2.f * M_PI);
            _mc[i]->setRPM(w);
            if (_verbosity)
                RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Setting RPM for drive" << i << " to " << w[0] << " " << w[1]);
        }
    }

    void EduDrive::receiveDynamixelData() // Renamed from receiveCAN
    {
        // This function needs to be implemented to read data from Dynamixel motors
        // via DynamixelSerialPort and update sensor data (RPM, position, IMU, voltage, current).
        // This will involve using packet_handler_->readTxRx() or similar functions.

        // Example placeholders for data
        float voltageAdapter = 0.0; // Get from RPiAdapterBoard
        float voltagePwrMgmt = 0.0; // Get from PowerManagementBoard
        
        std_msgs::msg::Float32MultiArray msgRPM;
        std_msgs::msg::ByteMultiArray msgEnabled;
        geometry_msgs::msg::TransformStamped msgTransform;

        bool controllersInitialized = true;
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
	        controllersInitialized = controllersInitialized && (*it)->isInitialized();
	}
        
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            float response[2] = {0, 0};
            bool enableState = false;
            if(controllersInitialized)
            {
                // Read Dynamixel motor status (present speed, present position)
                // and update response[0], response[1], and enableState.
                (*it)->getWheelResponse(response); // This now reads from Dynamixel
                enableState = (*it)->getEnableState(); // This now reads from Dynamixel
            }
            msgRPM.data.push_back(response[0]);
            msgRPM.data.push_back(response[1]);
            msgEnabled.data.push_back(enableState);
        }
        
        rclcpp::Time stampReceived = this->get_clock()->now();

        _enabled = false;
        if(msgEnabled.data.size()>0)
        {
            _enabled = msgEnabled.data[0];
            for(unsigned int i=1; i<msgEnabled.data.size(); i++)
            {
                _enabled &= msgEnabled.data[i];
            }
            _extension->sendEnabledState(_enabled); // This will also need to be adapted for Dynamixel
        }

        // Odometry update based on motor RPMs (needs actual RPMs from Dynamixel)
        _odometry->update(static_cast<std::uint64_t>(stampReceived.nanoseconds()), edu::Vec(msgRPM.data.begin(), msgRPM.data.end()));
        
        Pose pose = _odometry->get_pose();
        msgTransform.header.stamp = stampReceived;
        msgTransform.header.frame_id = "odom";
        msgTransform.child_frame_id = "base_link";

        tf2::Quaternion q_odom;
        q_odom.setEuler(0, 0, pose.theta);
        msgTransform.transform.translation.x = pose.x;
        msgTransform.transform.translation.y = pose.y;
        msgTransform.transform.translation.z = 0;
        msgTransform.transform.rotation.w = q_odom.w();
        msgTransform.transform.rotation.x = q_odom.x();
        msgTransform.transform.rotation.y = q_odom.y();
        msgTransform.transform.rotation.z = q_odom.z(); 
        _tf_broadcaster->sendTransform(msgTransform);

        _pubRPM->publish(msgRPM);
        _pubEnabled->publish(msgEnabled);

        // --- Sensor data publishing (PLACEHOLDERS for usb_to_dxl firmware) ---
        // The usb_to_dxl firmware primarily acts as a transparent bridge for Dynamixel communication.
        // It does NOT expose OpenCR's internal IMU, voltage, current, or temperature sensors
        // via Dynamixel Protocol or any other direct means through the USB-Serial connection.
        // Therefore, these values cannot be read directly using this firmware.
        // The following publishers will send dummy/default values.

        std_msgs::msg::Float32 msgTemperature;
        msgTemperature.data = _adapter->getTemperature(); // Returns dummy value
        _pubTemp->publish(msgTemperature);

        std_msgs::msg::Float32 msgVoltageAdapter;
        msgVoltageAdapter.data = _adapter->getVoltageSys(); // Returns dummy value
        _pubVoltageAdapter->publish(msgVoltageAdapter);

        double q[4], a[3], v[3];
        _adapter->getOrientation(q); // Returns dummy values
        _adapter->getAcceleration(a); // Returns dummy values
        _adapter->getAngularVelocity(v); // Returns dummy values
        sensor_msgs::msg::Imu msgImu;
        msgImu.header.stamp = stampReceived;
        msgImu.header.frame_id = "base_link";
        msgImu.orientation.w = q[0];
        msgImu.orientation.x = q[1];
        msgImu.orientation.y = q[2];
        msgImu.orientation.z = q[3];
        msgImu.linear_acceleration.x = a[0];
        msgImu.linear_acceleration.y = a[1];
        msgImu.linear_acceleration.z = a[2];
        msgImu.angular_velocity.x = v[0];
        msgImu.angular_velocity.y = v[1];
        msgImu.angular_velocity.z = v[2];
        _pubImu->publish(msgImu);

        std_msgs::msg::Float32 msgVoltagePwrMgmt;
        msgVoltagePwrMgmt.data = _pwr_mgmt->getVoltage(); // Returns dummy value
        _pubVoltagePwrMgmt->publish(msgVoltagePwrMgmt);

        std_msgs::msg::Float32 msgCurrentPwrMgmt;
        msgCurrentPwrMgmt.data = _pwr_mgmt->getCurrent(); // Returns dummy value
        _pubCurrentPwrMgmt->publish(msgCurrentPwrMgmt);
    }

    int EduDrive::gpio_write(const char *dev_name, int offset, int value)
    {
        // GPIO functions are likely not directly used for Dynamixel communication
        // but might be for other board features. Keep as is for now.
        return 0; // Dummy return
    }

    int EduDrive::gpio_read(const char *dev_name, int offset, int &value)
    {
        // GPIO functions are likely not directly used for Dynamixel communication
        // but might be for other board features. Keep as is for now.
        return 0; // Dummy return
    }

} // namespace
