#pragma once
#include <DynamixelSDK.h>

class SimpleMotorControl {
public:
    SimpleMotorControl(int ids[4]);
    void begin();
    void move(float linear_vel, float angular_vel);
    void stop();
    void setMotorVelocity(int id, int velocity);

private:
    int motor_ids[4];
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    void initDynamixel();
};
