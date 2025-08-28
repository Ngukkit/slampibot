#include "SimpleMotorControl.h"

SimpleMotorControl::SimpleMotorControl(int ids[4]) {
    for (int i = 0; i < 4; i++) motor_ids[i] = ids[i];
    portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
}

void SimpleMotorControl::begin() {
    initDynamixel();
}

void SimpleMotorControl::initDynamixel() {
    if (!portHandler->openPort()) {
        // 포트 열기 실패
    }
    if (!portHandler->setBaudRate(57600)) {
        // 보드레이트 설정 실패
    }
}

void SimpleMotorControl::move(float linear_vel, float angular_vel) {
    // 단순 예제: 좌우 속도 계산
    int left = int(linear_vel - angular_vel);
    int right = int(linear_vel + angular_vel);
    setMotorVelocity(motor_ids[0], left);
    setMotorVelocity(motor_ids[1], left);
    setMotorVelocity(motor_ids[2], right);
    setMotorVelocity(motor_ids[3], right);
}

void SimpleMotorControl::stop() {
    for (int i = 0; i < 4; i++)
        setMotorVelocity(motor_ids[i], 0);
}

void SimpleMotorControl::setMotorVelocity(int id, int velocity) {
    uint8_t dxl_error = 0;
    packetHandler->write2ByteTxRx(portHandler, id, 102, velocity, &dxl_error);
}
