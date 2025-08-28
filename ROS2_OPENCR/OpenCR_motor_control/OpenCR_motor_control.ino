#include "SimpleMotorControl.h"

int motor_ids[4] = {1, 2, 3, 4};
SimpleMotorControl motor(motor_ids);

void setup() {
    motor.begin();
}

void loop() {
    motor.move(50, 0); // 직진
    delay(1000);
    motor.move(0, 50); // 제자리 회전
    delay(1000);
    motor.stop();
    delay(1000);
}
