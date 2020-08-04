#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include "motors.h"
#define RADIUS_OF_WHEEL 0.052      //m
#define WIDTH_BETWEEN_WHEELS 0.12  //m
#define MOTOR1_IN1 8
#define MOTOR1_IN2 9
#define MOTOR1_PWM 5
#define MOTOR1_ENCO_INT 2
#define MOTOR1_ENCO_DIGI 4
#define MOTOR2_IN1 12
#define MOTOR2_IN2 13
#define MOTOR2_PWM 10
#define MOTOR2_ENCO_INT 3
#define MOTOR2_ENCO_DIGI 7

void wheelSpeedR();
void wheelSpeedL();
void controlMotor();

EncoderMotor motor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, MOTOR1_ENCO_INT, MOTOR1_ENCO_DIGI);
ros::NodeHandle nh;
std_msgs::Int32 str_msg;
ros::Publisher pub_msg("/state", &str_msg);
void velCB(const geometry_msgs::Twist &msg) {
    motor.setTarget(msg.linear.x/0.026);
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velCB);

void setup() {
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
    TIMSK2 = (1 << OCIE2A);
    OCR2A = 125;  // 제어주기 8ms
    sei();

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub_msg);

    attachInterrupt(0, wheelSpeedR, CHANGE);
//    attachInterrupt(1, wheelSpeedL, CHANGE);

    motor.setMaxPWM(160);
    motor.setControlPeriod(8);
}

void loop() {
    nh.spinOnce();
}

ISR(TIMER2_COMPA_vect) {
    motor.control();
    str_msg.data = motor.getPWM();
    pub_msg.publish(&str_msg);
    // driver.calPose();
}

void wheelSpeedR() {
    motor.counter();
}

void wheelSpeedL() {
}
