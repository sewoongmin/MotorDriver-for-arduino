#ifndef DRIVER_H_
#define DRIVER_H_

#include "Arduino.h"
#include "motors.h"



class Driver {
   private:
    PWMMotor motor_r_;
    PWMMotor motor_l_;
    Encoder encoder_r_;
    Encoder encoder_l_;

    double vel_r_;
    double vel_l_;
    double pose_x_;
    double pose_y_;
    double theta_;

    double target_speed_r_;
    double target_speed_l_;

   public:
    Driver() = delete;
    Driver(const int, const int, const int, const int, const int, const int, const int, const int, const int, const int);
    ~Driver();
    Driver &pinSetup();
    void setVelocity(double, double);
    void wheelSpeedR();
    void wheelSpeedL();
    void calPose();
    void controlR();
    void controlL();
    double getVelocity();
    double getVelocityR();
    double getVelocityL();
    double getPoseX();
    double getPoseY();
    double getTheta();
    int getCntR();
    int getCntL();
    double getPwmR();
    double getPwmL();
};

#endif
