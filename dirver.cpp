//#include "driver.h"
//
//Driver::Driver(const int motor1_pin1, const int motor1_pin2, const int motor1_pwm, const int encoder1_int, const int encoder1_digi, const int motor2_pin1, const int motor2_pin2, const int motor2_pwm, const int encoder2_int, const int encoder2_digi)
//    : motor_r_(motor1_pin1, motor1_pin2, motor1_pwm), motor_l_(motor2_pin1, motor2_pin2, motor2_pwm), encoder_r_(encoder1_int, encoder1_digi), encoder_l_(encoder2_int, encoder2_digi) {
//    vel_r_ = 0;
//    vel_l_ = 0;
//    pose_x_ = 0;
//    pose_y_ = 0;
//    theta_ = 0;
//    target_speed_r_ = 0;
//    target_speed_l_ = 0;
//}
//
//Driver::~Driver() {
//}
//
//void Driver::setVelocity(double linear_x, double angular_z) {
//    target_speed_r_ = linear_x + (angular_z * WIDTH_BETWEEN_WHEELS / 2);
//    target_speed_l_ = linear_x - (angular_z * WIDTH_BETWEEN_WHEELS / 2);
//}
//
//Driver &Driver::pinSetup() {
//  motor_r_.pinSetup();
//  motor_l_.pinSetup();
//  encoder_r_.pinSetup();
//  encoder_l_.pinSetup();
//}
//
//void Driver::calPose() {
//    int diff_r = (cnt_r_ - prev_cnt_r_pose_) * RADIUS_OF_WHEEL * PI / 1920;
//    int diff_l = (cnt_l_ - prev_cnt_l_pose_) * RADIUS_OF_WHEEL * PI / 1920;
//    int dist = (diff_r + diff_l) / 2;
//    int theta = (diff_r - diff_l) / WIDTH_BETWEEN_WHEELS;
//
//    theta_ += theta;
//    if (theta_ > PI * 2) {
//        theta_ -= (PI * 2);
//    } else if (theta_ < -(PI * 2)) {
//        theta_ += (PI * 2);
//    }
//    pose_x_ += (dist * cos(theta));
//    pose_y_ += (dist * sin(theta));
//}
//
//void Driver::controlR() {
//    if (target_speed_r_ > 0) {
//        digitalWrite(MOTOR1_IN1, HIGH);
//        digitalWrite(MOTOR1_IN2, LOW);
//    } else if (target_speed_r_ < 0) {
//        digitalWrite(MOTOR1_IN1, LOW);
//        digitalWrite(MOTOR1_IN2, HIGH);
//    } else {
//        digitalWrite(MOTOR1_IN1, LOW);
//        digitalWrite(MOTOR1_IN2, LOW);
//    }
//    vel_r_ = (cnt_r_ - prev_cnt_r_) * RADIUS_OF_WHEEL * PI / 15.36;  // 15.36 = 125 / 1920 여기서 125은 8ms의 역수 1920은 기어비 120에 엔코더 resolution 16펄스를 곱한 것
//    if (fabs(target_speed_r_) > fabs(vel_r_))
//        pwm_r_++;
//    else if (fabs(target_speed_r_) < fabs(vel_r_))
//        pwm_r_--;
//
//    if (pwm_r_ > 160)
//        pwm_r_ = 160;
//    else if (pwm_r_ < 0)
//        pwm_r_ = 0;
//
//    analogWrite(MOTOR1_PWM, pwm_r_);
//
//    if (cnt_r_ > 30000)
//        cnt_r_ = 0;
//    else if (cnt_r_ < -30000)
//        cnt_r_ = 0;
//
//    prev_cnt_r_ = cnt_r_;
//}
//
//void Driver::controlL() {
//    if (target_speed_l_ > 0) {
//        digitalWrite(MOTOR2_IN1, LOW);
//        digitalWrite(MOTOR2_IN2, HIGH);
//    } else if (target_speed_l_ < 0) {
//        digitalWrite(MOTOR2_IN1, HIGH);
//        digitalWrite(MOTOR2_IN2, LOW);
//    } else {
//        digitalWrite(MOTOR2_IN1, LOW);
//        digitalWrite(MOTOR2_IN2, LOW);
//    }
//    vel_l_ = -(cnt_l_ - prev_cnt_l_) * RADIUS_OF_WHEEL * PI / 15.36;  // 15.36 = 125 / 1920 여기서 125은 8ms의 역수 1920은 기어비 120에 엔코더 resolution 16펄스를 곱한 것
//    if (fabs(target_speed_l_) > fabs(vel_l_))
//        pwm_l_++;
//    else if (fabs(target_speed_l_) < fabs(vel_l_))
//        pwm_l_--;
//
//    if (pwm_l_ > 160)
//        pwm_l_ = 160;
//    else if (pwm_l_ < 0)
//        pwm_l_ = 0;
//    analogWrite(MOTOR2_PWM, pwm_l_);
//
//    if (cnt_l_ > 30000)
//        cnt_l_ = 0;
//    else if (cnt_l_ < -30000)
//        cnt_l_ = 0;
//
//    prev_cnt_l_ = cnt_l_;
//}
//
//double Driver::getVelocity() {
//    double vel = (vel_r_ + vel_l_) / 2;
//    return vel;
//}
//
//double Driver::getVelocityR() {
//    return vel_r_;
//}
//
//double Driver::getVelocityL() {
//    return vel_l_;
//}
//
//double Driver::getPoseX() {
//    return pose_x_;
//}
//
//double Driver::getPoseY() {
//    return pose_y_;
//}
//
//double Driver::getTheta() {
//    return theta_;
//}
//
//int Driver::getCntR() {
//    return cnt_r_;
//}
//
//int Driver::getCntL() {
//    return cnt_l_;
//}
//
//double Driver::getPwmR() {
//    return pwm_r_;
//}
//
//double Driver::getPwmL() {
//    return pwm_l_;
//}
//
//void Driver::wheelSpeedR() {
//    state_r_ = digitalRead(MOTOR1_ENCO_INT);
//    if ((prev_state_r_ == LOW) && state_r_ == HIGH) {
//        int val = digitalRead(MOTOR1_ENCO_DIGI);
//        if (val == LOW && direction_r_) {
//            direction_r_ = false;  //Reverse
//        } else if (val == HIGH && !direction_r_) {
//            direction_r_ = true;  //Forward
//        }
//    }
//    prev_state_r_ = state_r_;
//
//    if (!direction_r_)
//        cnt_r_++;
//    else
//        cnt_r_--;
//}
//
//void Driver::wheelSpeedL() {
//    state_l_ = digitalRead(MOTOR2_ENCO_INT);
//    if ((prev_state_l_ == LOW) && state_l_ == HIGH) {
//        int val = digitalRead(MOTOR2_ENCO_DIGI);
//        if (val == LOW && direction_l_) {
//            direction_l_ = false;  //Reverse
//        } else if (val == HIGH && !direction_l_) {
//            direction_l_ = true;  //Forward
//        }
//    }
//    prev_state_l_ = state_l_;
//
//    if (!direction_l_)
//        cnt_l_++;
//    else
//        cnt_l_--;
//}
