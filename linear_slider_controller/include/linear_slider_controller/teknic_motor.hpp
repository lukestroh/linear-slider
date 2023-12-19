#ifndef __TEKNIC_MOTOR_HPP__
#define __TEKNIC_MOTOR_HPP__

#include <string>
// #include <cmath>

class TeknicMotor {
    public:
        std::string name;
        double vel_cmd = 0;
        double rpm_cmd = 0;
        double rpm_state = 0;
        double vel_state = 0;

        TeknicMotor();
        TeknicMotor(const std::string& motor_name);
        ~TeknicMotor();

        void begin(const std::string& motor_name);
        double rpm_to_vel(int& rpm);
        int vel_to_rpm(double& vel);

};

#endif // __TEKNIC_MOTOR_HPP__