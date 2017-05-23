
#ifndef ___AMPRU_BASE_AMPRU_HARDWARE_H___
#define ___AMPRU_BASE_AMPRU_HARDWARE_H___

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace ampru_base
{
    class AmpruHardware : public hardware_interface::RobotHW
    {
    public:
        AmpruHardware(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        void registerControlInterface();
        void updateJointsFromHardware();
        void writeCommandsToHardware();
        
    private:
        void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right);
        double angularToLinear(const double &angle) const;

    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _private_nh;
        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::VelocityJointInterface _velocity_joint_interface;
        double _wheel_diameter;
        double _max_speed;
        double _cmd[2];
        double _pos[2];
        double _vel[2];
        double _eff[2];

    };
}

#endif // ___AMPRU_BASE_AMPRU_HARDWARE_H___
