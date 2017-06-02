
#ifndef ___AMPRU_BASE_AMPRU_HARDWARE_H___
#define ___AMPRU_BASE_AMPRU_HARDWARE_H___

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
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
        void rangeDataCallback(const sensor_msgs::Range::ConstPtr& msg);
        void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right);
        double angularToLinear(const double &angle) const;

    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _private_nh;
        ros::Publisher _wheel_control_publisher;
        ros::Subscriber _range_data_subscriber;
        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::VelocityJointInterface _velocity_joint_interface;
        double _wheel_diameter;
        double _max_speed;
        double _cmd[2];
        double _pos[2];
        double _vel[2];
        double _eff[2];
        double _range;
    };
}

#endif // ___AMPRU_BASE_AMPRU_HARDWARE_H___
