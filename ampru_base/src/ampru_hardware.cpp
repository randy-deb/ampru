
#include "ampru_msgs/WheelControl.h"
#include "ampru_base/ampru_hardware.h"

ampru_base::AmpruHardware::AmpruHardware(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : _nh(nh)
    , _private_nh(private_nh)
{
    _private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.35);
    _private_nh.param<double>("max_speed", _max_speed, 1.0);

    _wheel_control_publisher = _nh.advertise<ampru_msgs::WheelControl>("wheel_control", 10);
    _range_data_subscriber = _nh.subscribe("range_data", 10, &AmpruHardware::rangeDataCallback, this);

    registerControlInterface();
}

void ampru_base::AmpruHardware::registerControlInterface()
{
    hardware_interface::JointStateHandle hWheelStateFL("front_left_wheel_joint",  &_pos[0], &_vel[0], &_eff[0]);
    hardware_interface::JointStateHandle hWheelStateFR("front_right_wheel_joint", &_pos[1], &_vel[1], &_eff[1]);
    _joint_state_interface.registerHandle(hWheelStateFL);
    _joint_state_interface.registerHandle(hWheelStateFR);

    hardware_interface::JointHandle hWheelFL(hWheelStateFL, &_cmd[0]);
    hardware_interface::JointHandle hWheelFR(hWheelStateFR, &_cmd[1]);
    _velocity_joint_interface.registerHandle(hWheelFL);
    _velocity_joint_interface.registerHandle(hWheelFR);

    registerInterface(&_joint_state_interface);
    registerInterface(&_velocity_joint_interface);
}

void ampru_base::AmpruHardware::updateJointsFromHardware()
{
    //TODO: Update joint states
}

void ampru_base::AmpruHardware::writeCommandsToHardware()
{
    double diff_speed_left = angularToLinear(_cmd[0]);
    double diff_speed_right = angularToLinear(_cmd[1]);
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    ampru_msgs::WheelControl wheel_control_msg;
    if (_range > 5.0)
    {
        wheel_control_msg.left_wheel_speed = (float)diff_speed_left;
        wheel_control_msg.right_wheel_speed = (float)diff_speed_right;
    }
    else
    {
        wheel_control_msg.left_wheel_speed = 0.0;
        wheel_control_msg.right_wheel_speed = 0.0;
    }
    _wheel_control_publisher.publish(wheel_control_msg);
}

void ampru_base::AmpruHardware::rangeDataCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    _range = (double)msg->range;
}

void ampru_base::AmpruHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
{
    double speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
    if (speed > _max_speed)
    {
        diff_speed_left *= _max_speed / speed;
        diff_speed_right *= _max_speed / speed;
    }
}

double ampru_base::AmpruHardware::angularToLinear(const double &angle) const
{
    return angle * _wheel_diameter / 2;
}
