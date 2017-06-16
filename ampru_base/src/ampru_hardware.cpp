
#include "ampru_msgs/WheelControl.h"
#include "ampru_base/ampru_hardware.h"

ampru_base::AmpruHardware::AmpruHardware(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : _nh(nh)
    , _private_nh(private_nh)
{
    _private_nh.param<double>("wheel_diameter", _wheel_diameter, 0.35);
    _private_nh.param<double>("max_speed", _max_speed, 1.0);

    openSerial();
    registerControlInterface();
}

ampru_base::AmpruHardware::~AmpruHardware()
{
    closeSerial();
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
	ampru_base::GetWheelEncoder getWheelEncoder;
	_serialPort.sendMessage(&getWheelEncoder);

	auto wheelEncoderData = (WheelEncoderData*)_serialPort.waitMessage(WheelEncoderData::MESSAGE_TYPE, 1.0);
	if (wheelEncoderData != NULL)
	{
		ROS_INFO_STREAM("Wheel encoder: { " << wheelEncoderData->getLeftPulses() << ", " << wheelEncoderData->getRightPulses() << " }");
	}
	else
	{
		ROS_INFO_STREAM("No wheel encoder data received");
	}
}

void ampru_base::AmpruHardware::writeCommandsToHardware()
{
    double diff_speed_left = angularToLinear(_cmd[0]);
    double diff_speed_right = angularToLinear(_cmd[1]);
    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

	ampru_base::SetMotorSpeed setMotorSpeed(diff_speed_left, diff_speed_right);
	_serialPort.sendMessage(&setMotorSpeed);
}

bool ampru_base::AmpruHardware::openSerial()
{
	_serialPort.setPort("/dev/ttyUSB0");
	_serialPort.openConnection();
}

void ampru_base::AmpruHardware::closeSerial()
{
	_serialPort.closeConnection();
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
