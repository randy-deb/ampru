#include <ros.h>
#include <Arduino.h>
#include <ampru_msgs/WheelControl.h>

ros::NodeHandle nh;
int LWHEEL_PWR = 10;	// left wheel power (0-255)
int LWHEEL_FWD = 4;	// left wheel forward enabled
int LWHEEL_BWD = 5;	// left wheel backward enabled
int RWHEEL_PWR = 11;	// right wheel power (0-255)
int RWHEEL_FWD = 6;	// right wheel forward enabled
int RWHEEL_BWD = 7;	// right wheel backward enabled

void wheelControlCallback(const ampru_msgs::WheelControl& wheel_control_msg)
{
	// Control the left wheel
	if (wheel_control_msg.left_wheel_speed > 0)
	{
		digitalWrite(LWHEEL_FWD, HIGH);
		digitalWrite(LWHEEL_BWD, LOW);
		analogWrite(LWHEEL_PWR, (int)(wheel_control_msg.left_wheel_speed * 255));
	}
	else if (wheel_control_msg.left_wheel_speed < 0)
	{
		digitalWrite(LWHEEL_FWD, LOW);
		digitalWrite(LWHEEL_BWD, HIGH);
		analogWrite(LWHEEL_PWR, (int)(-wheel_control_msg.left_wheel_speed * 255));
	}
	else
	{
		analogWrite(LWHEEL_PWR, 0);
		digitalWrite(LWHEEL_FWD, LOW);
		digitalWrite(LWHEEL_BWD, LOW);
	}

	// Control the right wheel
	if (wheel_control_msg.right_wheel_speed > 0)
	{
		digitalWrite(RWHEEL_FWD, HIGH);
		digitalWrite(RWHEEL_BWD, LOW);
		analogWrite(RWHEEL_PWR, (int)(wheel_control_msg.right_wheel_speed * 255));
	}
	else if (wheel_control_msg.right_wheel_speed < 0)
	{
		digitalWrite(RWHEEL_FWD, LOW);
		digitalWrite(RWHEEL_BWD, HIGH);
		analogWrite(RWHEEL_PWR, (int)(-wheel_control_msg.right_wheel_speed * 255));
	}
	else
	{
		analogWrite(RWHEEL_PWR, 0);
		digitalWrite(RWHEEL_FWD, LOW);
		digitalWrite(RWHEEL_BWD, LOW);
	}
}

ros::Subscriber<ampru_msgs::WheelControl> wheelControlSubscriber("wheel_control", &wheelControlCallback);

void setup()
{
	// Setup the wheel pins
	pinMode(LWHEEL_PWR, OUTPUT);
	pinMode(LWHEEL_FWD, OUTPUT);
	pinMode(LWHEEL_BWD, OUTPUT);
	pinMode(RWHEEL_PWR, OUTPUT);
	pinMode(RWHEEL_FWD, OUTPUT);
	pinMode(RWHEEL_BWD, OUTPUT);

	// Setup the ROS node
	nh.initNode();
	nh.subscribe(wheelControlSubscriber);
}

void loop()
{
	nh.spinOnce();
	delay(1000);
}

