#include <ros.h>
#include <Arduino.h>
#include <ampru_msgs/WheelControl.h>

ros::NodeHandle nh;

void wheelControlCallback(const ampru_msgs::WheelControl& wheel_control_msg)
{

}

ros::Subscriber<ampru_msgs::WheelControl> wheelControlSubscriber("wheel_control", &wheelControlCallback);

void setup()
{
	nh.initNode();
	nh.subscribe(wheelControlSubscriber);
}

void loop()
{
	nh.spinOnce();
	delay(1000);
}
