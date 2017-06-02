#include <ros.h>
#include <Arduino.h>
#include <NewPing.h>
#include <sensor_msgs/Range.h>
#include <ampru_msgs/WheelControl.h>

ros::NodeHandle nh;

#define LWHEEL_PWR_PIN 10	// left wheel power (0-255)
#define LWHEEL_FWD_PIN 4	// left wheel forward enabled
#define LWHEEL_BWD_PIN 5	// left wheel backward enabled
#define RWHEEL_PWR_PIN 11	// right wheel power (0-255)
#define RWHEEL_FWD_PIN 6	// right wheel forward enabled
#define RWHEEL_BWD_PIN 7	// right wheel backward enabled

#define PING_TRIG_PIN 2
#define PING_ECHO_PIN 3
#define PING_MAX_DIST 200

NewPing sonar(PING_TRIG_PIN, PING_ECHO_PIN, PING_MAX_DIST);
unsigned int pingSpeed = 250;
unsigned long pingTimer;

char range_frameid[] = "/us_ranger";
sensor_msgs::Range range_msg;
ros::Publisher rangeDataPublisher("range_data", &range_msg);

void wheelControlCallback(const ampru_msgs::WheelControl& wheel_control_msg)
{
	// Control the left wheel
	if (wheel_control_msg.left_wheel_speed > 0)
	{
		digitalWrite(LWHEEL_FWD_PIN, HIGH);
		digitalWrite(LWHEEL_BWD_PIN, LOW);
		analogWrite(LWHEEL_PWR_PIN, (int)(wheel_control_msg.left_wheel_speed * 255));
	}
	else if (wheel_control_msg.left_wheel_speed < 0)
	{
		digitalWrite(LWHEEL_FWD_PIN, LOW);
		digitalWrite(LWHEEL_BWD_PIN, HIGH);
		analogWrite(LWHEEL_PWR_PIN, (int)(-wheel_control_msg.left_wheel_speed * 255));
	}
	else
	{
		analogWrite(LWHEEL_PWR_PIN, 0);
		digitalWrite(LWHEEL_FWD_PIN, LOW);
		digitalWrite(LWHEEL_BWD_PIN, LOW);
	}

	// Control the right wheel
	if (wheel_control_msg.right_wheel_speed > 0)
	{
		digitalWrite(RWHEEL_FWD_PIN, HIGH);
		digitalWrite(RWHEEL_BWD_PIN, LOW);
		analogWrite(RWHEEL_PWR_PIN, (int)(wheel_control_msg.right_wheel_speed * 255));
	}
	else if (wheel_control_msg.right_wheel_speed < 0)
	{
		digitalWrite(RWHEEL_FWD_PIN, LOW);
		digitalWrite(RWHEEL_BWD_PIN, HIGH);
		analogWrite(RWHEEL_PWR_PIN, (int)(-wheel_control_msg.right_wheel_speed * 255));
	}
	else
	{
		analogWrite(RWHEEL_PWR_PIN, 0);
		digitalWrite(RWHEEL_FWD_PIN, LOW);
		digitalWrite(RWHEEL_BWD_PIN, LOW);
	}
}

void echoCheck()
{
  if (sonar.check_timer())
  {
	  range_msg.range = sonar.ping_result / US_ROUNDTRIP_CM;
	  range_msg.header.stamp = nh.now();
	  rangeDataPublisher.publish(&range_msg);
  }
}

ros::Subscriber<ampru_msgs::WheelControl> wheelControlSubscriber("wheel_control", &wheelControlCallback);

void setup()
{
	// Setup the wheel pins
	pinMode(LWHEEL_PWR_PIN, OUTPUT);
	pinMode(LWHEEL_FWD_PIN, OUTPUT);
	pinMode(LWHEEL_BWD_PIN, OUTPUT);
	pinMode(RWHEEL_PWR_PIN, OUTPUT);
	pinMode(RWHEEL_FWD_PIN, OUTPUT);
	pinMode(RWHEEL_BWD_PIN, OUTPUT);

	// Setup the ping timer
	pingTimer = millis();

	// Setup the ROS node
	nh.initNode();
	nh.getHardware()->setBaud(57600);
	nh.subscribe(wheelControlSubscriber);
	nh.advertise(rangeDataPublisher);
	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.header.frame_id = range_frameid;
	range_msg.field_of_view = 0.01;
	range_msg.min_range = 0.0;
	range_msg.max_range = 0.5;
}

void loop()
{
	if (millis() >= pingTimer)
	{
		pingTimer += pingSpeed;
		sonar.ping_timer(echoCheck);
	}

	nh.spinOnce();
	delay(10);
}

