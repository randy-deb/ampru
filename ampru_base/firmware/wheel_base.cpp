
//-------------------------------------------------------------------
// Included header files
//-------------------------------------------------------------------
#include <Arduino.h>
#include "SerialHDLC.h"

//-------------------------------------------------------------------
// Defines
//-------------------------------------------------------------------
#define LWHEEL_ENC_PIN 2	// left wheel encoder
#define LWHEEL_PWR_PIN 6	// left wheel power (0-255)
#define LWHEEL_FWD_PIN 4	// left wheel forward enabled
#define LWHEEL_BWD_PIN 5	// left wheel backward enabled
#define RWHEEL_ENC_PIN 3	// right wheel encoder
#define RWHEEL_PWR_PIN 9	// right wheel power (0-255)
#define RWHEEL_FWD_PIN 7	// right wheel forward enabled
#define RWHEEL_BWD_PIN 8	// right wheel backward enabled

#define WHEEL_DIR_NONE	0
#define WHEEL_DIR_FWD	1
#define WHEEL_DIR_BWD	2

//-------------------------------------------------------------------
// Forward method declarations
//-------------------------------------------------------------------
void onHandleFrame(const uint8_t* frameBuffer, uint16_t frameLength);
void setWheelControl(const uint8_t* frameBuffer, uint16_t frameLength);
void getWheelEncoder(const uint8_t* frameBuffer, uint16_t frameLength);
uint8_t getWheelEncoderResponse[8] = { 0x02, 0x70, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00 };

//-------------------------------------------------------------------
// Globals
//-------------------------------------------------------------------
SerialHDLC hdlc(onHandleFrame, 32);
volatile long lwheel_pulses = 0;
volatile long rwheel_pulses = 0;
long stored_lwheel_pulses = 0;
long stored_rwheel_pulses = 0;
uint8_t last_lwheel_dir_cmd = WHEEL_DIR_NONE;
uint8_t last_rwheel_dir_cmd = WHEEL_DIR_NONE;
uint8_t last_lwheel_dir = WHEEL_DIR_NONE;
uint8_t last_rwheel_dir = WHEEL_DIR_NONE;
long last_rpm_update_time = 0;
long lwheel_rpm = 0;
long rwheel_rpm = 0;
long last_lwheel_rpm = 0;
long last_rwheel_rpm = 0;
bool led_on = false;

void onHandleFrame(const uint8_t* frameBuffer, uint16_t frameLength)
{
	if (frameLength >= 2)
	{
		uint16_t cmd = (((uint16_t)frameBuffer[0]) | (((uint16_t)frameBuffer[1]) << 8));
		uint16_t len = (((uint16_t)frameBuffer[2]) | (((uint16_t)frameBuffer[3]) << 8));
		switch (cmd)
		{
			case 0x0001:
				setWheelControl(&frameBuffer[4], len);
				break;
			case 0x0002:
				getWheelEncoder(&frameBuffer[4], len);
				break;
			case 0x00AA:
				hdlc.write(frameBuffer, frameLength);
				break;
		}
	}
}

void setWheelControl(const uint8_t* frameBuffer, uint16_t frameLength)
{
	if (frameLength == 4)
	{
		uint8_t lwheel_dir = frameBuffer[0];
		uint8_t lwheel_pwr = frameBuffer[1];
		uint8_t rwheel_dir = frameBuffer[2];
		uint8_t rwheel_pwr = frameBuffer[3];

		if (lwheel_dir == WHEEL_DIR_FWD)
		{
			digitalWrite(LWHEEL_FWD_PIN, HIGH);
			digitalWrite(LWHEEL_BWD_PIN, LOW);
			analogWrite(LWHEEL_PWR_PIN, lwheel_pwr);
			lwheel_pulses += lwheel_pwr / 10;
		}
		else if (lwheel_dir == WHEEL_DIR_BWD)
		{
			digitalWrite(LWHEEL_FWD_PIN, LOW);
			digitalWrite(LWHEEL_BWD_PIN, HIGH);
			analogWrite(LWHEEL_PWR_PIN, lwheel_pwr);
			lwheel_pulses += lwheel_pwr / 10;
		}
		else
		{
			analogWrite(LWHEEL_PWR_PIN, 0);
			digitalWrite(LWHEEL_FWD_PIN, LOW);
			digitalWrite(LWHEEL_BWD_PIN, LOW);
		}

		if (rwheel_dir == WHEEL_DIR_FWD)
		{
			digitalWrite(RWHEEL_FWD_PIN, HIGH);
			digitalWrite(RWHEEL_BWD_PIN, LOW);
			analogWrite(RWHEEL_PWR_PIN, rwheel_pwr);
			rwheel_pulses += rwheel_pwr / 10;
		}
		else if (rwheel_dir == WHEEL_DIR_BWD)
		{
			digitalWrite(RWHEEL_FWD_PIN, LOW);
			digitalWrite(RWHEEL_BWD_PIN, HIGH);
			analogWrite(RWHEEL_PWR_PIN, rwheel_pwr);
			rwheel_pulses += rwheel_pwr / 10;
		}
		else
		{
			analogWrite(RWHEEL_PWR_PIN, 0);
			digitalWrite(RWHEEL_FWD_PIN, LOW);
			digitalWrite(RWHEEL_BWD_PIN, LOW);
		}
		last_lwheel_dir_cmd = lwheel_dir;
		last_rwheel_dir_cmd = rwheel_dir;
	}
}

void getWheelEncoder(const uint8_t* frameBuffer, uint16_t frameLength)
{
	long lwheel_encoder_data = stored_lwheel_pulses;
	long rwheel_encoder_data = stored_rwheel_pulses;
	if (last_lwheel_dir == WHEEL_DIR_BWD)
	{
		lwheel_encoder_data = -lwheel_encoder_data;
	}
	if (last_rwheel_dir = WHEEL_DIR_BWD)
	{
		rwheel_encoder_data = -rwheel_encoder_data;
	}
	getWheelEncoderResponse[4] = (((uint16_t)lwheel_encoder_data) & 0xFF);
	getWheelEncoderResponse[5] = (((uint16_t)lwheel_encoder_data >> 8) & 0xFF);
	getWheelEncoderResponse[6] = (((uint16_t)rwheel_encoder_data) & 0xFF);
	getWheelEncoderResponse[7] = (((uint16_t)rwheel_encoder_data >> 8) & 0xFF);
	
	hdlc.write(getWheelEncoderResponse, 8);
}

void lwheelEncoderPulse()
{
	lwheel_pulses++;
}

void rwheelEncoderPulse()
{
	rwheel_pulses++;
}

void setup()
{
	// Setup the wheel wheel/encoder pins
	//pinMode(LWHEEL_ENC_PIN, INPUT);
	pinMode(LWHEEL_PWR_PIN, OUTPUT);
	pinMode(LWHEEL_FWD_PIN, OUTPUT);
	pinMode(LWHEEL_BWD_PIN, OUTPUT);
	//pinMode(RWHEEL_ENC_PIN, INPUT);
	pinMode(RWHEEL_PWR_PIN, OUTPUT);
	pinMode(RWHEEL_FWD_PIN, OUTPUT);
	pinMode(RWHEEL_BWD_PIN, OUTPUT);
	pinMode(13, OUTPUT);
	
	// Setup the wheel encoder interrupts
	//attachInterrupt(0, lwheelEncoderPulse, FALLING);
	//attachInterrupt(1, rwheelEncoderPulse, FALLING);

	Serial.begin(9600);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
}

void loop()
{
	// Get the current time
	long cur_time = millis();

	// store the wheel encoder pulses and calcule wheel rpm every 100ms
	if (cur_time - last_rpm_update_time > 100)
	{
		// Store the wheel encoder pulses
		stored_lwheel_pulses = lwheel_pulses;
		stored_rwheel_pulses = rwheel_pulses;
		lwheel_pulses -= stored_lwheel_pulses;
		rwheel_pulses -= stored_rwheel_pulses;

		// Calculate the rpm
		last_lwheel_rpm = lwheel_rpm;
		last_rwheel_rpm = rwheel_rpm;
		lwheel_rpm = (stored_lwheel_pulses * 60) / 20; // 60 sec, 20 pulses per rotation
		rwheel_rpm = (stored_rwheel_pulses * 60) / 20; // 60 sec, 20 pulses per rotation

		// Check if we want to change the left wheel direction
		if (last_lwheel_dir_cmd != last_lwheel_dir)
		{
			// Check if the wheel has stopped or if the rpm is increasing
			if ((last_lwheel_dir_cmd == WHEEL_DIR_NONE && lwheel_rpm == 0) || (lwheel_rpm > last_lwheel_rpm))
			{
				last_lwheel_dir = last_lwheel_dir_cmd;
			}
		}

		// Check if we want to change the right wheel direction
		if (last_rwheel_dir_cmd != last_rwheel_dir)
		{
			// Check if the wheel has stopped or if the rpm is increasing
			if ((last_rwheel_dir_cmd == WHEEL_DIR_NONE && rwheel_rpm == 0) || (rwheel_rpm > last_rwheel_rpm))
			{
				last_rwheel_dir = last_rwheel_dir_cmd;
			}
		}

		if (led_on)
		{
			digitalWrite(13, HIGH);
			led_on = false;
		}
		else
		{
			digitalWrite(13, LOW);
			led_on = true;
		}
	}

	// Read incomming messages
	hdlc.read();
}

