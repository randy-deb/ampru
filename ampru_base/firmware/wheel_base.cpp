
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
bool ledOn = false;

//-------------------------------------------------------------------
// Forward method declarations
//-------------------------------------------------------------------
void onHandleFrame(const uint8_t* frameBuffer, uint16_t frameLength);
void setMotorControl(const uint8_t* frameBuffer, uint16_t frameLength);
void getMotorEncoder(const uint8_t* frameBuffer, uint16_t frameLength);
uint8_t getMotorEncoderResponse[6] = { 0x02, 0x70, 0x00, 0x02, 0x00, 0x00 };

//-------------------------------------------------------------------
// Globals
//-------------------------------------------------------------------
SerialHDLC hdlc(onHandleFrame, 32);
volatile uint8_t lwheel_pulses = 0;
volatile uint8_t rwheel_pulses = 0;

void onHandleFrame(const uint8_t* frameBuffer, uint16_t frameLength)
{
	if (frameLength >= 2)
	{
		uint16_t cmd = (((uint16_t)frameBuffer[0]) | (((uint16_t)frameBuffer[1]) << 8));
		uint16_t len = (((uint16_t)frameBuffer[2]) | (((uint16_t)frameBuffer[3]) << 8));
		switch (cmd)
		{
			case 0x0001:
				setMotorControl(&frameBuffer[4], len);
				break;
			case 0x0002:
				getMotorEncoder(&frameBuffer[4], len);
				break;
			case 0x00AA:
				hdlc.write(frameBuffer, frameLength);
				break;
		}
	}
}

void setMotorControl(const uint8_t* frameBuffer, uint16_t frameLength)
{
	if (frameLength == 4)
	{
		uint8_t lmotorDir = frameBuffer[0];
		uint8_t lmotorPwr = frameBuffer[1];
		uint8_t rmotorDir = frameBuffer[2];
		uint8_t rmotorPwr = frameBuffer[3];

		if (lmotorDir == 0)
		{
			analogWrite(LWHEEL_PWR_PIN, 0);
			digitalWrite(LWHEEL_FWD_PIN, LOW);
			digitalWrite(LWHEEL_BWD_PIN, LOW);
		}
		else if (lmotorDir == 1)
		{
			digitalWrite(LWHEEL_FWD_PIN, HIGH);
			digitalWrite(LWHEEL_BWD_PIN, LOW);
			analogWrite(LWHEEL_PWR_PIN, lmotorPwr);
		}
		else if (lmotorDir == 2)
		{
			digitalWrite(LWHEEL_FWD_PIN, LOW);
			digitalWrite(LWHEEL_BWD_PIN, HIGH);
			analogWrite(LWHEEL_PWR_PIN, lmotorPwr);
		}

		if (rmotorDir == 0)
		{
			analogWrite(RWHEEL_PWR_PIN, 0);
			digitalWrite(RWHEEL_FWD_PIN, LOW);
			digitalWrite(RWHEEL_BWD_PIN, LOW);
		}
		else if (rmotorDir == 1)
		{
			digitalWrite(RWHEEL_FWD_PIN, HIGH);
			digitalWrite(RWHEEL_BWD_PIN, LOW);
			analogWrite(RWHEEL_PWR_PIN, rmotorPwr);
		}
		else if (rmotorDir == 2)
		{
			digitalWrite(RWHEEL_FWD_PIN, LOW);
			digitalWrite(RWHEEL_BWD_PIN, HIGH);
			analogWrite(RWHEEL_PWR_PIN, rmotorPwr);
		}
	}
}

void getMotorEncoder(const uint8_t* frameBuffer, uint16_t frameLength)
{
	const uint8_t last_lwheel_pulses = lwheel_pulses;
	const uint8_t last_rwheel_pulses = rwheel_pulses;
	lwheel_pulses -= last_lwheel_pulses;
	rwheel_pulses -= last_rwheel_pulses;
	getMotorEncoderResponse[4] = last_lwheel_pulses;
	getMotorEncoderResponse[5] = last_rwheel_pulses;
	hdlc.write(getMotorEncoderResponse, 6);
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
	// Setup the wheel motor/encoder pins
	pinMode(LWHEEL_ENC_PIN, INPUT);
	pinMode(LWHEEL_PWR_PIN, OUTPUT);
	pinMode(LWHEEL_FWD_PIN, OUTPUT);
	pinMode(LWHEEL_BWD_PIN, OUTPUT);
	pinMode(RWHEEL_ENC_PIN, INPUT);
	pinMode(RWHEEL_PWR_PIN, OUTPUT);
	pinMode(RWHEEL_FWD_PIN, OUTPUT);
	pinMode(RWHEEL_BWD_PIN, OUTPUT);
	pinMode(13, OUTPUT);
	
	// Setup the wheel encoder interrupts
	attachInterrupt(0, lwheelEncoderPulse, FALLING);
	attachInterrupt(1, rwheelEncoderPulse, FALLING);

	Serial.begin(9600);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
}

void loop()
{
	// Read incomming messages
	hdlc.read();
}

