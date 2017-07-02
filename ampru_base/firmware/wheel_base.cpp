
//-------------------------------------------------------------------
// Included header files
//-------------------------------------------------------------------
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "SerialHDLC.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#	include "Wire.h"
#endif

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
void getMPUData(const uint8_t* frameBuffer, uint16_t frameLength);

//-------------------------------------------------------------------
// Serial communication globals
//-------------------------------------------------------------------
SerialHDLC hdlc(onHandleFrame, 32);
uint8_t getWheelEncoderResponse[8] = { 0x02, 0x70, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t getMPU6050Response[16] = { 0x03, 0x70, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//-------------------------------------------------------------------
// Wheel globals
//-------------------------------------------------------------------
volatile uint16_t lwheel_pulses = 0;
volatile uint16_t lwheel_pulses_out = 0;
volatile uint32_t lwheel_pulse_time = 0;
volatile uint16_t rwheel_pulses = 0;
volatile uint16_t rwheel_pulses_out = 0;
volatile uint32_t rwheel_pulse_time = 0;
uint16_t stored_lwheel_pulses = 0;
uint16_t stored_rwheel_pulses = 0;
uint8_t last_lwheel_dir_cmd = WHEEL_DIR_NONE;
uint8_t last_rwheel_dir_cmd = WHEEL_DIR_NONE;
uint8_t last_lwheel_dir = WHEEL_DIR_NONE;
uint8_t last_rwheel_dir = WHEEL_DIR_NONE;
uint32_t last_rpm_update_time = 0;
uint32_t lwheel_rpm = 0;
uint32_t rwheel_rpm = 0;
uint32_t last_lwheel_rpm = 0;
uint32_t last_rwheel_rpm = 0;

//-------------------------------------------------------------------
// MPU6050 globals
//-------------------------------------------------------------------
MPU6050 mpu;
int16_t mpu_ax;
int16_t mpu_ay;
int16_t mpu_az;
int16_t mpu_gx;
int16_t mpu_gy;
int16_t mpu_gz;
bool mpu_connected = false;

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
			case 0x0003:
				getMPUData(&frameBuffer[4], len);
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
		}
		else if (lwheel_dir == WHEEL_DIR_BWD)
		{
			digitalWrite(LWHEEL_FWD_PIN, LOW);
			digitalWrite(LWHEEL_BWD_PIN, HIGH);
			analogWrite(LWHEEL_PWR_PIN, lwheel_pwr);
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
		}
		else if (rwheel_dir == WHEEL_DIR_BWD)
		{
			digitalWrite(RWHEEL_FWD_PIN, LOW);
			digitalWrite(RWHEEL_BWD_PIN, HIGH);
			analogWrite(RWHEEL_PWR_PIN, rwheel_pwr);
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
	int16_t lwheel_encoder_data = (int16_t)lwheel_pulses_out;
	int16_t rwheel_encoder_data = (int16_t)rwheel_pulses_out;
	lwheel_pulses_out -= lwheel_encoder_data;
	rwheel_pulses_out -= rwheel_encoder_data;
	if (last_lwheel_dir == WHEEL_DIR_BWD)
	{
		lwheel_encoder_data = -lwheel_encoder_data;
	}
	if (last_rwheel_dir == WHEEL_DIR_BWD)
	{
		rwheel_encoder_data = -rwheel_encoder_data;
	}

	getWheelEncoderResponse[4] = (uint8_t)((lwheel_encoder_data) & 0xFF);
	getWheelEncoderResponse[5] = (uint8_t)((lwheel_encoder_data >> 8) & 0xFF);
	getWheelEncoderResponse[6] = (uint8_t)((rwheel_encoder_data) & 0xFF);
	getWheelEncoderResponse[7] = (uint8_t)((rwheel_encoder_data >> 8) & 0xFF);
	
	hdlc.write(getWheelEncoderResponse, 8);
}

void getMPUData(const uint8_t* frameBuffer, uint16_t frameLength)
{
	if (mpu_connected)
	{
		mpu.getMotion6(&mpu_ax, &mpu_ay, &mpu_az, &mpu_gx, &mpu_gy, &mpu_gz);
		getMPU6050Response[4] = (uint8_t)((mpu_ax) & 0xFF);
		getMPU6050Response[5] = (uint8_t)((mpu_ax >> 8) & 0xFF);
		getMPU6050Response[6] = (uint8_t)((mpu_ay) & 0xFF);
		getMPU6050Response[7] = (uint8_t)((mpu_ay >> 8) & 0xFF);
		getMPU6050Response[8] = (uint8_t)((mpu_az) & 0xFF);
		getMPU6050Response[9] = (uint8_t)((mpu_az >> 8) & 0xFF);
		getMPU6050Response[10] = (uint8_t)((mpu_gx) & 0xFF);
		getMPU6050Response[11] = (uint8_t)((mpu_gx >> 8) & 0xFF);
		getMPU6050Response[12] = (uint8_t)((mpu_gy) & 0xFF);
		getMPU6050Response[13] = (uint8_t)((mpu_gy >> 8) & 0xFF);
		getMPU6050Response[14] = (uint8_t)((mpu_gz) & 0xFF);
		getMPU6050Response[15] = (uint8_t)((mpu_gz >> 8) & 0xFF);
	}
	else
	{
		getMPU6050Response[4] = 0x00;
		getMPU6050Response[5] = 0x00;
		getMPU6050Response[6] = 0x00;
		getMPU6050Response[7] = 0x00;
		getMPU6050Response[8] = 0x00;
		getMPU6050Response[9] = 0x00;
		getMPU6050Response[10] = 0x00;
		getMPU6050Response[11] = 0x00;
		getMPU6050Response[12] = 0x00;
		getMPU6050Response[13] = 0x00;
		getMPU6050Response[14] = 0x00;
		getMPU6050Response[15] = 0x00;
	}

	hdlc.write(getMPU6050Response, 16);
}

void lwheelEncoderPulse()
{
	uint32_t current_time = micros();
	if (digitalRead(LWHEEL_ENC_PIN) && ((current_time - lwheel_pulse_time) > 100))
	{
		lwheel_pulses++;
		lwheel_pulses_out++;
		lwheel_pulse_time = current_time;
	}
}

void rwheelEncoderPulse()
{
	uint32_t current_time = micros();
	if (digitalRead(RWHEEL_ENC_PIN) && ((current_time - rwheel_pulse_time) > 100))
	{
		rwheel_pulses++;
		rwheel_pulses_out++;
		rwheel_pulse_time = current_time;
	}
}

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	// Setup the wheel wheel/encoder pins
	pinMode(LWHEEL_ENC_PIN, INPUT);
	pinMode(LWHEEL_PWR_PIN, OUTPUT);
	pinMode(LWHEEL_FWD_PIN, OUTPUT);
	pinMode(LWHEEL_BWD_PIN, OUTPUT);
	pinMode(RWHEEL_ENC_PIN, INPUT);
	pinMode(RWHEEL_PWR_PIN, OUTPUT);
	pinMode(RWHEEL_FWD_PIN, OUTPUT);
	pinMode(RWHEEL_BWD_PIN, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	
	// Setup the wheel encoder interrupts
	attachInterrupt(0, lwheelEncoderPulse, FALLING);
	attachInterrupt(1, rwheelEncoderPulse, FALLING);

	// Initialize the MPU6050
	mpu.initialize();
	mpu_connected = mpu.testConnection();

	// Set the mpu offsets
	mpu.setXAccelOffset(-785);
    mpu.setYAccelOffset(2389);
    mpu.setZAccelOffset(1549);
    mpu.setXGyroOffset(-165);
    mpu.setYGyroOffset(44);
    mpu.setZGyroOffset(-22);

	// Start the serial communication
	Serial.begin(9600);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
}

void loop()
{
	// Get the current time
	uint32_t cur_time = millis();

	// store the wheel encoder pulses and calcule wheel rpm every 100ms
	if (cur_time - last_rpm_update_time > 10)
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
	}

	// Read incomming messages
	hdlc.read();
}
