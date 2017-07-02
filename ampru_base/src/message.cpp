
#include "ampru_base/message.h"
#include <ros/console.h>
#include <memory>
#include <cstring>

ampru_base::Message* ampru_base::Message::parse(const uint8_t* data, size_t length)
{
	uint16_t type = ((((uint16_t)data[1]) << 8) | (((uint16_t)data[0])));
	switch (type)
	{
	case EchoMessage::MESSAGE_TYPE:
		return new EchoMessage(data, length);
	case WheelEncoderData::MESSAGE_TYPE:
		return new WheelEncoderData(data, length);
	case MPUData::MESSAGE_TYPE:
		return new MPUData(data, length);
	default:
		return NULL;
	}
}

ampru_base::Message::Message(const uint8_t* data, size_t length)
	: _data(NULL)
	, _size(length)
{
	_data = (uint8_t*)malloc(_size);
	memcpy(_data, data, _size);
}

ampru_base::Message::Message(uint16_t messageType, size_t payloadLength)
	: _data(NULL)
	, _size(payloadLength + 4)
{
	_data = (uint8_t*)malloc(_size);
	_data[0] = messageType & 0xFF;
	_data[1] = (messageType >> 8) & 0xFF;
	_data[2] = payloadLength & 0xFF;
	_data[3] = (payloadLength >> 8) & 0xFF;
}

ampru_base::Message::~Message()
{
	if (_data != NULL)
	{
		free(_data);
	}
}

uint16_t ampru_base::Message::getType() const
{
	return (
	(((uint16_t)_data[1]) << 8) | 
	(((uint16_t)_data[0]))
	);
}

uint8_t* ampru_base::Message::getData() const
{
	return _data;
}

size_t ampru_base::Message::getSize() const
{
	return _size;
}

int8_t ampru_base::Message::getI8(size_t offset) const
{
	return (int8_t)getU8(offset);
}

int16_t ampru_base::Message::getI16(size_t offset) const
{
	return (int16_t)getU16(offset);
}

int32_t ampru_base::Message::getI32(size_t offset) const
{
	return (int32_t)getU32(offset);
}

uint8_t ampru_base::Message::getU8(size_t offset) const
{
	return _data[offset + 4];
}

uint16_t ampru_base::Message::getU16(size_t offset) const
{
	return (
	(((uint16_t)_data[offset + 5]) << 8) | 
	(((uint16_t)_data[offset + 4]))
	);
}

uint32_t ampru_base::Message::getU32(size_t offset) const
{
	return (
	(((uint32_t)_data[offset + 7]) << 24) | 
	(((uint32_t)_data[offset + 6]) << 16) | 
	(((uint32_t)_data[offset + 5]) << 8)  | 
	(((uint32_t)_data[offset + 4]))
	);
}

void ampru_base::Message::setI8(size_t offset, int8_t value)
{
	setU8(offset, (uint8_t)value);
}

void ampru_base::Message::setI16(size_t offset, int16_t value)
{
	setU16(offset, (uint16_t)value);
}

void ampru_base::Message::setI32(size_t offset, int32_t value)
{
	setU32(offset, (uint32_t)value);
}

void ampru_base::Message::setU8(size_t offset, uint8_t value)
{
	_data[offset + 4] = value;
}

void ampru_base::Message::setU16(size_t offset, uint16_t value)
{
	_data[offset + 4] = (uint8_t)((value) & 0xFF);
	_data[offset + 5] = (uint8_t)((value >> 8) & 0xFF);
}

void ampru_base::Message::setU32(size_t offset, uint32_t value)
{
	_data[offset + 4] = (uint8_t)((value) & 0xFF);
	_data[offset + 5] = (uint8_t)((value >> 8) & 0xFF);
	_data[offset + 6] = (uint8_t)((value >> 16) & 0xFF);
	_data[offset + 7] = (uint8_t)((value >> 24) & 0xFF);
}

ampru_base::SetMotorSpeed::SetMotorSpeed(double leftWheel, double rightWheel)
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
	if (leftWheel > 0.0)
	{
		setU8(L_DIR, 1);
		setU8(L_PWR, (uint8_t)(leftWheel * 255));
	}
	else if (leftWheel < 0.0)
	{
		setU8(L_DIR, 2);
		setU8(L_PWR, (uint8_t)(-leftWheel * 255));
	}
	else
	{
		setU8(L_DIR, 0);
		setU8(L_PWR, 0);
	}
	if (rightWheel > 0.0)
	{
		setU8(R_DIR, 1);
		setU8(R_PWR, (uint8_t)(rightWheel * 255));
	}
	else if (rightWheel < 0.0)
	{
		setU8(R_DIR, 2);
		setU8(R_PWR, (uint8_t)(-rightWheel * 255));
	}
	else
	{
		setU8(R_DIR, 0);
		setU8(R_PWR, 0);
	}
}

ampru_base::GetWheelEncoder::GetWheelEncoder()
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
}

ampru_base::GetMPUData::GetMPUData()
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
}

ampru_base::EchoMessage::EchoMessage(uint32_t value)
	: Message(MESSAGE_TYPE, PAYLOAD_LEN)
{
	setU16(VALUE, value);
}

ampru_base::EchoMessage::EchoMessage(const uint8_t* data, size_t length)
	: Message(data, length)
{
}

uint32_t ampru_base::EchoMessage::getValue() const
{
	return getU16(VALUE);
}

ampru_base::WheelEncoderData::WheelEncoderData(const uint8_t* data, size_t length)
	: Message(data, length)
{
}

int32_t ampru_base::WheelEncoderData::getLeftPulses() const
{
	return (int32_t)getI16(L_PULSES);
}

int32_t ampru_base::WheelEncoderData::getRightPulses() const
{
	return (int32_t)getI16(R_PULSES);
}

ampru_base::MPUData::MPUData(const uint8_t* data, size_t length)
	: Message(data, length)
{

}

int32_t ampru_base::MPUData::getAccelerationX()
{
	return (int32_t)getI16(ACC_X);
}

int32_t ampru_base::MPUData::getAccelerationY()
{
	return (int32_t)getI16(ACC_Y);
}

int32_t ampru_base::MPUData::getAccelerationZ()
{
	return (int32_t)getI16(ACC_Z);
}

int32_t ampru_base::MPUData::getGyroX()
{
	return (int32_t)getI16(GYR_X);
}

int32_t ampru_base::MPUData::getGyroY()
{
	return (int32_t)getI16(GYR_Y);
}

int32_t ampru_base::MPUData::getGyroZ()
{
	return (int32_t)getI16(GYR_Z);
}
