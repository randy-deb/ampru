
#ifndef ___AMPRU_BASE_MESSAGE_H___
#define ___AMPRU_BASE_MESSAGE_H___

#include <stddef.h>
#include <stdint.h>

namespace ampru_base
{
	class Message
	{
	public:
		static Message* parse(const uint8_t* data, size_t length);

	protected:
		Message(const uint8_t* data, size_t length);
		Message(uint16_t messageType, size_t payloadLength);
		virtual ~Message();

	public:
		uint16_t getType() const;
		uint8_t* getData() const;
		size_t getSize() const;

	public:
		int8_t getI8(size_t offset) const;
		int16_t getI16(size_t offset) const;
		int32_t getI32(size_t offset) const;
		uint8_t getU8(size_t offset) const;
		uint16_t getU16(size_t offset) const;
		uint32_t getU32(size_t offset) const;
		void setI8(size_t offset, int8_t value);
		void setI16(size_t offset, int16_t value);
		void setI32(size_t offset, int32_t value);
		void setU8(size_t offset, uint8_t value);
		void setU16(size_t offset, uint16_t value);
		void setU32(size_t offset, uint32_t value);

	private:
		uint8_t* _data;
		size_t _size;

	};

	class SetMotorSpeed 
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x0001 };
		enum
		{
			L_DIR = 0,
			L_PWR = 1,
			R_DIR = 2,
			R_PWR = 3,
			PAYLOAD_LEN = 4,
		};

	public:
		SetMotorSpeed(double leftWheel, double rightWheel);

	};

	class GetWheelEncoder
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x0002 };
		enum
		{
			PAYLOAD_LEN = 0,
		};

	public:
		GetWheelEncoder();

	};

	class EchoMessage
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x00AA };
		enum
		{
			VALUE = 0,
			PAYLOAD_LEN = 2,
		};

	public:
		EchoMessage(uint32_t value);
		EchoMessage(const uint8_t* data, size_t length);
		uint32_t getValue() const;

	};

	class WheelEncoderData
		: public Message
	{
	public:
		enum { MESSAGE_TYPE = 0x7002 };
		enum
		{
			L_PULSES = 0,
			R_PULSES = 2,
			PAYLOAD_LEN = 4,
		};

	public:
		WheelEncoderData(const uint8_t* data, size_t length);

	public:
		size_t getLeftPulses() const;
		size_t getRightPulses() const;

	};
}

#endif // ___AMPRU_BASE_MESSAGE_H___

