
#include "ampru_base/serial_port.h"
#include <ros/console.h>
#include <chrono>

#define AMPRU_HDLC_FRAME_BOUNDRY_FLAG	0x7E
#define AMPRU_HDLC_ESCAPE_FLAG		0x7D
#define AMPRU_HDLC_ESCAPE_XOR		0x20
#define AMPRU_HDLC_CRC_INIT_VALUE	0xFFFF

ampru_base::SerialPort::SerialPort(const std::string& port)
	: _serialPort(port)
	, _txBuffer(NULL)
	, _txBufferSize(0)
	, _txBufferLength(0)
	, _rxBuffer(NULL)
	, _rxBufferSize(0)
	, _rxBufferLength(0)
	, _rxEscapeByte(false)
	, _frameBuffer(NULL)
	, _frameBufferSize(0)
	, _frameBufferLength(0)
	, _frameChecksum(AMPRU_HDLC_CRC_INIT_VALUE)
{
	setTxBufferSize(AMPRU_SERIAL_DEFAULT_BUFFER_SIZE);
	setRxBufferSize(AMPRU_SERIAL_DEFAULT_BUFFER_SIZE);
	setMaxFrameLength(AMPRU_SERIAL_MAX_FRAME_LENGTH);
}

ampru_base::SerialPort::~SerialPort()
{
	closeConnection();
	setTxBufferSize(0);
	setRxBufferSize(0);
	setMaxFrameLength(0);
}

void ampru_base::SerialPort::setPort(const std::string& port)
{
	if (port != _serialPort)
	{
		closeConnection();
		_serialPort = port;
	}
}

void ampru_base::SerialPort::setTxBufferSize(size_t size)
{
	if (size != _txBufferSize)
	{
		if (_txBuffer != NULL)
		{
			free(_txBuffer);
			_txBuffer = NULL;
		}
		_txBufferSize = size;
		if (_txBufferSize > 0)
		{
			_txBuffer = (uint8_t*)malloc(_txBufferSize);
		}
	}
	_txBufferLength = 0;
}

void ampru_base::SerialPort::setRxBufferSize(size_t size)
{
	if (size != _rxBufferSize)
	{
		if (_rxBuffer != NULL)
		{
			free(_rxBuffer);
			_rxBuffer = NULL;
		}
		_rxBufferSize = size;
		if (_rxBufferSize > 0)
		{
			_rxBuffer = (uint8_t*)malloc(_rxBufferSize);
		}
	}
	_rxBufferLength = 0;
}

void ampru_base::SerialPort::setMaxFrameLength(size_t length)
{
	if (length != _frameBufferSize)
	{
		if (_frameBuffer != NULL)
		{
			free(_frameBuffer);
			_frameBuffer = NULL;
		}
		_frameBufferSize = length;
		if (_frameBufferSize > 0)
		{
			_frameBuffer = (uint8_t*)malloc(_frameBufferSize);
		}
	}
	_frameBufferLength = 0;
	_frameChecksum = AMPRU_HDLC_CRC_INIT_VALUE;
}

bool ampru_base::SerialPort::openConnection()
{
	if (_serial.isOpen())
	{
		return true;
	}

	auto timeout = serial::Timeout::simpleTimeout(1000);
	_serial.setPort(_serialPort);
	_serial.setBaudrate(9600);
	_serial.setTimeout(timeout);
	_serial.open();

	if (_serial.isOpen())
	{
		ROS_INFO_STREAM("Connected to serial port '" << _serial.getPort() << "'");
		return true;
	}
	else
	{
		ROS_ERROR_STREAM("Failed to connect to serial port '" << _serial.getPort() << "'");
		return false;
	}
}

void ampru_base::SerialPort::closeConnection()
{
	if (_serial.isOpen())
	{
		_serial.close();
	}
}

void ampru_base::SerialPort::sendMessage(Message* message)
{
	if (message == NULL)
	{
		return;
	}

	writeFrame(message->getData(), message->getSize());
}

ampru_base::Message* ampru_base::SerialPort::receiveMessage()
{
	read();

	if (_messageQueue.empty())
	{
		return NULL;
	}

	auto message = _messageQueue.front();
	_messageQueue.pop_front();
	return message;
}

ampru_base::Message* ampru_base::SerialPort::receiveMessage(uint16_t messageType)
{
	read();

	for (auto iter = _messageQueue.begin(); iter != _messageQueue.end(); ++iter)
	{
		if ((*iter)->getType() == messageType)
		{
			auto message = *iter;
			_messageQueue.erase(iter);
			return message;
		}
	}

	return NULL;
}

ampru_base::Message* ampru_base::SerialPort::waitMessage(double timeout)
{
	auto start_time = std::chrono::system_clock::now();
	while (true)
	{
		auto message = receiveMessage();
		if (message != NULL)
		{
			return message;
		}

		std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start_time;
		if (elapsed.count() >= timeout)
		{
			break;
		}
	}

	return NULL;
}

ampru_base::Message* ampru_base::SerialPort::waitMessage(uint16_t messageType, double timeout)
{
	auto start_time = std::chrono::system_clock::now();
	while (true)
	{
		auto message = receiveMessage(messageType);
		if (message != NULL)
		{
			return message;
		}

		std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start_time;
		if (elapsed.count() >= timeout)
		{
			break;
		}
	}

	return NULL;
}

void ampru_base::SerialPort::read()
{
	try
	{
		if (openConnection())
		{
			size_t available = _serial.available();
			if (available > 0)
			{
				if (available > _rxBufferSize)
				{
					available = _rxBufferSize;
				}
				_rxBufferLength = _serial.read(_rxBuffer, available);
				for (size_t i = 0; i < _rxBufferLength; i++)
				{
					decodeByte(_rxBuffer[i]);
				}
			}
		}
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Failed to receive data from serial port '" << _serial.getPort() << "'");
		closeConnection();
	}
}

void ampru_base::SerialPort::writeFrame(const uint8_t* data, size_t length)
{
	uint16_t checksum = AMPRU_HDLC_CRC_INIT_VALUE;

	_txBufferLength = 0;
	encodeByte(AMPRU_HDLC_FRAME_BOUNDRY_FLAG, false);
	for (size_t i = 0; i < length; i++)
	{
		checksum = crc_ccitt_update(checksum, data[i]);
		encodeByte(data[i], true);
	}
	encodeByte(checksum & 0xFF, true);
	encodeByte((checksum >> 8) & 0xFF, true);
	encodeByte(AMPRU_HDLC_FRAME_BOUNDRY_FLAG, false);

	try
	{
		if (openConnection())
		{
			_serial.write(_txBuffer, _txBufferLength);
		}
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Failed to send data to serial port '" << _serial.getPort() << "'");
		closeConnection();
	}
}

void ampru_base::SerialPort::encodeByte(uint8_t data, bool escape)
{
	if (escape && ((data == AMPRU_HDLC_ESCAPE_FLAG) || (data == AMPRU_HDLC_FRAME_BOUNDRY_FLAG)))
	{
		if (_txBufferLength < (_txBufferSize - 1))
		{
			_txBuffer[_txBufferLength] = AMPRU_HDLC_ESCAPE_FLAG;
			_txBufferLength++;
			data ^= AMPRU_HDLC_ESCAPE_XOR;
		}
	}

	if (_txBufferLength < (_txBufferSize - 1))
	{
		_txBuffer[_txBufferLength] = data;
		_txBufferLength++;
	}
}

void ampru_base::SerialPort::decodeByte(uint8_t data)
{
	if (data == AMPRU_HDLC_FRAME_BOUNDRY_FLAG)
	{
		if (_rxEscapeByte)
		{
			_rxEscapeByte = false;
		}
		else if (_frameBufferLength >= 2 && _frameChecksum == ((_frameBuffer[_frameBufferLength - 1] << 8) | _frameBuffer[_frameBufferLength - 2]))
		{
			auto message = Message::parse(_frameBuffer, _frameBufferLength - 2);
			if (message != NULL)
			{
				_messageQueue.push_back(message);
			}
		}
		_frameBufferLength = 0;
		_frameChecksum = AMPRU_HDLC_CRC_INIT_VALUE;
		return;
	}

	if (_rxEscapeByte)
	{
		_rxEscapeByte = false;
		data ^= AMPRU_HDLC_ESCAPE_XOR;
	}
	else if (data == AMPRU_HDLC_ESCAPE_FLAG)
	{
		_rxEscapeByte = true;
		return;
	}

	if (_frameBufferLength < (_frameBufferSize - 1))
	{
		_frameBuffer[_frameBufferLength] = data;
		if (_frameBufferLength >= 2)
		{
			_frameChecksum = crc_ccitt_update(_frameChecksum, _frameBuffer[_frameBufferLength - 2]);
		}

		_frameBufferLength++;
		if (_frameBufferLength == _frameBufferSize)
		{
			_frameBufferLength = 0;
			_frameChecksum = AMPRU_HDLC_CRC_INIT_VALUE;
		}
	}
}

uint16_t ampru_base::SerialPort::crc_ccitt_update(uint16_t crc, uint8_t data)
{
	data ^= (uint8_t)(crc & 0xFF);
	data ^= (data << 4);
	return ((((uint16_t)data << 8) | ((uint8_t)(crc >> 8) & 0xFF)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

