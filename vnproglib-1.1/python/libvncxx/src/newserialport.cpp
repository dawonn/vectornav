
#if _NEW_SERIAL_PORT
#include "vn/newserialport.h"

#include "vn/exceptions.h"
#include "vn/packet.h"
#include "vn/criticalsection.h"

#include "vn/vntime.h"

#include <algorithm>
#include <iostream>

namespace vn {
namespace xplat {
	const std::map < NewSerialPort::BinaryMessageGroup, std::vector<uint8_t> > NewSerialPort::BinaryFieldLengthsMap =
	{
		{ NewSerialPort::NSPBMG_GROUP1, {  8,  8,  8, 12, 16, 12, 24, 12, 12, 24, 20, 28,  2,  4,  8,  0 }},		// Group 1
		{ NewSerialPort::NSPBMG_GROUP2, {  8,  8,  8,  2,  8,  8,  8,  4,  0,  0,  0,  0,  0,  0,  0,  0 }},		// Group 2
		{ NewSerialPort::NSPBMG_GROUP3, {  2, 12, 12, 12,  4,  4, 16, 12, 12, 12, 12,  2, 40,  0,  0,  0 }},		// Group 3
		{ NewSerialPort::NSPBMG_GROUP4, {  8,  8,  2,  1,  1, 24, 24, 12, 12, 12,  4,  4, 32,  0,  0,  0 }},		// Group 4
		{ NewSerialPort::NSPBMG_GROUP5, {  2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24,  0,  0,  0,  0 }},		// Group 5
		{ NewSerialPort::NSPBMG_GROUP6, {  2, 24, 24, 12, 12, 12, 12, 12, 12,  4,  4, 68, 64,  0,  0,  0 }},		// Group 6
		{ NewSerialPort::NSPBMG_GROUP7, {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }},		// Invalid group
		{ NewSerialPort::NSPBMG_GROUP8, {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }}		// Invalid group
	};

	const size_t NewSerialPort::MaxBinaryPacketSize = 256;
	const size_t NewSerialPort::MaxAsciiPacketSize = 256;
	const size_t NewSerialPort::MaxBinaryHeaderSize = 18;
	const size_t NewSerialPort::NumBytesToPurge = 100;
	const size_t NewSerialPort::ReceiveBufferSize = 512;

	const std::vector<char> NewSerialPort::ASCIIStartSentinal = { '$' };
	const std::vector<char> NewSerialPort::ASCIIEndSentinal = { '\r', '\n' };
	const std::vector<char> NewSerialPort::BinaryStartSentinal = { static_cast<char>(0XFA) };

    const uint8_t NewSerialPort::ReadWaitTime = 100;

	NewSerialPort::NewSerialPort() :
#if _WIN32
		m_serialPortHandle(INVALID_HANDLE_VALUE),
#endif
		m_isOpen(false),
		m_packetQueueCriticalSection(NULL),
		m_readBufferCriticalSection(NULL),
		m_portName(""),
		m_stopBits(NSPSB_ONE),
		m_baudRate(0),
		m_packetCounter(0)
    {
        // Intentionally left blank; should not be used.
    }

    NewSerialPort::NewSerialPort(const std::string& aPortName,
                                 uint32_t aBaudRate) :
#if _WIN32
    m_serialPortHandle(INVALID_HANDLE_VALUE),
#endif
    m_isOpen(false),
    m_portName(aPortName),
    m_stopBits(NSPSB_ONE),
    m_baudRate(aBaudRate),
	m_packetCounter(0)
    {
        m_packetQueueCriticalSection = new CriticalSection();
        m_readBufferCriticalSection = new CriticalSection();
	}

    NewSerialPort::~NewSerialPort()
    {
		stop();

		while (!isFinished())
		{
			sleepMs(1);
		}

		close();

        if (m_readBufferCriticalSection != NULL)
        {
            delete m_readBufferCriticalSection;
            m_readBufferCriticalSection = NULL;
        }

        if ((NULL != m_packetQueueCriticalSection) &&
            (!m_packetQueue.empty()))
        {
            // If the critical section doesn't exist don't delete the vector
            // Something may have gone wrong and a slight memory leak is
            // better than a full on crash.
            m_packetQueueCriticalSection->enter();

			while (!m_packetQueue.empty())
            {
				delete m_packetQueue.front();
				m_packetQueue.pop();
            }

            m_packetQueueCriticalSection->leave();

            delete m_packetQueueCriticalSection;
            m_packetQueueCriticalSection = NULL;
        }

    }

	uint32_t NewSerialPort::baudRate()
	{
		return m_baudRate;
	}

	void NewSerialPort::baudRate(uint32_t aBaudRate)
	{
		m_baudRate = aBaudRate;
	}

    void NewSerialPort::close()
    {
		if (m_isOpen)
		{
			if (m_isRunning)
	        {
				stop();

				while (!m_isFinished)
				{
					NewThread::sleepMs(1);
				}
			}
#if _WIN32
            if (!CloseHandle(m_serialPortHandle))
            {
                // Closing failed
                throw unknown_error();
            }

			m_isOpen = false;
			m_serialPortHandle = INVALID_HANDLE_VALUE;
#endif
        }
    }

    void NewSerialPort::getAvailablePortNames(std::vector<std::string>& aPortNames)
{
    aPortNames.clear();

	#if _WIN32
	DWORD numOfSubkeys = 0;
	DWORD numOfValues = 0;

	HKEY serialCommKey;
	LONG error;

	error = RegOpenKeyEx(
		HKEY_LOCAL_MACHINE,
		TEXT("HARDWARE\\DEVICEMAP\\SERIALCOMM"),
		0,
		KEY_READ,
		&serialCommKey);

    if (ERROR_SUCCESS == error)
    {
        error = RegQueryInfoKey(
            serialCommKey,
            NULL,
            NULL,
            NULL,
            &numOfSubkeys,
            NULL,
            NULL,
            &numOfValues,
            NULL,
            NULL,
            NULL,
            NULL);
    }

    if (ERROR_SUCCESS == error)
    {
        for (size_t i = 0; i < numOfValues; i++)
        {
	        TCHAR data[0x100];
	        TCHAR value[0x100];
	        DWORD capacity = 0x100;
	        DWORD dataSize = sizeof(data);

	        error = RegEnumValue(
		        serialCommKey,
		        i,
		        value,
		        &capacity,
		        NULL,
		        NULL,
		        (LPBYTE) data,
		        &dataSize);

		    if (error != ERROR_SUCCESS)
			    throw unknown_error();

		    #ifdef UNICODE

		    char converted[0x100];
		    int convertResult = WideCharToMultiByte(
			    CP_ACP,
			    0,
			    data,
			    dataSize,
			    converted,
			    sizeof(converted),
			    NULL,
			    NULL);

		    if (convertResult == 0)
			    throw unknown_error();

		    aPortNames.push_back(std::string(converted));

		    #else
		    aPortNames.push_back(std::string(data));
		    #endif
	    }
    }

    // A file not found error is acceptable here and should not throw an error.
    // This simply indicates that there are no available serial ports.
    if ((ERROR_SUCCESS != error) &&
        (ERROR_FILE_NOT_FOUND != error))
    {
        throw unknown_error();
    }

	#elif __linux__ || __CYGWIN__ || __QNXNTO__

	// comPorts;

	int portFd = -1;
	const size_t MAX_PORTS = 255;
	std::string portName;
	std::stringstream stream;

	for(size_t index = 0; index < MAX_PORTS; ++index)
	{
		stream << "/dev/ttyUSB" << index;
		portName = stream.str();
		portFd = ::open(portName.c_str(),
						#if __linux__ || __CYGWIN__ || __QNXNTO__
						O_RDWR | O_NOCTTY );
						#else
						#error "Unknown System"
						#endif

		if(portFd != -1)
		{
			aPortNames.push_back(portName);
			::close(portFd);
		}

		portName.clear();
		stream.str(std::string());
	}

	#elif __APPLE__

	DIR *dp = NULL;
	struct dirent *dirp;

	if ((dp = opendir("/dev")) == NULL)
		throw unknown_error();

	while ((dirp = readdir(dp)) != NULL)
	{
		if (strstr(dirp->d_name, "tty.usbserial") != NULL)
			aPortNames.push_back(string(dirp->d_name));
	}

	closedir(dp);

	#else
	#error "Unknown System"
	#endif
}

	protocol::uart::Packet* NewSerialPort::getNextPacket()
	{
		protocol::uart::Packet* packet = NULL;

		if (!m_packetQueue.empty())
		{
			this->m_packetQueueCriticalSection->enter();
			packet = m_packetQueue.front();
			m_packetQueue.pop();
			this->m_packetQueueCriticalSection->leave();
		}

		return packet;
	}

	bool NewSerialPort::hasPacket()
	{
		return (!m_packetQueue.empty());
	}

    bool NewSerialPort::isOpen()
    {
        return m_isOpen;
    }

    void NewSerialPort::open()
    {
#ifdef _WIN32
        // If the port is already open then do nothing
        if (!m_isOpen)
        {
			DCB config;
            COMMTIMEOUTS comTimeOut;

            // Create a windows based port name
            std::string fullPortName = "\\\\.\\" + m_portName;

            // Call winows to create a handle to the port
            m_serialPortHandle = CreateFile(fullPortName.c_str(),
                                            GENERIC_READ | GENERIC_WRITE,
                                            0,
                                            NULL,
                                            OPEN_EXISTING,
                                            0,
                                            NULL);

			// Check for success
            if (INVALID_HANDLE_VALUE == m_serialPortHandle)
            {
                DWORD error = GetLastError();

                // The error was caused because the port is most likely already open
                if (ERROR_ACCESS_DENIED == error)
                {
                    throw permission_denied("Port '" + m_portName + "' already open.");
                }

                // The error was because the port does not exist
                if (ERROR_FILE_NOT_FOUND == error)
                {
                    throw not_found(m_portName);
                }

                // This is another case where the port is most likely already open
                if (ERROR_SHARING_VIOLATION == error)
                {
                    throw permission_denied("Port '" + m_portName + "' already open.");
                }

                // Something happend but we don't know what.
                throw unknown_error();
            }

            // Get the port's current configuration state
            if (!GetCommState(m_serialPortHandle, &config))
            {
                // That didn't work
                // Close the port
                CloseHandle(m_serialPortHandle);
                m_serialPortHandle = INVALID_HANDLE_VALUE;
                // Throw an error to indicate the port opening failed
                throw unknown_error();
            }

            // Set the stop bits
            switch (m_stopBits)
            {
            case NSPSB_ONE:
                config.StopBits = ONESTOPBIT;
            case NSPSB_TWO:
                config.StopBits = TWOSTOPBITS;
            default:
                // Do nothing
                break;
            }

            config.BaudRate = m_baudRate;
            config.Parity = NOPARITY;
            config.ByteSize = 8;
            config.fAbortOnError = 0;

            if (!SetCommState(m_serialPortHandle, &config))
            {
                // That didn't work
                // Close the port
                CloseHandle(m_serialPortHandle);
                m_serialPortHandle = INVALID_HANDLE_VALUE;

                // Throw an exception based on the error
                DWORD error = GetLastError();
                if (ERROR_INVALID_PARAMETER == error)
                {
                    throw std::invalid_argument("Unsupported baudrate.");
                }
                else
                {
                    throw unknown_error();
                }
            }

            comTimeOut.ReadIntervalTimeout = 1;
            comTimeOut.ReadTotalTimeoutMultiplier = 1;
            comTimeOut.ReadTotalTimeoutConstant = 1;
            comTimeOut.WriteTotalTimeoutMultiplier = 1;
            comTimeOut.WriteTotalTimeoutConstant = 1;

            if (!SetCommTimeouts(m_serialPortHandle, &comTimeOut))
            {
                // That didn't work
                // Close the port
                CloseHandle(m_serialPortHandle);
                // Throw an error
                throw unknown_error();
            }

            if (!SetCommMask(m_serialPortHandle, EV_RXCHAR))
            {
                throw unknown_error();
            }

            m_isOpen = true;

            // purge data
            //std::vector<char> tmpBuffer;
            //size_t tmpData;
            //read(tmpBuffer, NumBytesToPurge, tmpData);
        }
#endif
    }

    std::string NewSerialPort::port()
    {
        return m_portName;
    }

	vn::protocol::uart::Packet* NewSerialPort::processASCIIData(std::vector<char>::iterator aAsciiStartItr,
																std::vector<char>::iterator aAsciiEndItr)
	{
		vn::protocol::uart::Packet* packet = NULL;

		if ((m_readBuffer.end() != aAsciiStartItr) &&
			(m_readBuffer.end() != aAsciiEndItr))
		{
			char* start = aAsciiStartItr._Ptr;
			char* end = aAsciiEndItr._Ptr;

			// Add 2 to include the terminating characters in the length
			// This is due to the fact that the asciiEndItr is pointing to the characters
			// instead of after them
			size_t length = end - start + 2;

			packet = new protocol::uart::Packet(start, length);

			// It doesn't matter if the packet is valid; good data is used and bad
			// data needs to be removed.
			m_readBufferCriticalSection->enter();
			std::vector<char>(aAsciiEndItr + 2, m_readBuffer.end()).swap(m_readBuffer);
			m_readBufferCriticalSection->leave();
		}

		return packet;
	}

	vn::protocol::uart::Packet* NewSerialPort::processBinaryData(std::vector<char>::iterator aBinaryItr)
	{
		vn::protocol::uart::Packet* packet = NULL;

		BinaryMessageGroup group = NSPBMG_INVALID;

		// Increment the pointer to get away from the sentinal value
		char* binaryData = aBinaryItr._Ptr + 1;

		uint8_t groupsByte = static_cast<uint8_t>(*binaryData++);
		uint8_t groupsFieldByte = 1;

		uint16_t fieldWord = 0;
		uint16_t fieldPayloadWord = 0;
		uint16_t numFieldPayloadWordBits = sizeof(fieldPayloadWord) * 8;
		uint16_t packetSize = 4;  // this covers the sentinal byte, group byte, and the CRC word

		for (size_t groupIndex = 0; groupIndex < sizeof(groupsByte); ++groupIndex)
		{
			if(groupsByte & groupsFieldByte)
			{
				// Add two to packet size as each group field is a word
				packetSize += 2;

				// Get the group this belongs to
				group = static_cast<BinaryMessageGroup>(groupsFieldByte);
				// Get the actual word of the group
				fieldWord = static_cast<uint16_t>(*binaryData);
				// Increment the data pointer
				binaryData += 2;
				// Reset the field indicator
				fieldPayloadWord = 1;
				// Loop through each field bit in the group byte
				for (size_t fieldIndex = 0; fieldIndex < numFieldPayloadWordBits; ++fieldIndex)
				{
					//  Check if the bit is set
					if (fieldWord & fieldPayloadWord)
					{
						// The bit is set; add the length of the field to the packet length
						packetSize += BinaryFieldLengthsMap.find(group)->second[fieldIndex];
					}

					// Bit shift the field indicator
					fieldPayloadWord <<= 1;
				}
			}

			// Bit shift the group indicator
			groupsFieldByte >>= 1;
		}

		 packet = new protocol::uart::Packet(aBinaryItr._Ptr, packetSize);

		// It doesn't matter if the packet is valid; good data is used and bad
		// data needs to be removed.
		m_readBufferCriticalSection->enter();
		std::vector<char>(aBinaryItr + packetSize, m_readBuffer.end()).swap(m_readBuffer);
		m_readBufferCriticalSection->leave();

		return packet;
	}

    void NewSerialPort::processData()
    {
		bool outOfData = false;

		std::vector<char>::iterator binaryItr = m_readBuffer.begin();

		std::vector<char>::iterator asciiStartItr = m_readBuffer.begin();
		std::vector<char>::iterator asciiEndItr = m_readBuffer.end();

		vn::protocol::uart::Packet* packet = NULL;

		while (!outOfData)
		{
			packet = NULL;
			// find first ascii packet start sentinal after the last
			// ascii search position
			asciiStartItr = std::find(asciiStartItr,
				m_readBuffer.end(),
				ASCIIStartSentinal[0]);
			// find first ascii packet end sentinal after start
			asciiEndItr = std::search(asciiStartItr,
				m_readBuffer.end(),
				ASCIIEndSentinal.begin(),
				ASCIIEndSentinal.end());
			// find first binary packet start sentinal after the last
			// binary search position
			binaryItr = std::find(binaryItr,
				m_readBuffer.end(),
				BinaryStartSentinal[0]);

			// Check if this could be an ascii packet
			if ((asciiStartItr != m_readBuffer.end()) && 
				(asciiStartItr._Ptr < binaryItr._Ptr) &&
				(asciiEndItr != m_readBuffer.end()))
			{
				packet = processASCIIData(asciiStartItr, asciiEndItr);
			}

			// Check if this could be an binary packet
			else if (binaryItr != m_readBuffer.end())
			{
				packet = processBinaryData(binaryItr);
			}
			else
			{
				outOfData = true;
			}

			if (NULL != packet)
			{
				// Do this in case there is more data to process
				// The data buffer has changed start back from the new beginning
				asciiStartItr = m_readBuffer.begin();
				binaryItr = m_readBuffer.begin();

				// Store down the packet if it is valid
				if (packet->isValid())
				{
					m_packetQueueCriticalSection->enter();
					m_packetQueue.push(packet);
					m_packetCounter++;
					m_packetQueueCriticalSection->leave();
				}
				// Delete the packet because it is invalid
				else
				{
					delete packet;
					packet = NULL;
				}
			}
		}
    }

	void NewSerialPort::read(char aDataBuffer[], size_t aNumOfBytesToRead, size_t& aNumOfBytesActuallyRead)
	{
		std::vector<char> dataBuffer;
		read(dataBuffer, aNumOfBytesToRead, aNumOfBytesActuallyRead);

		// sanity check
		if (sizeof(aDataBuffer) >= dataBuffer.size())
		{
			memcpy(aDataBuffer, dataBuffer.data(), dataBuffer.size());
		}
	}

    void NewSerialPort::read(std::vector<char>& aDataBuffer,
                             size_t aNumOfBytesToRead,
                             size_t &aNumOfBytesActuallyRead)
    {
        read(aDataBuffer, aNumOfBytesToRead);

        aNumOfBytesActuallyRead = aDataBuffer.size();
    }

    void NewSerialPort::read(std::vector<char>& aDataBuffer,
                             size_t aNumOfBytesToRead)
    {
#if _WIN32
        size_t numBytesRead = 0;

        aDataBuffer.clear();
        aDataBuffer.assign(aNumOfBytesToRead, '\0');

		BOOL result = ReadFile(
            m_serialPortHandle,
            aDataBuffer.data(),
            aNumOfBytesToRead,
            reinterpret_cast<LPDWORD>(&numBytesRead),
            NULL);

		DWORD error = GetLastError();

        if (!result && GetLastError() != ERROR_IO_PENDING)
        {
            throw unknown_error();
        }
        else if (numBytesRead > 0)
        {
            // For an accurate reading of how many bytes were actually read
            aDataBuffer.resize(numBytesRead);
            storeReadData(aDataBuffer);
			processData();
        }

#endif
    }

    void NewSerialPort::run()
    {
        DWORD dwEventMask = 0;
        std::vector<char> tmpBuffer;
        size_t toRead = 256;
        size_t bytesRead = 256;

        while (m_isRunning)
        {
            dwEventMask = 0;

            // Wait for comm event
            //if (WaitCommEvent(m_serialPortHandle, &dwEventMask, NULL))
            {
                read(tmpBuffer, toRead, bytesRead);
            }
        }

        m_isFinished = true;;
    }

    void NewSerialPort::storeReadData(const std::vector<char>& aDataBuffer)
    {
        if (aDataBuffer.size() > 0)
        {
            m_readBufferCriticalSection->enter();

            m_readBuffer.insert(m_readBuffer.end(), aDataBuffer.begin(), aDataBuffer.end());

            m_readBufferCriticalSection->leave();
        }
    }

	void NewSerialPort::write(const char aData[], size_t aLength)
	{
		std::vector<char> dataBuffer(aData, aData + aLength);
		write(dataBuffer);
	}

    void NewSerialPort::write(const std::vector<char>& aDataBuffer)
    {
#if _WIN32

        DWORD numOfBytesWritten;
		DWORD numBytesToWrite = aDataBuffer.size();
		const char* data = aDataBuffer.data();

        BOOL result = WriteFile(m_serialPortHandle,
                                data,
                                numBytesToWrite,
                                &numOfBytesWritten,
                                NULL);

        if (!result && GetLastError() != ERROR_IO_PENDING)
            throw unknown_error();

        result = FlushFileBuffers(m_serialPortHandle);

        if (!result)
        {
            throw unknown_error();
        }
#endif
    }

    // BEGIN SEARCHER

    const std::vector<uint32_t> NewSearcher::TestBaudrates = { 115200, 128000, 230400, 460800, 921600, 57600, 38400, 19200, 9600 };

    NewSearcher::NewSearcher() :
        m_isMaster(true),
        m_port(NULL)
    {}

    NewSearcher::NewSearcher(const std::string& aPortName) :
        m_isMaster(false),
        m_port(NULL)
    {
        m_portList.push_back(aPortName);
    }

    NewSearcher::~NewSearcher()
    {
        if(NULL != m_port)
        {
            m_port->stop();
            m_port->close();
            delete m_port;
            m_port = NULL;
        }
    }

    bool NewSearcher::finished()
    {
        return m_isFinished;
    }

	size_t NewSearcher::getNumSensors()
    {
		return m_isFinished ? m_validPorts.size() : 0;
    }

	std::pair<std::string, std::uint32_t> NewSearcher::getSensor(size_t aIndex)
    {
		return (m_isFinished && (aIndex < m_validPorts.size()) ? m_validPorts[aIndex] : std::pair<std::string, std::uint32_t>("", 0));
    }

    void NewSearcher::reset()
    {
        m_isFinished = false;
        m_portList.clear();
        m_validPorts.clear();
    }

    void NewSearcher::run()
    {
        if (m_isMaster)
        {
            searchPorts();
        }
        else
        {
            searchBauds();
        }

        m_isRunning = false;
        m_isFinished = true;
    }

    void NewSearcher::search()
    {
        std::vector<std::string> portNames;
        NewSerialPort::getAvailablePortNames(portNames);
        search(portNames);
    }

    void NewSearcher::search(const std::string& aPortName)
    {
        m_portList.push_back(aPortName);
        start();
    }

    void NewSearcher::search(const std::vector<std::string>& aPortsToCheck)
    {
        m_portList = aPortsToCheck;
        start();
    }

    void NewSearcher::searchBauds()
    {
        bool found = false;
        uint32_t baud = 0;

        for(size_t index = 0; index < TestBaudrates.size(); ++index)
        {
            // Create the serial port
            m_port = new NewSerialPort(m_portList[0], TestBaudrates[index]);

            try
            {
                // Attempt to open the port
                m_port->open();
                m_port->start();
            }
            catch (std::exception e)
            {
                // We don't care what happened.  Simply continue.
                m_port->stop();
                m_port->close();
                delete m_port;
                m_port = NULL;
                continue;
            }

            // create message buffer
            char dataBuffer[] = "$VNRRG,01*XX\r\n";
    		std::vector<char> dataVector(dataBuffer, dataBuffer + sizeof(dataBuffer));

            // search for response for half a second
            Stopwatch stopwatch;

            while((stopwatch.elapsedMs() < 100.0) && !found)
            {
                m_port->write(dataVector);

                // Look for a response packet
                while(m_port->hasPacket() && !found && (stopwatch.elapsedMs() < 100))
                {
                    // Process the packet
                    protocol::uart::Packet* packet = m_port->getNextPacket();
                    if(packet->isResponse())
                    {
                        if(packet->datastr().compare(0, 9, dataBuffer, 9) == 0)
                        {
                            // this is our response
                            found = true;
                            baud = TestBaudrates[index];
                        }
                    }
					// The packet has been spent; clean it up
					delete packet;
                }
            }

            // Clean up
            m_port->stop();
            m_port->close();
            delete m_port;
            m_port = NULL;

            // if we have a resonpse then break else continue
            if ( found )
            {
                // We have a sensor; record the port and baud
                m_validPorts.push_back(std::pair<std::string, uint32_t>(m_portList[0], TestBaudrates[index]));

                break;
            }
        }
    }

    void NewSearcher::searchPorts()
    {
        if ( !m_isFinished )
        {
            std::vector<NewSearcher*> searchers;

            m_validPorts.clear();

            for ( size_t index = 0; index < m_portList.size(); ++index )
            {
                NewSearcher* searcher = new NewSearcher(m_portList[index]);
                searcher->start();
                searchers.push_back(searcher);
            }


            for ( size_t index = 0; index < searchers.size(); ++index )
            {
                while ( !searchers[index]->isFinished() )
                {
                    sleepMs(1);
                }

                if ( searchers[index]->m_validPorts.size() > 0 )
                {
                    m_validPorts.insert(m_validPorts.end(), searchers[index]->m_validPorts.begin(), searchers[index]->m_validPorts.end());
                }

                delete searchers[index];
            }
        }
    }

    bool NewSearcher::searching()
    {
        return m_isRunning;
    }
}} // End vn/xplat namespaces

#endif
