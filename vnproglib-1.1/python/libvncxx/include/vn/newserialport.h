#ifndef NEWSERIALPORT
#define NEWSERIALPORT

#if _NEW_SERIAL_PORT

#include "int.h"
#include "export.h"
#include "port.h"
#include "thread.h"

#include <cstdint>
#include <map>
#include <queue>
#include <string>
#include <vector>

#ifdef _WIN32
    // Windows exclusive includes
    #include <Windows.h>

#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
#endif

// Forward Declarations
namespace vn {
	namespace protocol {
		namespace uart {
			struct Packet;
		}
	}

	namespace xplat {
		class CriticalSection;
	}
}



namespace vn {
namespace xplat {

class vn_proglib_DLLEXPORT NewSerialPort : public IPort, public NewThread
{
public:
	// Enumerations

    /// \brief Enumeration to identify which message group an incoming packet belongs to
	enum BinaryMessageGroup
	{
		NSPBMG_INVALID = 0,
		NSPBMG_GROUP1 = 0x01,
		NSPBMG_GROUP2 = 0x02,
		NSPBMG_GROUP3 = 0x04,
		NSPBMG_GROUP4 = 0x08,
		NSPBMG_GROUP5 = 0x10,
		NSPBMG_GROUP6 = 0x20,
		NSPBMG_GROUP7 = 0x40,
		NSPBMG_GROUP8 = 0x80,
		NSPBMG_NUM_GROUPS
	};
    /// \brief Enumeration representing the stop bits used in serial communications
    enum StopBits
    {
        NSPSB_INVALID = -1,
        NSPSB_ONE,
        NSPSB_TWO
    };

    /// \brief Maximum size of a binary packet
	static const size_t MaxBinaryPacketSize;
    /// \brief Maximum size of an ascii packet
	static const size_t MaxAsciiPacketSize;
    /// \brief Maximum size of a binary header
	static const size_t MaxBinaryHeaderSize;
	/// \brief Constant to hold the number of bytes to purge when a port is first opened
	static const size_t NumBytesToPurge;
    /// \brief Number of bytes to receive on each read request
	static const size_t ReceiveBufferSize;
    /// \brief The number of bytes each field in a binary message is.  Used to calculate how large an
    /// incoming binary message is.
	static const std::map < NewSerialPort::BinaryMessageGroup, std::vector<uint8_t> > BinaryFieldLengthsMap;
    /// \brief Set of characters that indicate the start of an ascii packet
	static const std::vector<char> ASCIIStartSentinal;
    /// \brief Set of characters that indicate the end of an incoming ascii packet
	static const std::vector<char> ASCIIEndSentinal;
    /// \brief Set of characters that indicate the start of a binary packet
	static const std::vector<char> BinaryStartSentinal;
	/// \brief Constant to hold the number of miliseconds to wait with each read
	static const uint8_t ReadWaitTime;

	// General Functions

    /// \brief CTOR
    /// Constructs a SerialPort object with the input port name and baud rate
    /// \param aPortName Name of the port to connect to
    /// \param aBaudRate The baud rate in bits per second to connect at
    NewSerialPort(const std::string& aPortName, uint32_t aBaudRate);
    /// \DTOR
    ~NewSerialPort();

    /// \brief Returns the current baud rate the port is connected at
    /// \return The current baud rate
	uint32_t baudRate();
    /// \brief Sets the current baud rate to the input value
    /// \param aBaudRate The baud rate to set the port connection to
	void baudRate(uint32_t aBaudRate);
    /// \brief Signals to the serial port that it should close the connection down.
	void close();
    /// \brief Searches the system for available serial ports and returns their names.
    /// This list is not guaranteed to be inclusive of all port names as some may be
    /// in use and will not be included in the list.
    /// \param aPortNames A vector of available port names
    static void getAvailablePortNames(std::vector<std::string>& aPortNames);
    /// \brief getNextPacket Packets are stored in queue form.  This will pop the next
    /// packet off of the queue and return it to the caller.  The caller then becomes
    /// the owner of the packet and is responsible for clean up.  If the queue is empty
    /// this function returns a null pointer.
    /// \return A pointer to the next packet in the queue.
	protocol::uart::Packet* getNextPacket();
    /// \brief Indicates if there is a packet on the queue without popping the packet.
    /// \return Flag indicating if a packet is on the queue.
	bool hasPacket();
    /// \brief Indicates if the port is currently open.
    /// \return Flag to indicate if the port is open.
    bool isOpen();
    /// \brief Tells the serial port to create a connection to the current port and
    /// at the current baud rate.
    void open();
    /// \brief Returs the name of the port the object is connected to
    /// \return Name of the port.
    std::string port();
    /// \brief Reads up to aNumOfBytesToRead and stores them in the input vector.  The input
    /// buffer is cleared and a failure to read means the buffer will return empty.
    /// \param aDataBuffer vector to hold the data read from the serial port
    /// \param aNumOfBytesToRead The maximum number of bytes to read on this call
    void read(std::vector<char>& aDataBuffer,
              size_t aNumOfBytesToRead);
    /// \brief Writes the input data to the serial port and sends it.
    /// \param aDataBuffer A vector containing the data to send over the port.
    void write(const std::vector<char>& aDataBuffer);

    // DEPRICATED PUBLIC FUNCTIONS

    /// \deprecated Takes in the maximum number of bytes to read and returns the actual
    /// bytes read as well as the number read.  This is to provide some backwards
    /// compatibility with software written against the older serial port and will
    /// eventually be removed.
    /// \param aDataBuffer vector to hold the data read from the serial port
    /// \param aNumOfBytesToRead The maximum number of bytes to read on this call
    /// \param aNumOfBytesActuallyRead The number of bytes actually read off of the
    /// serial port.  This number can be less than aNumOfBytesToRead but never larger.
    void read(std::vector<char>& aDataBuffer,
              size_t aNumOfBytesToRead,
              size_t &aNumOfBytesActuallyRead);
    /// \deprecated This function takes in C-style inputs and will be removed in future
    /// releases
    /// Reads upto aNumOfBytesToRead bytes into the input aDataBuffer array.  Sets
    /// aNumOfBytesActuallyRead to the number of bytes the port was able to read.  This
    /// number can be less than aNumOfBytesToRead but never more.
    /// \param aDataBuffer Buffer to hold the data read from the serial port.  It should be
    /// at least aNumOfBytesToRead in size.
    /// \param aNumOfBytesToRead The maximum number of bytes to read during this call
    /// \param aNumOfBytesActuallyRead The number of bytes read from the serial port.  This
    /// number can be less than aNumOfBytesToRead but never more.
    void read(char aDataBuffer[], size_t aNumOfBytesToRead, size_t &aNumOfBytesActuallyRead);
    /// \deprecated This is for compatibility with the iPort template and should not be
    /// used as it does nothing.
    /// \param userData
    /// \param handler
    void registerDataReceivedHandler(void* userData, DataReceivedHandler handler) {}
    /// \deprecated This is for compatibility with the iPort template and should not be
    /// used as it does nothing.
    void unregisterDataReceivedHandler() {}
    /// \deprecated This function takes in C-style inputs and will be removed in future
    /// releases.
    /// Writes the data in the character array aData with length aLength to the serial
    /// port and sends it.
    /// \param aData Data to be sent over the serial port
    /// \param aLength Length of the data array to send
    void write(const char aData[], size_t aLength);

protected:
    /// \brief Overridden function that contains the thread loop.  This should only be
    /// called by the thread base class.
    virtual void run();

private:
    // General Variables
    /// \brief Flag indicating if the serial port is currently open
    bool m_isOpen;
    /// \brief CriticalSection to protect access to the packet vector
    CriticalSection* m_packetQueueCriticalSection;
    /// \brief CriticalSection to protect access to the read buffer
    CriticalSection* m_readBufferCriticalSection;
    /// \brief Queue that stores the packets after they are processed from data
    /// read from the port.
	std::queue<vn::protocol::uart::Packet*> m_packetQueue;
	/// \brief The name of the port to connect to
	std::string m_portName;
    /// \brief Vector of characters to store data read from the port for processing.  All
    /// data read is appeneded to the end of the vector and is only removed if a packet is
    /// found.  When a packet is found the packet's data, as well as all previous data, is
    /// removed.
    std::vector<char> m_readBuffer;
    /// \brief Number of stop bits the port should use
    StopBits m_stopBits;
    /// \brief The baud rate to connect the port at
    uint32_t m_baudRate;
    /// \brief The number of packets that has been received while the program is active.
    uint32_t m_packetCounter;

#ifdef _WIN32
    // Windows Based Variables
    /// \brief Handle to the serial port for reading and writing
    HANDLE m_serialPortHandle;
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
#error "New SerialPort class does not yet support Unix systems"
#else
#error "Unknown System"
#endif

    /// \brief CTOR
    /// Made private to enforce having at least a port name and baud rate before open
    /// can be called.
    NewSerialPort();
    /// \brief Ensures that conditions are correct for processsing an ascii packet if one
    /// has been detected.  If the conditions are met it passes the data to the packet class
    /// to process the ascii data.
    /// \param aAsciiStartItr Iterator to the start byte of the ascii packet
    /// \param aAsciiEndItr Iterator to the end byte of the ascii packet.  This value may
    /// be invalid under certain conditions such as an incomplete packet.
    vn::protocol::uart::Packet* processASCIIData(std::vector<char>::iterator aAsciiStartItr,
												 std::vector<char>::iterator aAsciiEndItr);
    /// \brief Ensures that conditions are correct for processing a binary packet if one
    /// has been detected.  If the conditions are met it passes the data to the packet class
    /// to process the binary data.
    /// \param aBinaryItr Iterator to the start byte of the binary packet
    vn::protocol::uart::Packet* processBinaryData(std::vector<char>::iterator aBinaryItr);
    /// \brief This should be called every time data is appeneded to the m_readBuffer.  This
    /// will do a preliminary search for binary and ascii packets and call the correct processing
    /// function to handle any that are possibly detected.
    void processData();
    /// \brief Appends the input aDataBuffer data to the m_readBuffer.
    /// \param aDataBuffer vector of bytes to append to the m_readBuffer; this can be empty.
    void storeReadData(const std::vector<char>& aDataBuffer);
};


class vn_proglib_DLLEXPORT NewSearcher : public NewThread
{
public:
    /// \brief Set of supported baud rates used when searching for a sensor.
    static const std::vector<uint32_t> TestBaudrates ;

    /// CTOR
    NewSearcher();
    /// DTOR
    ~NewSearcher();

    /// \brief Returns a flag to indicate if the searcher has finished searching the
    /// current list of ports.  This differs from the searching function as this function
    /// indicates if the input ports have been searched already.
    /// \return Flag indicating if the searcher has finished searching.
    bool finished();
	/// \brief Returns the number of sensors that have been detected by any of the search routines.
	/// This function returns a zero if a search has not been performed or if one is in progress.
	/// \return The number of detected sensors or zero if no search has been completed.
	size_t getNumSensors();
	/// \brief Returns the port name and baudrate of the detected sensor at aIndex.
	/// Returns an empty string and a zero if a search has not yet completed or if aIndex
	/// is out of range.
	/// \param aIndex The index of the port/baud pair to return.
	/// \return A pair contining the port name and baudrate.
	std::pair<std::string, std::uint32_t> getSensor(size_t aIndex);
	/// \brief Resets the searcher and removes all previous results.
    void reset();
    /// \brief Finds all available serial ports on the system and then searches them at various
    /// baud rates to see if a sensor exists.
    void search();
    /// \brief Searches the port at aPortName at various baud rates to see if a sensor is
    /// attached to it.
    /// \param aPortName The name of the port to search
    void search(const std::string& aPortName);
    /// \brief Searches the input serial port names for sensors at various baud rates.
    /// \param aPortsToCheck Name of the ports to check for sensors.
    void search(const std::vector<std::string>& aPortsToCheck);
    /// Returns a flag that indicates if the searcher is still busy searching for sensors.
    /// This differs from the finish function as this function indicates if the thread
    /// loop is running.
    /// \return Flag indicating if the search thread is running.
    bool searching();

protected:
    virtual void run();

private:
    bool m_isMaster;
    NewSerialPort* m_port;
    std::vector<std::string> m_portList;
    std::vector<std::pair<std::string, uint32_t>> m_validPorts;

    /// CTOR
    /// \brief Takes port name and searches it for a sensor.
    NewSearcher(const std::string& aPortName);
    /// \brief This function is for slave objects only.  It will search all
    /// known bauds to if a sensor is connected to its port.
    void searchBauds();
    /// \brief This function is for master objects only.  It will create a slave
    /// instance for each input port and have them search all known bauds to see
    /// if a sensor is connected.
    void searchPorts();
};
}} // End vn/xplat namespaces

#endif
#endif
