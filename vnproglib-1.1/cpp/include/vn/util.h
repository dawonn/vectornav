#ifndef _VNPROTOCOL_UART_UTIL_H_
#define _VNPROTOCOL_UART_UTIL_H_

#include <string>
#include "types.h"

namespace vn {
namespace protocol {
namespace uart {

// Utility functions.

/// \brief Converts an AsciiAsync enum into a string.
///
/// \param[in] val The AsciiAsync enum value to convert to string.
/// \return The converted value.
std::string str(AsciiAsync val);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// AsciiAsync enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, AsciiAsync e);

/// \brief Converts a SensorError enum into a string.
///
/// \param[in] val The SensorError enum value to convert to string.
/// \return The converted value.
std::string str(SensorError val);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SensorError enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SensorError e);

/// \brief Converts a SyncInMode enum into a string.
///
/// \param[in] val The SyncInMode enum value to convert to string.
/// \return The converted value.
std::string str(SyncInMode val);

/// \brief Converts a SyncInEdge enum into a string.
///
/// \param[in] val The SyncInEdge enum value to convert to string.
/// \return The converted value.
std::string str(SyncInEdge val);

/// \brief Converts a SyncOutMode enum into a string.
///
/// \param[in] val The SyncOutMode enum value to convert to string.
/// \return The converted value.
std::string str(SyncOutMode val);

/// \brief Converts a SyncOutPolarity enum into a string.
///
/// \param[in] val The SyncOutPolarity enum value to convert to string.
/// \return The converted value.
std::string str(SyncOutPolarity val);

/// \brief Converts a CountMode enum into a string.
///
/// \param[in] val The CountMode enum value to convert to string.
/// \return The converted value.
std::string str(CountMode val);

/// \brief Converts a StatusMode enum into a string.
///
/// \param[in] val The StatusMode enum value to convert to string.
/// \return The converted value.
std::string str(StatusMode val);

/// \brief Converts a ChecksumMode enum into a string.
///
/// \param[in] val The ChecksumMode enum value to convert to string.
/// \return The converted value.
std::string str(ChecksumMode val);

/// \brief Converts a ErrorMode enum into a string.
///
/// \param[in] val The ErrorMode enum value to convert to string.
/// \return The converted value.
std::string str(ErrorMode val);

/// \brief Converts a FilterMode enum into a string.
///
/// \param[in] val The FilterMode enum value to convert to string.
/// \return The converted value.
std::string str(FilterMode val);

/// \brief Converts a IntegrationFrame enum into a string.
///
/// \param[in] val The IntegrationFrame enum value to convert to string.
/// \return The converted value.
std::string str(IntegrationFrame val);

/// \brief Converts a CompensationMode enum into a string.
///
/// \param[in] val The CompensationMode enum value to convert to string.
/// \return The converted value.
std::string str(CompensationMode val);

/// \brief Converts a GpsFix enum into a string.
///
/// \param[in] val The GpsFix enum value to convert to string.
/// \return The converted value.
std::string str(GpsFix val);

/// \brief Converts a GpsMode enum into a string.
///
/// \param[in] val The GpsMode enum value to convert to string.
/// \return The converted value.
std::string str(GpsMode val);

/// \brief Converts a PpsSource enum into a string.
///
/// \param[in] val The PpsSource enum value to convert to string.
/// \return The converted value.
std::string str(PpsSource val);

/// \brief Converts a VpeEnable enum into a string.
///
/// \param[in] val The VpeEnable enum value to convert to string.
/// \return The converted value.
std::string str(VpeEnable val);

/// \brief Converts a HeadingMode enum into a string.
///
/// \param[in] val The HeadingMode enum value to convert to string.
/// \return The converted value.
std::string str(HeadingMode val);

/// \brief Converts a VpeMode enum into a string.
///
/// \param[in] val The VpeMode enum value to convert to string.
/// \return The converted value.
std::string str(VpeMode val);

/// \brief Converts a Scenario enum into a string.
///
/// \param[in] val The Scenario enum value to convert to string.
/// \return The converted value.
std::string str(Scenario val);

/// \brief Converts a HsiMode enum into a string.
///
/// \param[in] val The HsiMode enum value to convert to string.
/// \return The converted value.
std::string str(HsiMode val);

/// \brief Converts a HsiOutput enum into a string.
///
/// \param[in] val The HsiOutput enum value to convert to string.
/// \return The converted value.
std::string str(HsiOutput val);

/// \brief Converts a VelocityCompensationMode enum into a string.
///
/// \param[in] val The VelocityCompensationMode enum value to convert to string.
/// \return The converted value.
std::string str(VelocityCompensationMode val);

/// \brief Converts a MagneticMode enum into a string.
///
/// \param[in] val The MagneticMode enum value to convert to string.
/// \return The converted value.
std::string str(MagneticMode val);

/// \brief Converts a ExternalSensorMode enum into a string.
///
/// \param[in] val The ExternalSensorMode enum value to convert to string.
/// \return The converted value.
std::string str(ExternalSensorMode val);

/// \brief Converts a FoamInit enum into a string.
///
/// \param[in] val The FoamInit enum value to convert to string.
/// \return The converted value.
std::string str(FoamInit val);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncInMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncInMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncInEdge enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncInEdge e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncOutMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncOutMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// SyncOutPolarity enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, SyncOutPolarity e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// CountMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, CountMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// StatusMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, StatusMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// ChecksumMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, ChecksumMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// ErrorMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, ErrorMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// FilterMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, FilterMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// IntegrationFrame enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, IntegrationFrame e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// CompensationMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, CompensationMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// GpsFix enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, GpsFix e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// GpsMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, GpsMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// PpsSource enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, PpsSource e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// VpeEnable enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, VpeEnable e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// HeadingMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, HeadingMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// VpeMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, VpeMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// Scenario enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, Scenario e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// HsiMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, HsiMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// HsiOutput enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, HsiOutput e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// VelocityCompensationMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, VelocityCompensationMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// MagneticMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, MagneticMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// ExternalSensorMode enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, ExternalSensorMode e);

/// \brief Overloads the ostream << operator for easy usage in displaying
/// FoamInit enums.
///
/// \param[in] out The ostream being output to.
/// \param[in] e The enum to output to ostream.
/// \return Reference to the current ostream.
std::ostream& operator<<(std::ostream& out, FoamInit e);

}
}
}

#endif
