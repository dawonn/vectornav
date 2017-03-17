#include "vn/util.h"

#include "vn/exceptions.h"

#include <iostream>
using namespace std;

namespace vn {

string to_binary_string(uint16_t v)
{
	string r("0000000000000000");

	for (int i = 15; i >= 0; --i)
	{
		if (1 & v)
			r[i] = '1';

		v = v >> 1;
	}

	return r;
}

uint32_t parse_binary_string(const string& s)
{
  uint32_t v = 0;
  uint32_t curMask = 1;

  for (int i = s.length() - 1; i >= 0; --i)
  {
    char c = s[i];

    if (c == '1')
      v += curMask;
    else if (c != '0')
      throw new vn::invalid_format();

    curMask = curMask << 1;
  }

  return v;
}

vector<string> split(const string &s, const string &delims, bool removeEmpties)
{
	vector<string> r;

	size_t n = 0;
	size_t i = 0;
	for (i = 0; i < s.length(); ++i)
	{
		char c = s[i];

		for (size_t j = 0; j < delims.length(); ++j)
		{
			if (c == delims[j])
			{
				if (!removeEmpties)
					r.push_back(s.substr(n, i - n));
				else if (i != n)
					r.push_back(s.substr(n, i - n));

				n = i + 1;
				break;
			}
		}
	}

	if (n != i - 1)
		r.push_back(s.substr(n));

	return r;
}







namespace protocol {
namespace uart {

char* vnstrtok(char* str, size_t& startIndex);

string str(AsciiAsync val)
{
	switch (val)
	{
		case VNOFF:
			return "VNOFF";
		case VNYPR:
			return "VNYPR";
		case VNQTN:
			return "VNQTN";
		#ifdef INTERNAL
		case VNQTM:
			return "VNQTM";
		case VNQTA:
			return "VNQTA";
		case VNQTR:
			return "VNQTR";
		case VNQMA:
			return "VNQMA";
		case VNQAR:
			return "VNQAR";
		#endif
		case VNQMR:
			return "VNQMR";
		#ifdef INTERNAL
		case VNDCM:
			return "VNDCM";
		#endif
		case VNMAG:
			return "VNMAG";
		case VNACC:
			return "VNACC";
		case VNGYR:
			return "VNGYR";
		case VNMAR:
			return "VNMAR";
		case VNYMR:
			return "VNYMR";
		#ifdef INTERNAL
		case VNYCM:
			return "VNYCM";
		#endif
		case VNYBA:
			return "VNYBA";
		case VNYIA:
			return "VNYIA";
		#ifdef INTERNAL
		case VNICM:
			return "VNICM";
		#endif
		case VNIMU:
			return "VNIMU";
		case VNGPS:
			return "VNGPS";
		case VNGPE:
			return "VNGPE";
		case VNINS:
			return "VNINS";
		case VNINE:
			return "VNINE";
		case VNISL:
			return "VNISL";
		case VNISE:
			return "VNISE";
		case VNDTV:
			return "VNDTV";
		#ifdef INTERNAL
		case VNRAW:
			return "VNRAW";
		case VNCMV:
			return "VNCMV";
		case VNSTV:
			return "VNSTV";
		case VNCOV:
			return "VNCOV";
		#endif
		default:
			throw invalid_argument("val");
	}
}

ostream& operator<<(ostream& out, AsciiAsync e)
{
	out << str(e);
	return out;
}

string str(SENSORERROR val)
{
	switch (val)
	{
	case ERR_HARD_FAULT:
		return "HardFault";
	case ERR_SERIAL_BUFFER_OVERFLOW:
		return "SerialBufferOverflow";
	case ERR_INVALID_CHECKSUM:
		return "InvalidChecksum";
	case ERR_INVALID_COMMAND:
		return "InvalidCommand";
	case ERR_NOT_ENOUGH_PARAMETERS:
		return "NotEnoughParameters";
	case ERR_TOO_MANY_PARAMETERS:
		return "TooManyParameters";
	case ERR_INVALID_PARAMETER:
		return "InvalidParameter";
	case ERR_INVALID_REGISTER:
		return "InvalidRegister";
	case ERR_UNAUTHORIZED_ACCESS:
		return "UnauthorizedAccess";
	case ERR_WATCHDOG_RESET:
		return "WatchdogReset";
	case ERR_OUTPUT_BUFFER_OVERFLOW:
		return "OutputBufferOverflow";
	case ERR_INSUFFICIENT_BAUD_RATE:
		return "InsufficientBaudRate";
	case ERR_ERROR_BUFFER_OVERFLOW:
		return "ErrorBufferOverflow";
	default:
		throw invalid_argument("val");
	}
}

ostream& operator<<(ostream& out, SENSORERROR e)
{
	out << str(e);
	return out;
}

string to_string(SyncInMode val)
{
	switch (val)
	{
		#ifdef INTERNAL
		case SYNCINMODE_COUNT2:
			 return "Count2";
		case SYNCINMODE_ADC2:
			 return "Adc2";
		case SYNCINMODE_ASYNC2:
			 return "Async2";
		#endif
		case SYNCINMODE_COUNT:
			 return "Count";
		case SYNCINMODE_IMU:
			 return "Imu";
		case SYNCINMODE_ASYNC:
			 return "Async";
		default:
			throw invalid_argument("val");
	}
}

string to_string(SyncInEdge val)
{
	switch (val)
	{
		case SYNCINEDGE_RISING:
			 return "Rising";
		case SYNCINEDGE_FALLING:
			 return "Falling";
		default:
			throw invalid_argument("val");
	}
}

string to_string(SyncOutMode val)
{
	switch (val)
	{
		case SYNCOUTMODE_NONE:
			 return "None";
		case SYNCOUTMODE_ITEMSTART:
			 return "ItemStart";
		case SYNCOUTMODE_IMUREADY:
			 return "ImuReady";
		case SYNCOUTMODE_INS:
			 return "Ins";
		case SYNCOUTMODE_GPSPPS:
			 return "GpsPps";
		default:
			throw invalid_argument("val");
	}
}

string to_string(SyncOutPolarity val)
{
	switch (val)
	{
		case SYNCOUTPOLARITY_NEGATIVE:
			 return "Negative";
		case SYNCOUTPOLARITY_POSITIVE:
			 return "Positive";
		default:
			throw invalid_argument("val");
	}
}

string to_string(CountMode val)
{
	switch (val)
	{
		case COUNTMODE_NONE:
			 return "None";
		case COUNTMODE_SYNCINCOUNT:
			 return "SyncInCount";
		case COUNTMODE_SYNCINTIME:
			 return "SyncInTime";
		case COUNTMODE_SYNCOUTCOUNTER:
			 return "SyncOutCounter";
		case COUNTMODE_GPSPPS:
			 return "GpsPps";
		default:
			throw invalid_argument("val");
	}
}

string to_string(StatusMode val)
{
	switch (val)
	{
		case STATUSMODE_OFF:
			 return "Off";
		case STATUSMODE_VPESTATUS:
			 return "VpeStatus";
		case STATUSMODE_INSSTATUS:
			 return "InsStatus";
		default:
			throw invalid_argument("val");
	}
}

string to_string(ChecksumMode val)
{
	switch (val)
	{
		case CHECKSUMMODE_OFF:
			 return "Off";
		case CHECKSUMMODE_CHECKSUM:
			 return "Checksum";
		case CHECKSUMMODE_CRC:
			 return "Crc";
		default:
			throw invalid_argument("val");
	}
}

string to_string(ErrorMode val)
{
	switch (val)
	{
		case ERRORMODE_IGNORE:
			 return "Ignore";
		case ERRORMODE_SEND:
			 return "Send";
		case ERRORMODE_SENDANDOFF:
			 return "SendAndOff";
		default:
			throw invalid_argument("val");
	}
}

string to_string(FilterMode val)
{
	switch (val)
	{
		case FILTERMODE_NOFILTERING:
			 return "NoFiltering";
		case FILTERMODE_ONLYRAW:
			 return "OnlyRaw";
		case FILTERMODE_ONLYCOMPENSATED:
			 return "OnlyCompensated";
		case FILTERMODE_BOTH:
			 return "Both";
		default:
			throw invalid_argument("val");
	}
}

string to_string(IntegrationFrame val)
{
	switch (val)
	{
		case INTEGRATIONFRAME_BODY:
			 return "Body";
		case INTEGRATIONFRAME_NED:
			 return "Ned";
		default:
			throw invalid_argument("val");
	}
}

string to_string(CompensationMode val)
{
	switch (val)
	{
		case COMPENSATIONMODE_NONE:
			 return "None";
		case COMPENSATIONMODE_BIAS:
			 return "Bias";
		default:
			throw invalid_argument("val");
	}
}

string to_string(GpsFix val)
{
	switch (val)
	{
		case GPSFIX_NOFIX:
			 return "NoFix";
		case GPSFIX_TIMEONLY:
			 return "TimeOnly";
		case GPSFIX_2D:
			 return "2D";
		case GPSFIX_3D:
			 return "3D";
		default:
			throw invalid_argument("val");
	}
}

string to_string(GpsMode val)
{
	switch (val)
	{
		case GPSMODE_ONBOARDGPS:
			 return "OnBoardGps";
		case GPSMODE_EXTERNALGPS:
			 return "ExternalGps";
		case GPSMODE_EXTERNALVN200GPS:
			 return "ExternalVn200Gps";
		default:
			throw invalid_argument("val");
	}
}

string to_string(PpsSource val)
{
	switch (val)
	{
		case PPSSOURCE_GPSPPSRISING:
			 return "GpsPpsRising";
		case PPSSOURCE_GPSPPSFALLING:
			 return "GpsPpsFalling";
		case PPSSOURCE_SYNCINRISING:
			 return "SyncInRising";
		case PPSSOURCE_SYNCINFALLING:
			 return "SyncInFalling";
		default:
			throw invalid_argument("val");
	}
}

string to_string(VpeEnable val)
{
	switch (val)
	{
		case VPEENABLE_DISABLE:
			 return "Disable";
		case VPEENABLE_ENABLE:
			 return "Enable";
		default:
			throw invalid_argument("val");
	}
}

string to_string(HeadingMode val)
{
	switch (val)
	{
		case HEADINGMODE_ABSOLUTE:
			 return "Absolute";
		case HEADINGMODE_RELATIVE:
			 return "Relative";
		case HEADINGMODE_INDOOR:
			 return "Indoor";
		default:
			throw invalid_argument("val");
	}
}

string to_string(VpeMode val)
{
	switch (val)
	{
		case VPEMODE_OFF:
			 return "Off";
		case VPEMODE_MODE1:
			 return "Mode1";
		default:
			throw invalid_argument("val");
	}
}

string to_string(Scenario val)
{
	switch (val)
	{
		case SCENARIO_AHRS:
			 return "Ahrs";
		case SCENARIO_INSWITHPRESSURE:
			 return "InsWithPressure";
		case SCENARIO_INSWITHOUTPRESSURE:
			 return "InsWithoutPressure";
		case SCENARIO_GPSMOVINGBASELINEDYNAMIC:
			 return "GpsMovingBaselineDynamic";
		case SCENARIO_GPSMOVINGBASELINESTATIC:
			 return "GpsMovingBaselineStatic";
		default:
			throw invalid_argument("val");
	}
}

string to_string(HsiMode val)
{
	switch (val)
	{
		case HSIMODE_OFF:
			 return "Off";
		case HSIMODE_RUN:
			 return "Run";
		case HSIMODE_RESET:
			 return "Reset";
		default:
			throw invalid_argument("val");
	}
}

string to_string(HsiOutput val)
{
	switch (val)
	{
		case HSIOUTPUT_NOONBOARD:
			 return "NoOnboard";
		case HSIOUTPUT_USEONBOARD:
			 return "UseOnboard";
		default:
			throw invalid_argument("val");
	}
}

string to_string(VelocityCompensationMode val)
{
	switch (val)
	{
		case VELOCITYCOMPENSATIONMODE_DISABLED:
			 return "Disabled";
		case VELOCITYCOMPENSATIONMODE_BODYMEASUREMENT:
			 return "BodyMeasurement";
		default:
			throw invalid_argument("val");
	}
}

string to_string(MagneticMode val)
{
	switch (val)
	{
		case MAGNETICMODE_2D:
			 return "2D";
		case MAGNETICMODE_3D:
			 return "3D";
		default:
			throw invalid_argument("val");
	}
}

string to_string(ExternalSensorMode val)
{
	switch (val)
	{
		case EXTERNALSENSORMODE_INTERNAL:
			 return "Internal";
		case EXTERNALSENSORMODE_EXTERNAL200HZ:
			 return "External200Hz";
		case EXTERNALSENSORMODE_EXTERNALONUPDATE:
			 return "ExternalOnUpdate";
		default:
			throw invalid_argument("val");
	}
}

string to_string(FoamInit val)
{
	switch (val)
	{
		case FOAMINIT_NOFOAMINIT:
			 return "NoFoamInit";
		case FOAMINIT_FOAMINITPITCHROLL:
			 return "FoamInitPitchRoll";
		case FOAMINIT_FOAMINITHEADINGPITCHROLL:
			 return "FoamInitHeadingPitchRoll";
		case FOAMINIT_FOAMINITPITCHROLLCOVARIANCE:
			 return "FoamInitPitchRollCovariance";
		case FOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE:
			 return "FoamInitHeadingPitchRollCovariance";
		default:
			throw invalid_argument("val");
	}
}

ostream& operator<<(ostream& out, SyncInMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, SyncInEdge e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, SyncOutMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, SyncOutPolarity e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, CountMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, StatusMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, ChecksumMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, ErrorMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, FilterMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, IntegrationFrame e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, CompensationMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, GpsFix e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, GpsMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, PpsSource e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, VpeEnable e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, HeadingMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, VpeMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, Scenario e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, HsiMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, HsiOutput e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, VelocityCompensationMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, MagneticMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, ExternalSensorMode e)
{
	out << to_string(e);
	return out;
}

ostream& operator<<(ostream& out, FoamInit e)
{
	out << to_string(e);
	return out;
}

}
}
}
