#include "vn/types.h"
#include <algorithm>
#include <sstream>
#include "vn/exceptions.h"

using namespace std;

namespace vn {
namespace protocol {
namespace uart {

VpeStatus::VpeStatus()
{
}

VpeStatus::VpeStatus(uint16_t raw)
{
	attitudeQuality = 0x0003 & raw;
	gyroSaturation = (0x0004 & raw) != 0;
	gyroSaturationRecovery = (0x0008 & raw) != 0;
	magDisturbance = (0x0030 & raw) >> 4;
	magSaturation = (0x0040 & raw) != 0;
	accDisturbance = (0x0180 & raw) >> 7;
	accSaturation = (0x0200 & raw) != 0;
	knownMagDisturbance = (0x0800 & raw) != 0;
	knownAccelDisturbance = (0x1000 & raw) != 0;
}


COMMONGROUP operator|(COMMONGROUP lhs, COMMONGROUP rhs)
{
	return COMMONGROUP(int(lhs) | int(rhs));
}

TIMEGROUP operator|(TIMEGROUP lhs, TIMEGROUP rhs)
{
	return TIMEGROUP(int(lhs) | int(rhs));
}

IMUGROUP operator|(IMUGROUP lhs, IMUGROUP rhs)
{
	return IMUGROUP(int(lhs) | int(rhs));
}

GPSGROUP operator|(GPSGROUP lhs, GPSGROUP rhs)
{
	return GPSGROUP(int(lhs) | int(rhs));
}

ATTITUDEGROUP operator|(ATTITUDEGROUP lhs, ATTITUDEGROUP rhs)
{
	return ATTITUDEGROUP(int(lhs) | int(rhs));
}

INSGROUP operator|(INSGROUP lhs, INSGROUP rhs)
{
	return INSGROUP(int(lhs) | int(rhs));
}

string to_string(ASYNCMODE val)
{
  switch (val)
    {
      case ASYNCMODE_NONE:
        return "None";
      case ASYNCMODE_PORT1:
        return "Port1";
      case ASYNCMODE_PORT2:
        return "Port2";
      case ASYNCMODE_BOTH:
        return "Both";
      default:
        throw invalid_argument("val");
    }
}

ostream& operator<<(ostream& out, ASYNCMODE e)
{
	return out << to_string(e);
}

string to_string(COMMONGROUP val)
{
  if (!val)
    return "None";

  stringstream s;

  if (val & COMMONGROUP_TIMESTARTUP)
    s << "TimeStartup";
  if (val & COMMONGROUP_TIMEGPS)
    s << (s.tellp() ? "|" : "") << "TimeGps";
  if (val & COMMONGROUP_TIMESYNCIN)
    s << (s.tellp() ? "|" : "") << "TimeSyncIn";
  if (val & COMMONGROUP_YAWPITCHROLL)
    s << (s.tellp() ? "|" : "") << "YawPitchRoll";
  if (val & COMMONGROUP_QUATERNION)
    s << (s.tellp() ? "|" : "") << "Quaternion";
  if (val & COMMONGROUP_ANGULARRATE)
    s << (s.tellp() ? "|" : "") << "AngularRate";
  if (val & COMMONGROUP_POSITION)
    s << (s.tellp() ? "|" : "") << "Position";
  if (val & COMMONGROUP_VELOCITY)
    s << (s.tellp() ? "|" : "") << "Velocity";
  if (val & COMMONGROUP_ACCEL)
    s << (s.tellp() ? "|" : "") << "Accel";
  if (val & COMMONGROUP_IMU)
    s << (s.tellp() ? "|" : "") << "Imu";
  if (val & COMMONGROUP_MAGPRES)
    s << (s.tellp() ? "|" : "") << "MagPres";
  if (val & COMMONGROUP_DELTATHETA)
    s << (s.tellp() ? "|" : "") << "DeltaTheta";
  if (val & COMMONGROUP_INSSTATUS)
    s << (s.tellp() ? "|" : "") << "InsStatus";
  if (val & COMMONGROUP_SYNCINCNT)
    s << (s.tellp() ? "|" : "") << "SyncInCnt";
  if (val & COMMONGROUP_TIMEGPSPPS)
    s << (s.tellp() ? "|" : "") << "TimeGpsPps";

  return s.str();
}

ostream& operator<<(ostream& out, COMMONGROUP e)
{
  return out << to_string(e);
}

string to_string(TIMEGROUP val)
{
  if (!val)
    return "None";

  stringstream s;

  if (val & TIMEGROUP_TIMESTARTUP)
    s << "TimeStartup";
  if (val & TIMEGROUP_TIMEGPS)
    s << (s.tellp() ? "|" : "") << "TimeGps";
  if (val & TIMEGROUP_GPSTOW)
    s << (s.tellp() ? "|" : "") << "GpsTow";
  if (val & TIMEGROUP_GPSWEEK)
    s << (s.tellp() ? "|" : "") << "GpsWeek";
  if (val & TIMEGROUP_TIMESYNCIN)
    s << (s.tellp() ? "|" : "") << "TimeSyncIn";
  if (val & TIMEGROUP_TIMEGPSPPS)
    s << (s.tellp() ? "|" : "") << "TimeGpsPps";
  if (val & TIMEGROUP_TIMEUTC)
    s << (s.tellp() ? "|" : "") << "TimeUtc";
  if (val & TIMEGROUP_SYNCINCNT)
    s << (s.tellp() ? "|" : "") << "SyncInCnt";

  return s.str();
}

ostream& operator<<(ostream& out, TIMEGROUP e)
{
  return out << to_string(e);
}

string to_string(IMUGROUP val)
{
  if (!val)
    return "None";

  stringstream s;

  if (val & IMUGROUP_IMUSTATUS)
    s << "ImuStatus";
  if (val & IMUGROUP_UNCOMPMAG)
    s << (s.tellp() ? "|" : "") << "UncompMag";
  if (val & IMUGROUP_UNCOMPACCEL)
    s << (s.tellp() ? "|" : "") << "UncompAccel";
  if (val & IMUGROUP_UNCOMPGYRO)
    s << (s.tellp() ? "|" : "") << "UncompGyro";
  if (val & IMUGROUP_TEMP)
    s << (s.tellp() ? "|" : "") << "Temp";
  if (val & IMUGROUP_PRES)
    s << (s.tellp() ? "|" : "") << "Pres";
  if (val & IMUGROUP_DELTATHETA)
    s << (s.tellp() ? "|" : "") << "DeltaTheta";
  if (val & IMUGROUP_DELTAVEL)
    s << (s.tellp() ? "|" : "") << "DeltaVel";
  if (val & IMUGROUP_MAG)
    s << (s.tellp() ? "|" : "") << "Mag";
  if (val & IMUGROUP_ACCEL)
    s << (s.tellp() ? "|" : "") << "Accel";
  if (val & IMUGROUP_ANGULARRATE)
    s << (s.tellp() ? "|" : "") << "AngularRate";
  if (val & IMUGROUP_SENSSAT)
    s << (s.tellp() ? "|" : "") << "SensSat";
  #ifdef INTERNAL
  if (val & IMUGROUP_RAW)
    s << (s.tellp() ? "|" : "") << "Raw";
  #endif

  return s.str();
}

ostream& operator<<(ostream& out, IMUGROUP e)
{
  return out << to_string(e);
}

string to_string(GPSGROUP val)
{
  if (!val)
    return "None";

  stringstream s;

  if (val & GPSGROUP_UTC)
    s << "Utc";
  if (val & GPSGROUP_TOW)
    s << (s.tellp() ? "|" : "") << "Tow";
  if (val & GPSGROUP_WEEK)
    s << (s.tellp() ? "|" : "") << "Week";
  if (val & GPSGROUP_NUMSATS)
    s << (s.tellp() ? "|" : "") << "NumSats";
  if (val & GPSGROUP_FIX)
    s << (s.tellp() ? "|" : "") << "Fix";
  if (val & GPSGROUP_POSLLA)
    s << (s.tellp() ? "|" : "") << "PosLla";
  if (val & GPSGROUP_POSECEF)
    s << (s.tellp() ? "|" : "") << "PosEcef";
  if (val & GPSGROUP_VELNED)
    s << (s.tellp() ? "|" : "") << "VelNed";
  if (val & GPSGROUP_VELECEF)
    s << (s.tellp() ? "|" : "") << "VelEcef";
  if (val & GPSGROUP_POSU)
    s << (s.tellp() ? "|" : "") << "PosU";
  if (val & GPSGROUP_VELU)
    s << (s.tellp() ? "|" : "") << "VelU";
  if (val & GPSGROUP_TIMEU)
    s << (s.tellp() ? "|" : "") << "TimeU";

  return s.str();
}

ostream& operator<<(ostream& out, GPSGROUP e)
{
  return out << to_string(e);
}

string to_string(ATTITUDEGROUP val)
{
  if (!val)
    return "None";

  stringstream s;

  if (val & ATTITUDEGROUP_VPESTATUS)
    s << "VpeStatus";
  if (val & ATTITUDEGROUP_YAWPITCHROLL)
    s << (s.tellp() ? "|" : "") << "YawPitchRoll";
  if (val & ATTITUDEGROUP_QUATERNION)
    s << (s.tellp() ? "|" : "") << "Quaternion";
  if (val & ATTITUDEGROUP_DCM)
    s << (s.tellp() ? "|" : "") << "Dcm";
  if (val & ATTITUDEGROUP_MAGNED)
    s << (s.tellp() ? "|" : "") << "MagNed";
  if (val & ATTITUDEGROUP_ACCELNED)
    s << (s.tellp() ? "|" : "") << "AccelNed";
  if (val & ATTITUDEGROUP_LINEARACCELBODY)
    s << (s.tellp() ? "|" : "") << "LinearAccelBody";
  if (val & ATTITUDEGROUP_LINEARACCELNED)
    s << (s.tellp() ? "|" : "") << "LinearAccelNed";
  #ifdef INTERNAL
  if (val & ATTITUDEGROUP_YPRRATE)
    s << (s.tellp() ? "|" : "") << "YprRate";
  if (val & ATTITUDEGROUP_STATEAHRS)
    s << (s.tellp() ? "|" : "") << "StateAhrs";
  if (val & ATTITUDEGROUP_COVAHRS)
    s << (s.tellp() ? "|" : "") << "CovAhrs";
  #endif

  return s.str();
}

ostream& operator<<(ostream& out, ATTITUDEGROUP e)
{
  return out << to_string(e);
}

string to_string(INSGROUP val)
{
  if (!val)
    return "None";

  stringstream s;

  if (val & INSGROUP_INSSTATUS)
    s << "InsStatus";
  if (val & INSGROUP_POSLLA)
    s << (s.tellp() ? "|" : "") << "PosLla";
  if (val & INSGROUP_POSECEF)
    s << (s.tellp() ? "|" : "") << "PosEcef";
  if (val & INSGROUP_VELBODY)
    s << (s.tellp() ? "|" : "") << "VelBody";
  if (val & INSGROUP_VELNED)
    s << (s.tellp() ? "|" : "") << "VelNed";
  if (val & INSGROUP_VELECEF)
    s << (s.tellp() ? "|" : "") << "VelEcef";
  if (val & INSGROUP_MAGECEF)
    s << (s.tellp() ? "|" : "") << "MagEcef";
  if (val & INSGROUP_ACCELECEF)
    s << (s.tellp() ? "|" : "") << "AccelEcef";
  if (val & INSGROUP_LINEARACCELECEF)
    s << (s.tellp() ? "|" : "") << "LinearAccelEcef";
  if (val & INSGROUP_POSU)
    s << (s.tellp() ? "|" : "") << "PosU";
  if (val & INSGROUP_VELU)
    s << (s.tellp() ? "|" : "") << "VelU";
  #ifdef INTERNAL
  if (val & INSGROUP_STATEINS)
    s << (s.tellp() ? "|" : "") << "StateIns";
  if (val & INSGROUP_COVINS)
    s << (s.tellp() ? "|" : "") << "CovIns";
  #endif

  return s.str();
}

ostream& operator<<(ostream& out, INSGROUP e)
{
  return out << to_string(e);
}

}
}

using namespace vn::protocol::uart;

bool parse(const std::string &in, AsciiAsync &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("vnoff") == 0) { val = VNOFF; return true; }
		if (norm.compare("vnypr") == 0) { val = VNYPR; return true; }
		if (norm.compare("vnqtn") == 0) { val = VNQTN; return true; }
		#ifdef INTERNAL
		if (norm.compare("vnqtm") == 0) { val = VNQTM; return true; }
		if (norm.compare("vnqta") == 0) { val = VNQTA; return true; }
		if (norm.compare("vnqtr") == 0) { val = VNQTR; return true; }
		if (norm.compare("vnqma") == 0) { val = VNQMA; return true; }
		if (norm.compare("vnqar") == 0) { val = VNQAR; return true; }
		#endif
		if (norm.compare("vnqmr") == 0) { val = VNQMR; return true; }
		#ifdef INTERNAL
		if (norm.compare("vndcm") == 0) { val = VNDCM; return true; }
        #endif
		if (norm.compare("vnmag") == 0) { val = VNMAG; return true; }
		if (norm.compare("vnacc") == 0) { val = VNACC; return true; }
		if (norm.compare("vngyr") == 0) { val = VNGYR; return true; }
		if (norm.compare("vnmar") == 0) { val = VNMAR; return true; }
		if (norm.compare("vnymr") == 0) { val = VNYMR; return true; }
		#ifdef INTERNAL
		if (norm.compare("vnycm") == 0) { val = VNYCM; return true; }
        #endif
		if (norm.compare("vnyba") == 0) { val = VNYBA; return true; }
		if (norm.compare("vnyia") == 0) { val = VNYIA; return true; }
		#ifdef INTERNAL
		if (norm.compare("vnicm") == 0) { val = VNICM; return true; }
        #endif
		if (norm.compare("vnimu") == 0) { val = VNIMU; return true; }
		if (norm.compare("vngps") == 0) { val = VNGPS; return true; }
		if (norm.compare("vngpe") == 0) { val = VNGPE; return true; }
		if (norm.compare("vnins") == 0) { val = VNINS; return true; }
		if (norm.compare("vnine") == 0) { val = VNINE; return true; }
		if (norm.compare("vnisl") == 0) { val = VNISL; return true; }
		if (norm.compare("vnise") == 0) { val = VNISE; return true; }
		if (norm.compare("vndtv") == 0) { val = VNDTV; return true; }
		#ifdef INTERNAL
		if (norm.compare("vnraw") == 0) { val = VNRAW; return true; }
		if (norm.compare("vncmv") == 0) { val = VNCMV; return true; }
		if (norm.compare("vnstv") == 0) { val = VNSTV; return true; }
		if (norm.compare("vncov") == 0) { val = VNCOV; return true; }
        #endif
	}
	else
	{
		if (in.compare("VNOFF") == 0) { val = VNOFF; return true; }
		if (in.compare("VNYPR") == 0) { val = VNYPR; return true; }
		if (in.compare("VNQTN") == 0) { val = VNQTN; return true; }
		#ifdef INTERNAL
		if (in.compare("VNQTM") == 0) { val = VNQTM; return true; }
		if (in.compare("VNQTA") == 0) { val = VNQTA; return true; }
		if (in.compare("VNQTR") == 0) { val = VNQTR; return true; }
		if (in.compare("VNQMA") == 0) { val = VNQMA; return true; }
		if (in.compare("VNQAR") == 0) { val = VNQAR; return true; }
		#endif
		if (in.compare("VNQMR") == 0) { val = VNQMR; return true; }
		#ifdef INTERNAL
		if (in.compare("VNDCM") == 0) { val = VNDCM; return true; }
        #endif
		if (in.compare("VNMAG") == 0) { val = VNMAG; return true; }
		if (in.compare("VNACC") == 0) { val = VNACC; return true; }
		if (in.compare("VNGYR") == 0) { val = VNGYR; return true; }
		if (in.compare("VNMAR") == 0) { val = VNMAR; return true; }
		if (in.compare("VNYMR") == 0) { val = VNYMR; return true; }
		#ifdef INTERNAL
		if (in.compare("VNYCM") == 0) { val = VNYCM; return true; }
        #endif
		if (in.compare("VNYBA") == 0) { val = VNYBA; return true; }
		if (in.compare("VNYIA") == 0) { val = VNYIA; return true; }
		#ifdef INTERNAL
		if (in.compare("VNICM") == 0) { val = VNICM; return true; }
        #endif
		if (in.compare("VNIMU") == 0) { val = VNIMU; return true; }
		if (in.compare("VNGPS") == 0) { val = VNGPS; return true; }
		if (in.compare("VNGPE") == 0) { val = VNGPE; return true; }
		if (in.compare("VNINS") == 0) { val = VNINS; return true; }
		if (in.compare("VNINE") == 0) { val = VNINE; return true; }
		if (in.compare("VNISL") == 0) { val = VNISL; return true; }
		if (in.compare("VNISE") == 0) { val = VNISE; return true; }
		if (in.compare("VNDTV") == 0) { val = VNDTV; return true; }
		#ifdef INTERNAL
		if (in.compare("VNRAW") == 0) { val = VNRAW; return true; }
		if (in.compare("VNCMV") == 0) { val = VNCMV; return true; }
		if (in.compare("VNSTV") == 0) { val = VNSTV; return true; }
		if (in.compare("VNCOV") == 0) { val = VNCOV; return true; }
        #endif
	}

	return false;
}
bool parse(const string &in, SyncInMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		#ifdef INTERNAL
		if (norm.compare("count2") == 0) { val = SYNCINMODE_COUNT2; return true; }
		if (norm.compare("adc2") == 0) { val = SYNCINMODE_ADC2; return true; }
		if (norm.compare("async2") == 0) { val = SYNCINMODE_ASYNC2; return true; }
		#endif
		if (norm.compare("count") == 0) { val = SYNCINMODE_COUNT; return true; }
		if (norm.compare("imu") == 0) { val = SYNCINMODE_IMU; return true; }
		if (norm.compare("async") == 0) { val = SYNCINMODE_ASYNC; return true; }
	}
	else
	{
		#ifdef INTERNAL
		if (in.compare("COUNT2") == 0) { val = SYNCINMODE_COUNT2; return true; }
		if (in.compare("ADC2") == 0) { val = SYNCINMODE_ADC2; return true; }
		if (in.compare("ASYNC2") == 0) { val = SYNCINMODE_ASYNC2; return true; }
		#endif
		if (in.compare("COUNT") == 0) { val = SYNCINMODE_COUNT; return true; }
		if (in.compare("IMU") == 0) { val = SYNCINMODE_IMU; return true; }
		if (in.compare("ASYNC") == 0) { val = SYNCINMODE_ASYNC; return true; }
	}

	return false;
}

bool parse(const string &in, SyncInEdge &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("rising") == 0) { val = SYNCINEDGE_RISING; return true; }
		if (norm.compare("falling") == 0) { val = SYNCINEDGE_FALLING; return true; }
	}
	else
	{
		if (in.compare("RISING") == 0) { val = SYNCINEDGE_RISING; return true; }
		if (in.compare("FALLING") == 0) { val = SYNCINEDGE_FALLING; return true; }
	}

	return false;
}

bool parse(const string &in, SyncOutMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("none") == 0) { val = SYNCOUTMODE_NONE; return true; }
		if (norm.compare("itemstart") == 0) { val = SYNCOUTMODE_ITEMSTART; return true; }
		if (norm.compare("imuready") == 0) { val = SYNCOUTMODE_IMUREADY; return true; }
		if (norm.compare("ins") == 0) { val = SYNCOUTMODE_INS; return true; }
		if (norm.compare("gpspps") == 0) { val = SYNCOUTMODE_GPSPPS; return true; }
	}
	else
	{
		if (in.compare("NONE") == 0) { val = SYNCOUTMODE_NONE; return true; }
		if (in.compare("ITEMSTART") == 0) { val = SYNCOUTMODE_ITEMSTART; return true; }
		if (in.compare("IMUREADY") == 0) { val = SYNCOUTMODE_IMUREADY; return true; }
		if (in.compare("INS") == 0) { val = SYNCOUTMODE_INS; return true; }
		if (in.compare("GPSPPS") == 0) { val = SYNCOUTMODE_GPSPPS; return true; }
	}

	return false;
}

bool parse(const string &in, SyncOutPolarity &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("negative") == 0) { val = SYNCOUTPOLARITY_NEGATIVE; return true; }
		if (norm.compare("positive") == 0) { val = SYNCOUTPOLARITY_POSITIVE; return true; }
	}
	else
	{
		if (in.compare("NEGATIVE") == 0) { val = SYNCOUTPOLARITY_NEGATIVE; return true; }
		if (in.compare("POSITIVE") == 0) { val = SYNCOUTPOLARITY_POSITIVE; return true; }
	}

	return false;
}

bool parse(const string &in, CountMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("none") == 0) { val = COUNTMODE_NONE; return true; }
		if (norm.compare("syncincount") == 0) { val = COUNTMODE_SYNCINCOUNT; return true; }
		if (norm.compare("syncintime") == 0) { val = COUNTMODE_SYNCINTIME; return true; }
		if (norm.compare("syncoutcounter") == 0) { val = COUNTMODE_SYNCOUTCOUNTER; return true; }
		if (norm.compare("gpspps") == 0) { val = COUNTMODE_GPSPPS; return true; }
	}
	else
	{
		if (in.compare("NONE") == 0) { val = COUNTMODE_NONE; return true; }
		if (in.compare("SYNCINCOUNT") == 0) { val = COUNTMODE_SYNCINCOUNT; return true; }
		if (in.compare("SYNCINTIME") == 0) { val = COUNTMODE_SYNCINTIME; return true; }
		if (in.compare("SYNCOUTCOUNTER") == 0) { val = COUNTMODE_SYNCOUTCOUNTER; return true; }
		if (in.compare("GPSPPS") == 0) { val = COUNTMODE_GPSPPS; return true; }
	}

	return false;
}

bool parse(const string &in, StatusMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("off") == 0) { val = STATUSMODE_OFF; return true; }
		if (norm.compare("vpestatus") == 0) { val = STATUSMODE_VPESTATUS; return true; }
		if (norm.compare("insstatus") == 0) { val = STATUSMODE_INSSTATUS; return true; }
	}
	else
	{
		if (in.compare("OFF") == 0) { val = STATUSMODE_OFF; return true; }
		if (in.compare("VPESTATUS") == 0) { val = STATUSMODE_VPESTATUS; return true; }
		if (in.compare("INSSTATUS") == 0) { val = STATUSMODE_INSSTATUS; return true; }
	}

	return false;
}

bool parse(const string &in, ChecksumMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("off") == 0) { val = CHECKSUMMODE_OFF; return true; }
		if (norm.compare("checksum") == 0) { val = CHECKSUMMODE_CHECKSUM; return true; }
		if (norm.compare("crc") == 0) { val = CHECKSUMMODE_CRC; return true; }
	}
	else
	{
		if (in.compare("OFF") == 0) { val = CHECKSUMMODE_OFF; return true; }
		if (in.compare("CHECKSUM") == 0) { val = CHECKSUMMODE_CHECKSUM; return true; }
		if (in.compare("CRC") == 0) { val = CHECKSUMMODE_CRC; return true; }
	}

	return false;
}

bool parse(const string &in, ErrorMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("ignore") == 0) { val = ERRORMODE_IGNORE; return true; }
		if (norm.compare("send") == 0) { val = ERRORMODE_SEND; return true; }
		if (norm.compare("sendandoff") == 0) { val = ERRORMODE_SENDANDOFF; return true; }
	}
	else
	{
		if (in.compare("IGNORE") == 0) { val = ERRORMODE_IGNORE; return true; }
		if (in.compare("SEND") == 0) { val = ERRORMODE_SEND; return true; }
		if (in.compare("SENDANDOFF") == 0) { val = ERRORMODE_SENDANDOFF; return true; }
	}

	return false;
}

bool parse(const string &in, FilterMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("nofiltering") == 0) { val = FILTERMODE_NOFILTERING; return true; }
		if (norm.compare("onlyraw") == 0) { val = FILTERMODE_ONLYRAW; return true; }
		if (norm.compare("onlycompensated") == 0) { val = FILTERMODE_ONLYCOMPENSATED; return true; }
		if (norm.compare("both") == 0) { val = FILTERMODE_BOTH; return true; }
	}
	else
	{
		if (in.compare("NOFILTERING") == 0) { val = FILTERMODE_NOFILTERING; return true; }
		if (in.compare("ONLYRAW") == 0) { val = FILTERMODE_ONLYRAW; return true; }
		if (in.compare("ONLYCOMPENSATED") == 0) { val = FILTERMODE_ONLYCOMPENSATED; return true; }
		if (in.compare("BOTH") == 0) { val = FILTERMODE_BOTH; return true; }
	}

	return false;
}

bool parse(const string &in, IntegrationFrame &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("body") == 0) { val = INTEGRATIONFRAME_BODY; return true; }
		if (norm.compare("ned") == 0) { val = INTEGRATIONFRAME_NED; return true; }
	}
	else
	{
		if (in.compare("BODY") == 0) { val = INTEGRATIONFRAME_BODY; return true; }
		if (in.compare("NED") == 0) { val = INTEGRATIONFRAME_NED; return true; }
	}

	return false;
}

bool parse(const string &in, CompensationMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("none") == 0) { val = COMPENSATIONMODE_NONE; return true; }
		if (norm.compare("bias") == 0) { val = COMPENSATIONMODE_BIAS; return true; }
	}
	else
	{
		if (in.compare("NONE") == 0) { val = COMPENSATIONMODE_NONE; return true; }
		if (in.compare("BIAS") == 0) { val = COMPENSATIONMODE_BIAS; return true; }
	}

	return false;
}

bool parse(const string &in, GpsFix &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("nofix") == 0) { val = GPSFIX_NOFIX; return true; }
		if (norm.compare("timeonly") == 0) { val = GPSFIX_TIMEONLY; return true; }
		if (norm.compare("2d") == 0) { val = GPSFIX_2D; return true; }
		if (norm.compare("3d") == 0) { val = GPSFIX_3D; return true; }
	}
	else
	{
		if (in.compare("NOFIX") == 0) { val = GPSFIX_NOFIX; return true; }
		if (in.compare("TIMEONLY") == 0) { val = GPSFIX_TIMEONLY; return true; }
		if (in.compare("2D") == 0) { val = GPSFIX_2D; return true; }
		if (in.compare("3D") == 0) { val = GPSFIX_3D; return true; }
	}

	return false;
}

bool parse(const string &in, GpsMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("onboardgps") == 0) { val = GPSMODE_ONBOARDGPS; return true; }
		if (norm.compare("externalgps") == 0) { val = GPSMODE_EXTERNALGPS; return true; }
		if (norm.compare("externalvn200gps") == 0) { val = GPSMODE_EXTERNALVN200GPS; return true; }
	}
	else
	{
		if (in.compare("ONBOARDGPS") == 0) { val = GPSMODE_ONBOARDGPS; return true; }
		if (in.compare("EXTERNALGPS") == 0) { val = GPSMODE_EXTERNALGPS; return true; }
		if (in.compare("EXTERNALVN200GPS") == 0) { val = GPSMODE_EXTERNALVN200GPS; return true; }
	}

	return false;
}

bool parse(const string &in, PpsSource &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("gpsppsrising") == 0) { val = PPSSOURCE_GPSPPSRISING; return true; }
		if (norm.compare("gpsppsfalling") == 0) { val = PPSSOURCE_GPSPPSFALLING; return true; }
		if (norm.compare("syncinrising") == 0) { val = PPSSOURCE_SYNCINRISING; return true; }
		if (norm.compare("syncinfalling") == 0) { val = PPSSOURCE_SYNCINFALLING; return true; }
	}
	else
	{
		if (in.compare("GPSPPSRISING") == 0) { val = PPSSOURCE_GPSPPSRISING; return true; }
		if (in.compare("GPSPPSFALLING") == 0) { val = PPSSOURCE_GPSPPSFALLING; return true; }
		if (in.compare("SYNCINRISING") == 0) { val = PPSSOURCE_SYNCINRISING; return true; }
		if (in.compare("SYNCINFALLING") == 0) { val = PPSSOURCE_SYNCINFALLING; return true; }
	}

	return false;
}

bool parse(const string &in, VpeEnable &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("disable") == 0) { val = VPEENABLE_DISABLE; return true; }
		if (norm.compare("enable") == 0) { val = VPEENABLE_ENABLE; return true; }
	}
	else
	{
		if (in.compare("DISABLE") == 0) { val = VPEENABLE_DISABLE; return true; }
		if (in.compare("ENABLE") == 0) { val = VPEENABLE_ENABLE; return true; }
	}

	return false;
}

bool parse(const string &in, HeadingMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("absolute") == 0) { val = HEADINGMODE_ABSOLUTE; return true; }
		if (norm.compare("relative") == 0) { val = HEADINGMODE_RELATIVE; return true; }
		if (norm.compare("indoor") == 0) { val = HEADINGMODE_INDOOR; return true; }
	}
	else
	{
		if (in.compare("ABSOLUTE") == 0) { val = HEADINGMODE_ABSOLUTE; return true; }
		if (in.compare("RELATIVE") == 0) { val = HEADINGMODE_RELATIVE; return true; }
		if (in.compare("INDOOR") == 0) { val = HEADINGMODE_INDOOR; return true; }
	}

	return false;
}

bool parse(const string &in, VpeMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("off") == 0) { val = VPEMODE_OFF; return true; }
		if (norm.compare("mode1") == 0) { val = VPEMODE_MODE1; return true; }
	}
	else
	{
		if (in.compare("OFF") == 0) { val = VPEMODE_OFF; return true; }
		if (in.compare("MODE1") == 0) { val = VPEMODE_MODE1; return true; }
	}

	return false;
}

bool parse(const string &in, Scenario &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("ahrs") == 0) { val = SCENARIO_AHRS; return true; }
		if (norm.compare("inswithpressure") == 0) { val = SCENARIO_INSWITHPRESSURE; return true; }
		if (norm.compare("inswithoutpressure") == 0) { val = SCENARIO_INSWITHOUTPRESSURE; return true; }
		if (norm.compare("gpsmovingbaselinedynamic") == 0) { val = SCENARIO_GPSMOVINGBASELINEDYNAMIC; return true; }
		if (norm.compare("gpsmovingbaselinestatic") == 0) { val = SCENARIO_GPSMOVINGBASELINESTATIC; return true; }
	}
	else
	{
		if (in.compare("AHRS") == 0) { val = SCENARIO_AHRS; return true; }
		if (in.compare("INSWITHPRESSURE") == 0) { val = SCENARIO_INSWITHPRESSURE; return true; }
		if (in.compare("INSWITHOUTPRESSURE") == 0) { val = SCENARIO_INSWITHOUTPRESSURE; return true; }
		if (in.compare("GPSMOVINGBASELINEDYNAMIC") == 0) { val = SCENARIO_GPSMOVINGBASELINEDYNAMIC; return true; }
		if (in.compare("GPSMOVINGBASELINESTATIC") == 0) { val = SCENARIO_GPSMOVINGBASELINESTATIC; return true; }
	}

	return false;
}

bool parse(const string &in, HsiMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("off") == 0) { val = HSIMODE_OFF; return true; }
		if (norm.compare("run") == 0) { val = HSIMODE_RUN; return true; }
		if (norm.compare("reset") == 0) { val = HSIMODE_RESET; return true; }
	}
	else
	{
		if (in.compare("OFF") == 0) { val = HSIMODE_OFF; return true; }
		if (in.compare("RUN") == 0) { val = HSIMODE_RUN; return true; }
		if (in.compare("RESET") == 0) { val = HSIMODE_RESET; return true; }
	}

	return false;
}

bool parse(const string &in, HsiOutput &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("noonboard") == 0) { val = HSIOUTPUT_NOONBOARD; return true; }
		if (norm.compare("useonboard") == 0) { val = HSIOUTPUT_USEONBOARD; return true; }
	}
	else
	{
		if (in.compare("NOONBOARD") == 0) { val = HSIOUTPUT_NOONBOARD; return true; }
		if (in.compare("USEONBOARD") == 0) { val = HSIOUTPUT_USEONBOARD; return true; }
	}

	return false;
}

bool parse(const string &in, VelocityCompensationMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("disabled") == 0) { val = VELOCITYCOMPENSATIONMODE_DISABLED; return true; }
		if (norm.compare("bodymeasurement") == 0) { val = VELOCITYCOMPENSATIONMODE_BODYMEASUREMENT; return true; }
	}
	else
	{
		if (in.compare("DISABLED") == 0) { val = VELOCITYCOMPENSATIONMODE_DISABLED; return true; }
		if (in.compare("BODYMEASUREMENT") == 0) { val = VELOCITYCOMPENSATIONMODE_BODYMEASUREMENT; return true; }
	}

	return false;
}

bool parse(const string &in, MagneticMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("2d") == 0) { val = MAGNETICMODE_2D; return true; }
		if (norm.compare("3d") == 0) { val = MAGNETICMODE_3D; return true; }
	}
	else
	{
		if (in.compare("2D") == 0) { val = MAGNETICMODE_2D; return true; }
		if (in.compare("3D") == 0) { val = MAGNETICMODE_3D; return true; }
	}

	return false;
}

bool parse(const string &in, ExternalSensorMode &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("internal") == 0) { val = EXTERNALSENSORMODE_INTERNAL; return true; }
		if (norm.compare("external200hz") == 0) { val = EXTERNALSENSORMODE_EXTERNAL200HZ; return true; }
		if (norm.compare("externalonupdate") == 0) { val = EXTERNALSENSORMODE_EXTERNALONUPDATE; return true; }
	}
	else
	{
		if (in.compare("INTERNAL") == 0) { val = EXTERNALSENSORMODE_INTERNAL; return true; }
		if (in.compare("EXTERNAL200HZ") == 0) { val = EXTERNALSENSORMODE_EXTERNAL200HZ; return true; }
		if (in.compare("EXTERNALONUPDATE") == 0) { val = EXTERNALSENSORMODE_EXTERNALONUPDATE; return true; }
	}

	return false;
}

bool parse(const string &in, FoamInit &val, bool allowSloppy)
{
	if (allowSloppy)
	{
		// Adjust for user input slop.
		string norm = in;
		norm.erase(remove_if(norm.begin(), norm.end(), ::isspace), norm.end());
		transform(norm.begin(), norm.end(), norm.begin(), ::tolower);

		if (norm.compare("nofoaminit") == 0) { val = FOAMINIT_NOFOAMINIT; return true; }
		if (norm.compare("foaminitpitchroll") == 0) { val = FOAMINIT_FOAMINITPITCHROLL; return true; }
		if (norm.compare("foaminitheadingpitchroll") == 0) { val = FOAMINIT_FOAMINITHEADINGPITCHROLL; return true; }
		if (norm.compare("foaminitpitchrollcovariance") == 0) { val = FOAMINIT_FOAMINITPITCHROLLCOVARIANCE; return true; }
		if (norm.compare("foaminitheadingpitchrollcovariance") == 0) { val = FOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE; return true; }
	}
	else
	{
		if (in.compare("NOFOAMINIT") == 0) { val = FOAMINIT_NOFOAMINIT; return true; }
		if (in.compare("FOAMINITPITCHROLL") == 0) { val = FOAMINIT_FOAMINITPITCHROLL; return true; }
		if (in.compare("FOAMINITHEADINGPITCHROLL") == 0) { val = FOAMINIT_FOAMINITHEADINGPITCHROLL; return true; }
		if (in.compare("FOAMINITPITCHROLLCOVARIANCE") == 0) { val = FOAMINIT_FOAMINITPITCHROLLCOVARIANCE; return true; }
		if (in.compare("FOAMINITHEADINGPITCHROLLCOVARIANCE") == 0) { val = FOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE; return true; }
	}

	return false;
}

}
