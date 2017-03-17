#include "vn/sensorfeatures.h"

using namespace std;
using namespace vn::protocol::uart;

namespace vn {
namespace sensors {

VnSensorFeatures::VnSensorFeatures() :
  _family(VnSensor::VnSensor_Family_Unknown),
  _ver("")
{ }

VnSensorFeatures::VnSensorFeatures(string modelNumber, string firmwareVersion, uint32_t hardwareRevision) :
  _family(VnSensor::determineDeviceFamily(modelNumber)),
  _ver(firmwareVersion),
  _rev(hardwareRevision)
{ }

vector<AsciiAsync> VnSensorFeatures::supportedAsyncDataOutputTypes()
{
  if (_family == VnSensor::VnSensor_Family_Vn100)
  {
    if (_ver < Version("2.0"))
    {
      AsciiAsync v[] = {
          VNOFF,
          VNYPR,
          VNQTN,
          #ifdef INTERNAL
          VNQTM,
          VNQTA,
          VNQTR,
          VNQMA,
          VNQAR,
          #endif
          VNQMR,
          #ifdef INTERNAL
          VNDCM,
          #endif
          VNMAG,
          VNACC,
          VNGYR,
          VNMAR,
          VNYMR,
          #ifdef INTERNAL
          VNYCM,
          #endif
          VNYBA,
          VNYIA
          #ifdef INTERNAL
          ,
          VNICM,
          VNRAW,
          VNCMV,
          VNSTV,
          VNCOV
          #endif
      };

      return vector<AsciiAsync>(&v[0], &v[0] + sizeof(v) / sizeof(AsciiAsync));
    }
    else
    {
      AsciiAsync v[] = {
          VNOFF,
          VNYPR,
          VNQTN,
          VNQMR,
          VNMAG,
          VNACC,
          VNGYR,
          VNMAR,
          VNYMR,
          VNYBA,
          VNYIA,
          VNIMU,
          VNDTV
      };

      return vector<AsciiAsync>(&v[0], &v[0] + sizeof(v) / sizeof(AsciiAsync));
    }
  }
  else if (_family == VnSensor::VnSensor_Family_Vn200)
  {
    if (_ver < Version("1.0"))
    {
      AsciiAsync v[] = {
          VNOFF,
          VNIMU,
          VNGPS,
          VNGPE,
          VNINS,
          VNISL,
      };

      return vector<AsciiAsync>(&v[0], &v[0] + sizeof(v) / sizeof(AsciiAsync));
    }
    else
    {
      AsciiAsync v[] = {
          VNOFF,
          VNYPR,
          VNQTN,
          VNQMR,
          VNMAG,
          VNACC,
          VNGYR,
          VNMAR,
          VNYMR,
          VNYBA,
          VNYIA,
          VNIMU,
          VNGPS,
          VNGPE,
          VNINS,
          VNINE,
          VNISL,
          VNISE,
          VNDTV
      };

      return vector<AsciiAsync>(&v[0], &v[0] + sizeof(v) / sizeof(AsciiAsync));
    }
  }
  else if (_family == VnSensor::VnSensor_Family_Vn300)
  {
    AsciiAsync v[] = {
        VNOFF,
        VNYPR,
        VNQTN,
        VNQMR,
        VNMAG,
        VNACC,
        VNGYR,
        VNMAR,
        VNYMR,
        VNYBA,
        VNYIA,
        VNIMU,
        VNGPS,
        VNGPE,
        VNINS,
        VNINE,
        VNISL,
        VNISE,
        VNDTV
    };

    return vector<AsciiAsync>(&v[0], &v[0] + sizeof(v) / sizeof(AsciiAsync));
  }

  throw new vn::invalid_operation("unknown device family");
}

AsciiAsync VnSensorFeatures::defaultAsyncDataOutputType()
{
  if (_family == VnSensor::VnSensor_Family_Vn100)
  {
    if (_ver <= Version("1.0"))
      return VNYPR;

    return VNYMR;
  }
  else if (_family == VnSensor::VnSensor_Family_Vn200)
  {
    return VNINS;
  }
  else if (_family == VnSensor::VnSensor_Family_Vn300)
  {
    return VNINS;
  }

  throw new vn::invalid_operation("unknown device family");
}

bool VnSensorFeatures::supportsUserTag()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver < Version("1.1"))
    return false;

  return true;
}

bool VnSensorFeatures::supportsDualSerialOutputs()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _rev < 6)
    return false;

  return true;
}

bool VnSensorFeatures::supportsMagneticAndGravityReferenceVectors()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsFilterMeasurementsVarianceParameters()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("0.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsFilterActiveTuningParameters()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("0.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1.7"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsMagnetometerCompensation()
{
  if (_family == VnSensor::VnSensor_Family_Vn100)
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1.7"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsAccelerationCompensation()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsGyroCompensation()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1.144.4"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsCommunicationProtocolControl()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200)
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsSynchronizationControl()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1.7"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsFilterBasicControl()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1.7"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsVpeBasicControl()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver > Version("1.0.0.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsVpeMagnetometerBasicTuning()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver > Version("1.0.0.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsVpeMagnetometerAdvancedTuning()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1.7"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsVpeAccelerometerBasicTuning()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver > Version("1.0.0.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsVpeAccelerometerAdvancedTuning()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsVpeGyroBasicTuning()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1.7"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsFilterStartupGyroBias()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.1.7"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsMagnetometerCalibrationControl()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver > Version("1.0.0.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsIndoorHeadingModeControl()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsGpsConfiguration()
{
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsGpsAntennaOffset()
{
  if (_family == VnSensor::VnSensor_Family_Vn200)
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsGpsCompassBaseline()
{
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsBinaryOutput()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("2.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsImuFilteringConfiguration()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver > Version("1.1.144.4"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsDeltaThetaAndDeltaVelocityConfiguration()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver > Version("1.1.144.4"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsStartupFilterBiasEstimate()
{
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("0.2"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsInsBasicConfiguration()
{
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsVelocityCompensationControl()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("1.1.140.4"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsReferenceVectorConfiguration()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver > Version("1.1.144.4"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn300)
    return true;

  return false;
}

bool VnSensorFeatures::supportsImuRateConfiguration()
{
  if (_family == VnSensor::VnSensor_Family_Vn100 && _ver >= Version("2.0"))
    return true;
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0"))
    return true;

  return false;
}

bool VnSensorFeatures::supportsInsAdvancedConfiguration()
{
  if (_family == VnSensor::VnSensor_Family_Vn200 && _ver >= Version("1.0.1.0"))
    return true;

  return false;
}


}
}