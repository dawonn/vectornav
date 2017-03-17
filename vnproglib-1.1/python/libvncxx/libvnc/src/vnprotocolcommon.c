#include "vnprotocolcommon.h"
#include "vnstring.h"

/**\defgroup VnAsciiAsyncDescriptions ASCII Async Output Descriptions
 * \{ */
const char *VNOFF_DESC = "Off";
const char *VNYPR_DESC = "Yaw, Pitch, Roll";
const char *VNQTN_DESC = "Quaterion";
const char *VNQMR_DESC = "Quaternion, Magnetic, Acceleration and Angular Rates";
const char *VNMAG_DESC = "Magnetic Measurements";
const char *VNACC_DESC = "Acceleration Measurements";
const char *VNGYR_DESC = "Angular Rate Measurements";
const char *VNMAR_DESC = "Magnetic, Acceleration, and Angular Rate Measurements";
const char *VNYMR_DESC = "Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements";
const char *VNYBA_DESC = "Yaw, Pitch, Roll, Body True Acceleration";
const char *VNYIA_DESC = "Yaw, Pitch, Roll, Inertial True Acceleration";
const char *VNIMU_DESC = "Calibrated Inertial Measurements";
const char *VNGPS_DESC = "GPS LLA";
const char *VNGPE_DESC = "GPS ECEF";
const char *VNINS_DESC = "INS LLA solution";
const char *VNINE_DESC = "INS ECEF solution";
const char *VNISL_DESC = "INS LLA 2 solution";
const char *VNISE_DESC = "INS ECEF 2 solution";
const char *VNDTV_DESC = "Delta Theta and Delta Velocity";
#ifdef VN_EXTRA
const char *VNQTM_DESC = "Quaternion and Magnetic";
const char *VNQTA_DESC = "Quaternion and Acceleration";
const char *VNQTR_DESC = "Quaternion and Angular Rates";
const char *VNQMA_DESC = "Quaternion, Magnetic and Acceleration";
const char *VNQAR_DESC = "Quaternion, Acceleration and Angular Rates";
const char *VNDCM_DESC = "Directional Cosine Orientation Matrix";
const char *VNYCM_DESC = "Yaw, Pitch, Roll, and Calibrated Measurements";
const char *VNICM_DESC = "Yaw, Pitch, Roll, Inertial Magnetic/Acceleration, and Angular Rate Measurements";
const char *VNRAW_DESC = "Raw Voltage Measurements";
const char *VNCMV_DESC = "Calibrated Measurements";
const char *VNSTV_DESC = "Kalman Filter State Vector";
const char *VNCOV_DESC = "Kalman Filter Covariance Matrix Diagonal";
#endif
/** \} */

enum VnError
to_string_VnAsciiAsync(
    char* out,
    size_t outSize,
    enum VnAsciiAsync v)
{
  const char *desc = NULL;

  switch (v)
  {
    case VNOFF:
      desc = "VNOFF"; break;
    case VNYPR:
      desc = "VNYPR"; break;
    case VNQTN:
      desc = "VNQTN"; break;
    #ifdef VN_EXTRA
    case VNQTM:
      desc = "VNQTM"; break;
    case VNQTA:
      desc = "VNQTA"; break;
    case VNQTR:
      desc = "VNQTR"; break;
    case VNQMA:
      desc = "VNQMA"; break;
    case VNQAR:
      desc = "VNQAR"; break;
    #endif
    case VNQMR:
      desc = "VNQMR"; break;
    #ifdef VN_EXTRA
    case VNDCM:
      desc = "VNDCM"; break;
    #endif
    case VNMAG:
      desc = "VNMAG"; break;
    case VNACC:
      desc = "VNACC"; break;
    case VNGYR:
      desc = "VNGYR"; break;
    case VNMAR:
      desc = "VNMAR"; break;
    case VNYMR:
      desc = "VNYMR"; break;
    #ifdef VN_EXTRA
    case VNYCM:
      desc = "VNYCM"; break;
    #endif
    case VNYBA:
      desc = "VNYBA"; break;
    case VNYIA:
      desc = "VNYIA"; break;
    #ifdef VN_EXTRA
    case VNICM:
      desc = "VNICM"; break;
    #endif
    case VNIMU:
      desc = "VNIMU"; break;
    case VNGPS:
      desc = "VNGPS"; break;
    case VNGPE:
      desc = "VNGPE"; break;
    case VNINS:
      desc = "VNINS"; break;
    case VNINE:
      desc = "VNINE"; break;
    case VNISL:
      desc = "VNISL"; break;
    case VNISE:
      desc = "VNISE"; break;
    case VNDTV:
      desc = "VNDTV"; break;
    #ifdef VN_EXTRA
    case VNRAW:
      desc = "VNRAW"; break;
    case VNCMV:
      desc = "VNCMV"; break;
    case VNSTV:
      desc = "VNSTV"; break;
    case VNCOV:
      desc = "VNCOV"; break;
    #endif
    default:
      return E_INVALID_VALUE;
  }

  return strcpy_x(out, outSize, desc);
}

enum VnError
description_VnAsciiAsync(
    char *out,
    size_t outSize,
    enum VnAsciiAsync value)
{
  const char *desc = NULL;

  switch (value)
  {
    case VNOFF:
      desc = VNOFF_DESC; break;
    case VNYPR:
      desc = VNYPR_DESC; break;
    case VNQTN:
      desc = VNQTN_DESC; break;
    #ifdef VN_EXTRA
    case VNQTM:
      desc = VNQTM_DESC; break;
    case VNQTA:
      desc = VNQTA_DESC; break;
    case VNQTR:
      desc = VNQTR_DESC; break;
    case VNQMA:
      desc = VNQMA_DESC; break;
    case VNQAR:
      desc = VNQAR_DESC; break;
    #endif
    case VNQMR:
      desc = VNQMR_DESC; break;
    #ifdef VN_EXTRA
    case VNDCM:
      desc = VNDCM_DESC; break;
    #endif
    case VNMAG:
      desc = VNMAG_DESC; break;
    case VNACC:
      desc = VNACC_DESC; break;
    case VNGYR:
      desc = VNGYR_DESC; break;
    case VNMAR:
      desc = VNMAR_DESC; break;
    case VNYMR:
      desc = VNYMR_DESC; break;
    #ifdef VN_EXTRA
    case VNYCM:
      desc = VNYCM_DESC; break;
    #endif
    case VNYBA:
      desc = VNYBA_DESC; break;
    case VNYIA:
      desc = VNYIA_DESC; break;
    #ifdef VN_EXTRA
    case VNICM:
      desc = VNICM_DESC; break;
    #endif
    case VNIMU:
      desc = VNIMU_DESC; break;
    case VNGPS:
      desc = VNGPS_DESC; break;
    case VNGPE:
      desc = VNGPE_DESC; break;
    case VNINS:
      desc = VNINS_DESC; break;
    case VNINE:
      desc = VNINE_DESC; break;
    case VNISL:
      desc = VNISL_DESC; break;
    case VNISE:
      desc = VNISE_DESC; break;
    case VNDTV:
      desc = VNDTV_DESC; break;
    #ifdef VN_EXTRA
    case VNRAW:
      desc = VNRAW_DESC; break;
    case VNCMV:
      desc = VNCMV_DESC; break;
    case VNSTV:
      desc = VNSTV_DESC; break;
    case VNCOV:
      desc = VNCOV_DESC; break;
    #endif
    default:
      return E_INVALID_VALUE;
  }

  return strcpy_x(out, outSize, desc);
}
