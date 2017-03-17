#ifndef __VNCONV_H__
#define __VNCONV_H__

#include "vnvector.h"

/** Converts ECEF coordinate to LLA frame.
 * \param[in] ecef Coordinate in ECEF frame.
 * \return Coordinate converted to LLA frame. */
union vn_vec3d ecef_to_lla_v3d(union vn_vec3d ecef);

/** Converts LLA coordinate to ECEF frame.
 * \param[in] lla Coordinate in LLA frame in (deg, deg, meter) units.
 * \return Coordinate converted to ECEF frame in (km, km, km) units. */
union vn_vec3d lla_to_ecef_v3d(union vn_vec3d lla);

#endif
