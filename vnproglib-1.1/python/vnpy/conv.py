import vnpy
from . import vncxxlib as lib
from .vncxxlib import (celsius2fahren, fahren2celsius, celsius2kelvin, kelvin2celsius, fahren2kelvin, kelvin2fahren,
                       ypr_degs2quat, ypr_rads2quat, ypr_degs2dcm, ypr_rads2dcm, quat2ypr_degs, quat2ypr_rads,
                       quat2dcm, dcm2ypr_degs, dcm2ypr_rads, dcm2quat, velocity_ned_xy2course_over_ground,
                       velocity_ned_xy2speed_over_ground, quat2omega_phi_kappa_rads, dcm2omega_phi_kappa_rads,
                       ypr_degs2omega_phi_kappa_rads, ypr_rads2omega_phi_kappa_rads)


def rad2deg(val):
    if type(val) is vnpy.vec3f:
        return lib.__rad2deg_v3f(val)
    elif type(val) is vnpy.vec3d:
        return lib.__rad2deg_v3d(val)
    elif type(val) is int or type(val) is float:
        return lib.__rad2deg_d(val)

    raise RuntimeError('unknown type')


def deg2rad(val):
    if type(val) is vnpy.vec3f:
        return lib.__deg2rad_v3f(val)
    elif type(val) is vnpy.vec3d:
        return lib.__deg2rad_v3d(val)
    elif type(val) is int or type(val) is float:
        return lib.__deg2rad_d(val)

    raise RuntimeError('unknown type')
