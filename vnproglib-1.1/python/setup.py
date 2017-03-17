#!/usr/bin/env python

import os
from distutils.core import setup, Extension

additional_libs = None
if os.name == 'nt':
    additional_libs = [ 'Advapi32' ]


module_raw = Extension(
    'vnpy._libvncxx',
    include_dirs = [ 'libvncxx/include', 'libvncxx/libvnc/include' ],
    swig_opts = [ '-c++' ],
    libraries = additional_libs,
    sources = [
        'vnpy/libvncxx_wrap.cpp',
        'libvncxx/src/attitude.cpp',
        'libvncxx/src/compositedata.cpp',
        'libvncxx/src/conversions.cpp',
        'libvncxx/src/criticalsection.cpp',
        'libvncxx/src/error_detection.cpp',
        'libvncxx/src/event.cpp',
        'libvncxx/src/ezasyncdata.cpp',
        'libvncxx/src/packet.cpp',
        'libvncxx/src/packetfinder.cpp',
        'libvncxx/src/port.cpp',
        'libvncxx/src/position.cpp',
        'libvncxx/src/registers.cpp',
        'libvncxx/src/searcher.cpp',
        'libvncxx/src/sensorconfig.cpp',
        'libvncxx/src/sensorfeatures.cpp',
        'libvncxx/src/sensors.cpp',
        'libvncxx/src/serialport.cpp',
        'libvncxx/src/thread.cpp',
        'libvncxx/src/types.cpp',
        'libvncxx/src/util.cpp',
        'libvncxx/src/utilities.cpp',
        'libvncxx/src/version.cpp',
        'libvncxx/src/vntime.cpp',
        'libvncxx/src/devices/DeviceError.cpp',
        'libvncxx/libvnc/src/vncompositedata.c',
        'libvncxx/libvnc/src/vnconv.c',
        'libvncxx/libvnc/src/vncriticalsection.c',
        'libvncxx/libvnc/src/vnerrdet.c',
        'libvncxx/libvnc/src/vnerror.c',
        'libvncxx/libvnc/src/vnevent.c',
        'libvncxx/libvnc/src/vnezasyncdata.c',
        'libvncxx/libvnc/src/vnmatrix.c',
        'libvncxx/libvnc/src/vnsearcher.c',
        'libvncxx/libvnc/src/vnsensor.c',
        'libvncxx/libvnc/src/vnserialport.c',
        'libvncxx/libvnc/src/vnspi.c',
        'libvncxx/libvnc/src/vnstring.c',
        'libvncxx/libvnc/src/vnthread.c',
        'libvncxx/libvnc/src/vntime.c',
        'libvncxx/libvnc/src/vnupack.c',
        'libvncxx/libvnc/src/vnupackf.c',
        'libvncxx/libvnc/src/vnutil.c',
        'libvncxx/libvnc/src/vnvector.c'
    ])

setup(
    name='vnpy',
    version='1.0',
    description='VectorNav Python Library',
    author='VectorNav Technologies, LLC',
    packages=['vnpy'],
    ext_modules = [ module_raw ])
