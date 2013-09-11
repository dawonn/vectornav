/**
 * \file
 *
 * \section LICENSE
 * MIT License (MIT)
 *
 * Copyright (c) 2011 VectorNav Technologies, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 * This header file provides access to devices based on VectorNav's VN-200
 * family of orientation sensors.
 */
#ifndef _VN200_H_
#define _VN200_H_

#include "vncp_services.h"
#include "vn_kinematics.h"
#include "vn_linearAlgebra.h"

#if EXPORT_TO_DLL
	#define DLL_EXPORT __declspec(dllexport)
#else
	#define DLL_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define VNASYNC_OFF		0		/**< Asynchronous output is turned off. */
#define VNASYNC_VNIMU	19		/**< Asynchronous output type is Calibrated Interial Measurements. */
#define VNASYNC_VNGPS	20		/**< Asynchronous output type is GPS Measurements. */
#define VNASYNC_VNINS	22		/**< Asynchronous output type is INS Solution. */

/**
 * \brief Holds connection information for accessing a VN-200 device.
 */
typedef struct {
	char*	portName;		/**< The name of the serial port. */
	int		baudRate;		/**< The baudrate of the serial port. */
	VN_BOOL	isConnected;	/**< Inidicates if the serial port is open. */
	void*	internalData;	/**< Pointer to data used internally by the Vn200 function handlers. */
} Vn200;

/**
 * \brief Composite structure of the various asynchronous data the VN-200 device
 * is capable of outputting.
 */
typedef struct {
	VnYpr			ypr;						/**< Yaw, pitch, roll. */
	VnQuaternion	quaternion;					/**< Quaternion. */
	VnVector3		magnetic;					/**< Magnetic measurements. */
	VnVector3		acceleration;				/**< Acceleration measurements. */
	VnVector3		angularRate;				/**< Angular rate / gyro measurements. */
	VnMatrix3x3		dcm;						/**< Direction cosine matrix. */
	double			temperature;				/**< Temperature. */
	VnVector3		magneticVoltage;			/**< Magnetic sensor voltages. */
	VnVector3		accelerationVoltage;		/**< Accleration sensor voltages. */
	VnVector3		angularRateVoltage;			/**< Angular rate sensor voltages. */
	double			temperatureVoltage;			/**< Temperatue sensor voltages. */
	VnVector3		angularRateBias;			/**< Angular rate estimated biases. */
	VnVector3		attitudeVariance;			/**< Variance for the computed attitude. */
	VnVector3		angularRateBiasVariance;	/**< Angular rate bias variance. */
	double			pressure;					/**< Pressure. */
	double			gpsTimeOfWeek;				/**< GPS time of week in seconds. */
	unsigned short	gpsWeek;					/**< GPS week. */
	unsigned char	gpsFix;						/**< Gps fix type. */
	unsigned char	numberOfSatellites;			/**< Number of GPS satellites used in solution. */
	VnVector3		latitudeLongitudeAltitude;	/**< Latitude, longitude and altitude. */
	VnVector3		velocity;					/**< Velocity measurements. */
	VnVector3		positionAccuracy;			/**< Position accuracy. */
	float			speedAccuracy;				/**< Velocity/speed accuracy estimate. */
	float			timeAccuracy;				/**< Time accuracy estimate. */
	unsigned short	insStatus;					/**< Status flags for the INS filter. */
	float			attitudeUncertainty;		/**< Uncertainty in attitude estimate. */
	float			positionUncertainty;		/**< Uncertainty in position estimate. */
	float			velocityUncertainty;		/**< Uncertainty in velocity estimate. */
} Vn200CompositeData;

/**
 * \brief Fuction type used for receiving notifications of when new asynchronous
 * data is received.
 *
 * \param[in]	sender	The device that sent the notification.
 * \param[in]	newData	Pointer to the new data.
 */
typedef void (*Vn200NewAsyncDataReceivedListener)(Vn200* sender, Vn200CompositeData* newData);

/**
 * \brief Connects to a VectorNav VN-200 device.
 *
 * \param[out]	newVn200	An uninitialized Vn200 control object should be passed in.
 * \param[in]	portName	The name of the COM port to connect to.
 * \param[in]	baudrate	The baudrate to connect at.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_connect(Vn200* newVn200, const char* portName, int baudrate);

/**
 * \brief Disconnects from the VN-200 device and disposes of any internal resources.
 *
 * \param vn200 Pointer to the Vn200 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_disconnect(Vn200* vn200);

/**
 * \brief Computes the checksum for the provided command.
 *
 * \param[in]	cmdToCheck
 * Null-terminated string of the form "VNRRG,1".
 *
 * \return The computed checksum number.
 */
DLL_EXPORT unsigned char vn200_checksum_compute(const char* cmdToCheck);

/**
 * \brief Computes the checksum for the provided command and returns it as a
 * two character string representing it in hexidecimal.
 *
 * \param[in]	cmdToCheck
 * Null-terminated string of the form "VNRRG,1".
 *
 * \param[out]	checksum
 * A character array of length 2 which the computed checksum will be placed.
 */
DLL_EXPORT void vn200_checksum_computeAndReturnAsHex(const char* cmdToCheck, char* checksum);

/**
 * \brief Sets the timeout value for the reading values from the VN-200 sensor.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] timeout The timeout value in milliseconds. Specify -1 to not use
 * any timeouts.
 *
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_set_timeout(Vn200* vn200, int timeout);

/**
 * \brief Retrieves the associated timeout value for the Vn200 object.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 *
 * \return The timeout value in milliseconds. -1 indicates that timeouts are
 * not used.
 */
DLL_EXPORT int vn200_get_timeout(Vn200* vn200);

/**
 * \brief Retreives the most recently received asynchronous measurements
 * received from the VN-200 device.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out]	curData
 * Composite of the last currently received data from the device. If there was
 * no data received for certain fields, they will be zeroed out.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getCurrentAsyncData(Vn200* vn200, Vn200CompositeData* curData);

/**
 * \brief Commands the VN-200 unit to write its current register setting to
 * non-volatile memory.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_writeSettings(Vn200* vn200, VN_BOOL waitForResponse);

/**
 * \brief Commands the VN-200 unit to revert its settings to factory defaults.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_restoreFactorySettings(Vn200* vn200, VN_BOOL waitForResponse);

/**
 * \brief Commands the VN-200 module to reset itself.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_reset(Vn200* vn200);

/**
 * \brief Allows registering a function which will be called whenever a new
 * asynchronous data packet is received from the VN-200 module.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * 
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_registerAsyncDataReceivedListener(Vn200* vn200, Vn200NewAsyncDataReceivedListener listener);

/**
 * \brief Unregisters an already registered function for recieving notifications
 * of when new asynchronous data is received.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * 
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_unregisterAsyncDataReceivedListener(Vn200* vn200, Vn200NewAsyncDataReceivedListener listener);

/**
 * \brief Checks if we are able to send and receive communication with the VN-200 sensor.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 *
 * \return VN_TRUE if the library was able to send and receive a valid response from the VN-200 sensor; otherwise VN_FALSE.
 */
DLL_EXPORT VN_BOOL vn200_verifyConnectivity(Vn200* vn200);


/**
 * \brief Gets the values in the User Tag register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] userTagBuffer Buffer to store the response. Must have a length of at least 21 characters.
 * \param[in] userTagBufferLength Length of the provided userTagBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getUserTag(Vn200* vn200, char* userTagBuffer, unsigned int userTagBufferLength);

/**
 * \brief Sets the values of the User Tag register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] userTagData Array containg the data to send. Length must be equal to or less than 20 characters.
 * \param[in] userTagDataLength Length of the data to send in the userTagData array.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setUserTag(Vn200* vn200, char* userTagData, unsigned int userTagDataLength, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Model Number register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] modelBuffer Buffer to store the response. Must have a length of at least 25 characters.
 * \param[in] modelBufferLength Length of the provided modelBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getModelNumber(Vn200* vn200, char* modelBuffer, unsigned int modelBufferLength);

/**
 * \brief Gets the values in the Hardware Revision register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] hardwareRevision The hardware revision value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getHardwareRevision(Vn200* vn200, int* hardwareRevision);

/**
 * \brief Gets the values in the Serial Number register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] serialNumberBuffer Buffer to store the response. Must have a length of at least 13 characters.
 * \param[in] serialNumberBufferLength Length of the provided serialNumberBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getSerialNumber(Vn200* vn200, char* serialNumberBuffer, unsigned int serialNumberBufferLength);

/**
 * \brief Gets the values in the Firmware Version register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] firmwareVersionBuffer Buffer to store the response. Must have a length of at least 16 characters.
 * \param[in] firmwareVersionBufferLength Length of the provided firmwareVersionBuffer buffer.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getFirmwareVersion(Vn200* vn200, char* firmwareVersionBuffer, unsigned int firmwareVersionBufferLength);

/**
 * \brief Gets the values in the Serial Baud Rate register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] serialBaudrate The serial baudrate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getSerialBaudRate(Vn200* vn200, unsigned int* serialBaudrate);

/**
 * \brief Sets the values of the Serial Baud Rate register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] serialBaudrate Value for the serial baudrate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setSerialBaudRate(Vn200* vn200, unsigned int serialBaudrate, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Type register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] asyncDataOutputType The asynchronous data output type value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getAsynchronousDataOutputType(Vn200* vn200, unsigned int* asyncDataOutputType);

/**
 * \brief Sets the values of the Asynchronous Data Output Type register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] asyncDataOutputType Value for the asynchronous data output type field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setAsynchronousDataOutputType(Vn200* vn200, unsigned int asyncDataOutputType, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Asynchronous Data Output Frequency register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] asyncDataOutputFrequency The asynchronous data output frequency value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getAsynchronousDataOutputFrequency(Vn200* vn200, unsigned int* asyncDataOutputFrequency);

/**
 * \brief Sets the values of the Asynchronous Data Output Frequency register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] asyncDataOutputFrequency Value for the asynchronous data output frequency field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setAsynchronousDataOutputFrequency(Vn200* vn200, unsigned int asyncDataOutputFrequency, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] magneticReference The current sensor magnetic reference vector (X,Y,Z) values.
 * \param[out] gravityReference The current sensor gravity reference vector (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getMagneticGravityReferenceVectors(Vn200* vn200, VnVector3* magneticReference, VnVector3* gravityReference);

/**
 * \brief Sets the values of the Magnetic and Gravity Reference Vectors register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] magneticReference The magnetic reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] gravityReference The gravity reference vector (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setMagneticGravityReferenceVectors(Vn200* vn200, VnVector3 magneticReference, VnVector3 gravityReference, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Reference Frame Rotation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] c The current sensor C matrix values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getReferenceFrameRotation(Vn200* vn200, VnMatrix3x3* c);

/**
 * \brief Sets the values of the Reference Frame Rotation register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] c The C matrix values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setReferenceFrameRotation(Vn200* vn200, VnMatrix3x3 c, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Communication Protocol Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] serialCount The serial count value of the sensor.
 * \param[out] serialStatus The serial status value of the sensor.
 * \param[out] spiCount The SPI count value of the sensor.
 * \param[out] spiStatus The SPI status value of the sensor.
 * \param[out] serialChecksum The serial checksum value of the sensor.
 * \param[out] spiChecksum The SPI checksum value of the sensor.
 * \param[out] errorMode The error mode value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getCommunicationProtocolControl(Vn200* vn200, unsigned char* serialCount, unsigned char* serialStatus, unsigned char* spiCount, unsigned char* spiStatus, unsigned char* serialChecksum, unsigned char* spiChecksum, unsigned char* errorMode);

/**
 * \brief Sets the values of the Communication Protocol Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] serialCount Value for the serial count field.
 * \param[in] serialStatus Value for the serial status field.
 * \param[in] spiCount Value for the SPI count field.
 * \param[in] spiStatus Value for the SPI status field.
 * \param[in] serialChecksum Value for the serial checksum field.
 * \param[in] spiChecksum Value for the SPI checksum field.
 * \param[in] errorMode Value for the error mode field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setCommunicationProtocolControl(Vn200* vn200, unsigned char serialCount, unsigned char serialStatus, unsigned char spiCount, unsigned char spiStatus, unsigned char serialChecksum, unsigned char spiChecksum, unsigned char errorMode, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Synchronization Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] syncInMode The input signal synchronization mode value of the sensor.
 * \param[out] syncInEdge The input signal synchronization edge selection value of the sensor.
 * \param[out] syncInSkipFactor The input signal trigger skip factor value of the sensor.
 * \param[out] reserved0 The reserved value of the sensor.
 * \param[out] syncOutMode The output signal synchronization mode value of the sensor.
 * \param[out] syncOutPolarity The output signal synchronization polarity value of the sensor.
 * \param[out] syncOutSkipFactor The output synchronization signal skip factor value of the sensor.
 * \param[out] syncOutPulseWidth The output synchronization signal pulse width value of the sensor.
 * \param[out] reserved1 The reserved value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getSynchronizationControl(Vn200* vn200, unsigned char* syncInMode, unsigned char* syncInEdge, unsigned short* syncInSkipFactor, unsigned int* reserved0, unsigned char* syncOutMode, unsigned char* syncOutPolarity, unsigned short* syncOutSkipFactor, unsigned int* syncOutPulseWidth, unsigned int* reserved1);

/**
 * \brief Sets the values of the Synchronization Control register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] syncInMode Value for the input signal synchronization mode field.
 * \param[in] syncInEdge Value for the input signal synchronization edge selection field.
 * \param[in] syncInSkipFactor Value for the input signal trigger skip factor field.
 * \param[in] reserved0 Value for the reserved field.
 * \param[in] syncOutMode Value for the output signal synchronization mode field.
 * \param[in] syncOutPolarity Value for the output signal synchronization polarity field.
 * \param[in] syncOutSkipFactor Value for the output synchronization signal skip factor field.
 * \param[in] syncOutPulseWidth Value for the output synchronization signal pulse width field.
 * \param[in] reserved1 Value for the reserved field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setSynchronizationControl(Vn200* vn200, unsigned char syncInMode, unsigned char syncInEdge, unsigned short syncInSkipFactor, unsigned int reserved0, unsigned char syncOutMode, unsigned char syncOutPolarity, unsigned short syncOutSkipFactor, unsigned int syncOutPulseWidth, unsigned int reserved1, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the Calibrated Sensor Measurements register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] magnetic The current sensor magnetic (X,Y,Z) values.
 * \param[out] acceleration The current sensor acceleration (X,Y,Z) values.
 * \param[out] angularRate The current sensor angular rate (X,Y,Z) values.
 * \param[out] temperature The temperature value of the sensor.
 * \param[out] pressure The pressure value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getCalibratedSensorMeasurements(Vn200* vn200, VnVector3* magnetic, VnVector3* acceleration, VnVector3* angularRate, float* temperature, float* pressure);

/**
 * \brief Gets the values in the GPS Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] mode The mode value of the sensor.
 * \param[out] nmeaSerial1 The NMEA_Serial1 value of the sensor.
 * \param[out] nmeaSerial2 The NMEA_Serial2 value of the sensor.
 * \param[out] nmeaRate The NMEA_Rate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsConfiguration(Vn200* vn200, unsigned char* mode, unsigned char* nmeaSerial1, unsigned char* nmeaSerial2, unsigned char* nmeaRate);

/**
 * \brief Sets the values of the GPS Configuration register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] mode Value for the mode field.
 * \param[in] nmeaSerial1 Value for the NMEA_Serial1 field.
 * \param[in] nmeaSerial2 Value for the NMEA_Serial2 field.
 * \param[in] nmeaRate Value for the NMEA_Rate field.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setGpsConfiguration(Vn200* vn200, unsigned char mode, unsigned char nmeaSerial1, unsigned char nmeaSerial2, unsigned char nmeaRate, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the GPS Antenna Offset register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] position The current sensor relative postion of GPS antenna (X,Y,Z) values.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsAntennaOffset(Vn200* vn200, VnVector3* position);

/**
 * \brief Sets the values of the GPS Antenna Offset register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[in] position The relative postion of GPS antenna (X,Y,Z) values to write to the sensor.
 * \param[in] waitForResponse Signals if the function should block until a response is
       received from the sensor. TRUE to block for a response; FALSE to immediately
       return after sending out the command.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_setGpsAntennaOffset(Vn200* vn200, VnVector3 position, VN_BOOL waitForResponse);

/**
 * \brief Gets the values in the GPS Solution register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] gpsTime The GPS time of week in seconds value of the sensor.
 * \param[out] gpsWeek The GPS week value of the sensor.
 * \param[out] gpsFix The GPS fix type value of the sensor.
 * \param[out] numberOfSatellites The number of GPS satellites used in solution value of the sensor.
 * \param[out] lattitudeLongitudeAltitude The current sensor latitude, longitude, and altitude values.
 * \param[out] nedVelocity The current sensor velocity measurements (X,Y,Z) in north, east, down directions values.
 * \param[out] positionAccuracy The current sensor position accuracy (X,Y,Z) values.
 * \param[out] speedAccuracy The speed accuracy estimate value of the sensor.
 * \param[out] timeAccuracy The time accuracy estimate value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getGpsSolution(Vn200* vn200, double* gpsTime, unsigned short* gpsWeek, unsigned char* gpsFix, unsigned char* numberOfSatellites, VnVector3* lattitudeLongitudeAltitude, VnVector3* nedVelocity, VnVector3* positionAccuracy, float* speedAccuracy, float* timeAccuracy);

/**
 * \brief Gets the values in the INS Solution register.
 *
 * \param[in] vn200 Pointer to the Vn200 control object.
 * \param[out] gpsTime The GPS time of week in seconds value of the sensor.
 * \param[out] gpsWeek The GPS week value of the sensor.
 * \param[out] status The status flags for the INS filter value of the sensor.
 * \param[out] ypr The current sensor heading, pitch, and roll values.
 * \param[out] lattitudeLongitudeAltitude The current sensor latitude, longitude, and altitude values.
 * \param[out] nedVelocity The current sensor velocity measurements (X,Y,Z) in north, east, down directions values.
 * \param[out] attitudeUncertainty The attitude uncertainty value of the sensor.
 * \param[out] positionUncertainty The position uncertainty value of the sensor.
 * \param[out] velocityUncertainty The velocity uncertainty value of the sensor.
 * \return VectorNav error code.
 */
DLL_EXPORT VN_ERROR_CODE vn200_getInsSolution(Vn200* vn200, double* gpsTime, unsigned short* gpsWeek, unsigned short* status, VnVector3* ypr, VnVector3* lattitudeLongitudeAltitude, VnVector3* nedVelocity, float* attitudeUncertainty, float* positionUncertainty, float* velocityUncertainty);


#ifdef __cplusplus
}
#endif

#endif /* _VN200_H_ */
