#ifndef _VNEZASYNCDATA_H_
#define _VNEZASYNCDATA_H_

#include "vnint.h"
#include "vnerror.h"
#include "vncompositedata.h"
#include "vnsensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Structure supporting easy and reliable access to asynchronous data
*   from a VectorNav sensor at the cost of a slight performance hit. */
struct VnEzAsyncData
{
	/** \brief The associated connected sensor. */
	struct VnSensor* sensor;

	/** \brief Critical section for accessing the current data. */
	struct VnCriticalSection* curDataCS;

	/** \brief The current data received from asynchronous data packets. */
	struct VnCompositeData* curData;

};

#ifndef __cplusplus
typedef struct VnEzAsyncData VnEzAsyncData_t;
#endif

/** \brief Initializes and connects to a VectorNav sensor with the specified
*   connection parameters.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \param]in] portName The name of the serial port to connect to.
* \param[in] baudrate The baudrate to connect at.
* \return Any errors encountered. */
enum VnError VnEzAsyncData_initializeAndConnect(struct VnEzAsyncData* ezAsyncData, const char* portName, uint32_t baudrate);

/** \brief Disconnects from a VectorNav sensor.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \return Any errors encountered. */
enum VnError VnEzAsyncData_disconnect(struct VnEzAsyncData* ezAsyncData);

/** \brief Returns the most recent asynchronous data the VnEzAsyncData structure
*   has processed.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \return The latest data processed. */
struct VnCompositeData VnEzAsyncData_currentData(struct VnEzAsyncData* ezAsyncData);

/** \brief Returns the underlying VnSensor referenced.
*
* \param[in] ezAsyncData The associated VnEzAsyncData structure.
* \return The underlying VnSensor reference. */
struct VnSensor* VnEzAsyncData_sensor(struct VnEzAsyncData* ezAsyncData);

#ifdef __cplusplus
}
#endif

#endif
