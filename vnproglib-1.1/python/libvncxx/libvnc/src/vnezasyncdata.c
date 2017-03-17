#include "vnsensor.h"
#include "vncompositedata.h"
#include "vnezasyncdata.h"
#include "vncriticalsection.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void VnEzAsyncData_processReceivedAsyncPacket(void *userData, struct VnUartPacket *packet, size_t runningIndex);

enum VnError VnEzAsyncData_initializeAndConnect(struct VnEzAsyncData* ez, const char* portName, uint32_t baudrate)
{
	enum VnError error;

	/** \brief The associated connected sensor. */
    ez->sensor = (struct VnSensor*)malloc(sizeof(struct VnSensor));
    ez->curData = (struct VnCompositeData*)malloc(sizeof(struct VnCompositeData));
    ez->curDataCS = (struct VnCriticalSection*)malloc(sizeof(struct VnCompositeData));

    if((ez->sensor == NULL) ||
       (ez->curData == NULL) ||
       (ez->curDataCS == NULL))
    {
        error = E_MEMORY_NOT_ALLOCATED;
        return error;
    }

	VnCompositeData_initialize(ez->curData);

	if ((error = VnCriticalSection_initialize(ez->curDataCS)) != E_NONE)
		return error;

	if ((error = VnSensor_initialize(ez->sensor)) != E_NONE)
		return error;

	if ((error = VnSensor_connect(ez->sensor, portName, baudrate)) != E_NONE)
		return error;

	return VnSensor_registerAsyncPacketReceivedHandler(ez->sensor, VnEzAsyncData_processReceivedAsyncPacket, ez);
}

enum VnError VnEzAsyncData_disconnect(struct VnEzAsyncData* ezAsyncData)
{
	enum VnError error;

	if ((error = VnSensor_unregisterAsyncPacketReceivedHandler(ezAsyncData->sensor)) != E_NONE)
		return error;

	if ((error = VnCriticalSection_deinitialize(ezAsyncData->curDataCS)) != E_NONE)
		return error;

	return VnSensor_disconnect(ezAsyncData->sensor);
}

struct VnCompositeData VnEzAsyncData_currentData(struct VnEzAsyncData* ez)
{
	struct VnCompositeData cd;

	VnCriticalSection_enter(ez->curDataCS);

	cd = *(ez->curData);

	VnCriticalSection_leave(ez->curDataCS);

	return cd;
}

struct VnSensor* VnEzAsyncData_sensor(struct VnEzAsyncData* ez)
{
	return ez->sensor;
}

void
VnEzAsyncData_processReceivedAsyncPacket(
    void *userData,
    struct VnUartPacket *packet,
    size_t runningIndex)
{
  struct VnEzAsyncData* ez = (struct VnEzAsyncData*) userData;

  VnCriticalSection_enter(ez->curDataCS);

  VnCompositeData_parse(ez->curData, packet);

  VnCriticalSection_leave(ez->curDataCS);
}
