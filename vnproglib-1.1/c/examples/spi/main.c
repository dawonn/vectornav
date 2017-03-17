#include <stdio.h>
#include <string.h>

/* Include to get access to the VectorNav SPI functions. */
#include "vn/protocol/spi.h"
#include "vn/protocol/common.h"
#include "vn/xplat/thread.h"

int display_error(const char* msg);
void mockspi_initialize(void);
void mockspi_writeread(const char* dataOut, size_t dataOutSize, char* dataIn);

int main(void)
{
	/* This example walks through using the VectorNav C Library to connect to
	 * and interact with a mock VectorNav sensor through the Serial Peripheral
	 * Interaface (SPI). Once you work through and understand the example, you
	 * may want to try replacing the mock functions with ones that interface
	 * with your SPI subsystem. */

	char txbuf[0x100];
	char rxbuf[0x100];
	size_t bufcmdsize;
	size_t responseSize;
	vec3f ypr;
	char strConversions[50];
	size_t i;

	mockspi_initialize();

	/* With SPI 'initialize', let's work through reading the current yaw, pitch,
	 * roll values from the sensor. */

	/* First we must generate the command to query the sensor. */
	bufcmdsize = sizeof(txbuf);		/* First set this variable to the size of the buffer. */
	if (VnSpi_genReadYawPitchRoll(
		txbuf,
		&bufcmdsize,				/* Pass in the pointer since the function will set this with the length of the command generate. */
		0,
		&responseSize) != E_NONE)
		return display_error("Error generating read yaw, pitch, roll command.\n");

	/* Send out the command over SPI. */
	mockspi_writeread(
		txbuf,
		responseSize,
		rxbuf);

	/* Now the sensor will have responded with data on this transaction but
	 * since the sensor only responds on following transaction, we will
	 * disregard this data. These double transactions can be mitigated by only
	 * requesting the same data each time or by staggering the the requested
	 * data in an appropriate order. */

	/* Make sure enough time has passed for the sensor to format the previous response. */
	VnThread_sleepMs(1);	/* Actual sensor requirement is only 50 us. */

	/* Retransmit so the sensor responds with the previous request. */
	mockspi_writeread(
		txbuf,
		responseSize,
		rxbuf);

	/* Now parse the received response. */
	if (VnSpi_parseYawPitchRoll(
		rxbuf,
		&ypr) != E_NONE)
		return display_error("Error parsing yaw, pitch, roll.\n");

	str_vec3f(strConversions, ypr);
	printf("Current YPR: %s\n", strConversions);

	/* We have now shown how to process one full command transaction which
	 * requires two SPI transactions because the VectorNav sensor requires a
	 * short amount of time to ready the response. Now we can optimize this
	 * transaction squence to utilize this behavior when we are only requesting
	 * the same data each time. This is illustrated in the for loop below. */

	for (i = 0; i < 25; i++)
	{
		/* For this loop, we want to display data at ~10 Hz. */
		VnThread_sleepMs(100);

		/* Perform a transaction for the same sensor register. */
		mockspi_writeread(
			txbuf,
			responseSize,
			rxbuf);

		/* Now since the previous command was for the same register, we will
		 * have valid data and can print/use the results. */
		if (VnSpi_parseYawPitchRoll(
			rxbuf,
			&ypr) != E_NONE)
			return display_error("Error parsing yaw, pitch, roll.\n");

		str_vec3f(strConversions, ypr);
		printf("Current YPR: %s\n", strConversions);
	}

	/* We illustrate how to write settings to the sensor by changing the
	 * asynchronous data output type. Note that this setting only affects the
	 * output on the UART ports and has no effect on the SPI ports. It is only
	 * used for illustration purposes. */

	/* Remember to reset the bufcmdsize variable to let the function know how
	 * large the provided buffer is. */
	bufcmdsize = sizeof(txbuf);

	if (VnSpi_genWriteAsyncDataOutputType(
		txbuf,
		&bufcmdsize,
		0,
		&responseSize,
		VNYPR) != E_NONE)
		return display_error("Error generating write async data output type command.\n");

	mockspi_writeread(
		txbuf,
		responseSize,
		rxbuf);

	return 0;
}

int display_error(const char* msg)
{
	printf("%s\n", msg);
	return -1;
}

void mockspi_initialize(void)
{
	/* Do nothing since we are faking the SPI interface. */
}

void mockspi_writeread(const char* dataOut, size_t dataOutSize, char* dataIn)
{
	/* This function fakes a SPI subsystem for this example. */

	char yprResponse[] = { 0x00, 0x01, 0x08, 0x00, 0xd8, 0x9c, 0xd4, 0x42, 0x44, 0xba, 0x9e, 0x40, 0x4e, 0xe4, 0x8b, 0x40 };

	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(dataOut);
	(dataOutSize);

	memcpy(dataIn, yprResponse, sizeof(yprResponse));
}
