#include <stdio.h>
#include <unistd.h>
#include "vectornav.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyS1";
const int BAUD_RATE = 115200;

int main()
{
	Vn100 vn100;
	VnYpr ypr;
	int i;

	vn100_connect(&vn100, COM_PORT, BAUD_RATE);

	for (i = 0; i < 10; i++) {
		vn100_getYawPitchRoll(&vn100, &ypr);
		printf("YPR: %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
		sleep(1);
	}
	
	vn100_disconnect(&vn100);

	return 0;
}

