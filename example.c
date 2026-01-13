#include <stdio.h>

#include "subTouch.h"

void app_main(void)
{
	subTouchInit();

//	subTouchDeepSleep();

	subTouchStartScan();
}
