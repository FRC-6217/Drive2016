#include "Autonomous.h"

#include <stdio.h>

#include "WPILib.h"

bool doNothing() {
	Wait(0.1);
	printf("Doing Nothing.\n")
	return true;
}

bool lowBar() {
	return false;
}


