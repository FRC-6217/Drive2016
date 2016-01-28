#include "Autonomous.h"

#include <stdio.h>

#include "WPILib.h"

bool doNothing() {
	Wait(0.1);
	printf("Doing Nothing.\n");
	return true;
}

//cross functions
bool lowBar() {
	return false;
}

bool portcullis() {
	return false;
}
bool chevalDeFrise() {
	return false;
}
bool moat() {
	return false;
}
bool ramparts() {
	return false;
}
bool drawBridge() {
	return false;
}
bool sallyPort() {
	return false;
}
bool rockWall() {
	return false;
}
bool roughTerrain() {
	return false;
}
