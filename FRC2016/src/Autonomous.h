#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include <map>

bool doNothing();
//cross functions
bool lowBar();
bool portcullis();
bool chevalDeFrise();
bool moat();
bool ramparts();
bool drawBridge();
bool sallyPort();
bool rockWall();
bool roughTerrain();

const std::map<std::string, bool (*)()> crossFunctions
{
	{"Low Bar", &lowBar},
	{"Portcullis", &portcullis},
	{"Cheval De Frise", &chevalDeFrise},
	{"Moat", &moat},
	{"Ramparts", &ramparts},
	{"Drawbridge", &drawBridge},
	{"Sally Port", &sallyPort},
	{"Rock Wall", &rockWall},
	{"Rough Terrain", &roughTerrain}
};

#endif
