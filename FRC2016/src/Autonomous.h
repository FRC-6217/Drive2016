#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include <map>

bool doNothing();
bool lowBar();

const std::map<std::string, bool (*)()> crossFunctions
{
	{"Low Bar", &lowBar}
};

#endif
