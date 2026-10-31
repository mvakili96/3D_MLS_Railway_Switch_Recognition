#include "SharedData.h"

bool SharedData::instanceFlag = false;
SharedData* SharedData::single = NULL;
SharedData* SharedData::getInstance()
{
	if (!instanceFlag)
	{
		single = new SharedData();
		instanceFlag = true;
		return single;
	}
	else
	{
		return single;
	}
}

void SharedData::method()
{
	
}