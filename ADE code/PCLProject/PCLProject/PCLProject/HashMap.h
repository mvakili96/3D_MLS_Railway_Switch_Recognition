#pragma once

#include <iostream>
#include <unordered_map>
#include <string>
#include "RailwayExtraction.h"


class HashMap
{
	std::unordered_map<string, RailwayExtraction *> htmap;

public:
	void put(string key, RailwayExtraction *value) {
		htmap[key] = value;
	}

	RailwayExtraction *get(string key) {
		return htmap[key];
	}
};

