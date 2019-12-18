#pragma once
#ifndef K_MAJORITY_DATA_H
#define K_MAJORITY_DATA_H

#include <vector>
#include <string>

struct Data {
	Data(std::string _filename, int _DATA_DIMENSION = 32);
	Data(const std::vector<unsigned char>& _value, std::string _filename, int _DATA_DIMENSION = 32);
	std::vector<unsigned char> value;
	std::string filename;
	int DATA_DIMENSION;
};



#endif