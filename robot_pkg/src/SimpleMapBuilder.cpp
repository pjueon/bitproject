#include "SimpleMapBuilder.h"
#include <cmath>
#include <vector>

#include <iostream>

using namespace std;

constexpr bool CLEAR = true;
constexpr bool BLOCKED = false;

//===========================================================
SimpleMapBuilder::SimpleMapBuilder():origin_w(0), origin_h(0), simple_w(0), simple_h(0), blockThreshold(50), compressionRate(4){}


//===========================================================
bool SimpleMapBuilder::isClear(signed char value) const {
    return value != -1 && value < blockThreshold;
}

//===========================================================
void SimpleMapBuilder::setOriginalMapSize(int w, int h){
	origin_w = w;
	origin_h = h;
}

//===========================================================
void SimpleMapBuilder::setCompressionRate(int compressionRate){
	this->compressionRate = compressionRate;
}

//===========================================================
void SimpleMapBuilder::setBlockThreshold(int blockThreshold){
	this->blockThreshold = blockThreshold;
}


//===========================================================
bool SimpleMapBuilder::calculateSimpleValue(int x, int y) const {
    for(int i = y; i < y + compressionRate; i++)
            for(int j = x; j < x + compressionRate; j++)
                    if(paddedMap[i][j] == BLOCKED) return BLOCKED;
    return CLEAR;
}


//===========================================================
void SimpleMapBuilder::setMapData(const std::vector<signed char>& data){
	cout << "SimpleMapBuilder::setMapData starts" << endl;

    simple_w = std::ceil(double(origin_w)/compressionRate);
    simple_h = std::ceil(double(origin_h)/compressionRate);

    int new_w =  simple_w * compressionRate;
    int new_h =  simple_h * compressionRate;

    paddedMap.assign(new_h, std::vector<bool>(new_w, BLOCKED));

	cout << "[DEBUG] origin_w, origin_h: " << origin_w  << ", " << origin_h << endl;
	cout << "[DEBUG]simple_w, simple_h: " << simple_w  << ", " << simple_h << endl;
	cout << "[DEBUG]new_w, new_h: " << new_w  << ", " << new_h << endl;
	//cout << "data.size():" << data.size() << endl;
	//cout << "origin_w*origin_h = " << origin_w*origin_h  << endl;

	for(int i = 0; i < data.size(); i++){
		int x = i % origin_w;
		int y = i / origin_w;
		paddedMap[y][x] = isClear(data[i]);
	}

	simpleMap.assign(simple_h, std::vector<bool>(simple_w, BLOCKED));

	for(int y = 0; y < simple_h; y++){
		for(int x = 0; x < simple_w; x++){
			simpleMap[y][x] = calculateSimpleValue(compressionRate*x, compressionRate*y);
		}
	}

	cout << "SimpleMapBuilder::setMapData ends" << endl;
}


//===========================================================
std::vector<std::vector<bool>> SimpleMapBuilder::getSimpleMap() const{
	return simpleMap;
}

std::vector<std::vector<bool>> SimpleMapBuilder::getPaddedMap() const{
	return paddedMap;
}

bool SimpleMapBuilder::getPaddedMapValue(int x, int y) const {
	if(0 <= y && y < paddedMap.size() && 0 <= x && x < paddedMap[0].size()) return paddedMap[y][x];
	return false;
}


bool SimpleMapBuilder::getSimpleMapValue(int x, int y) const {
	if(0 <= y && y < simpleMap.size() && 0 <= x && x < simpleMap[0].size()) return simpleMap[y][x];
	return false;
}


