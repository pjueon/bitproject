#pragma once
#ifndef SIMPLE_MAP_BUILDER_H
#define SIMPLE_MAP_BUILDER_H

#include<vector>

class SimpleMapBuilder{
private:
	int origin_w;
	int origin_h;

	int simple_w;
	int simple_h;

	int blockThreshold;
	int compressionRate;
	
	std::vector<std::vector<bool>> paddedMap;
	std::vector<std::vector<bool>> simpleMap;


	bool isClear(signed char value) const;
	bool calculateSimpleValue(int x, int y) const;


public:
	SimpleMapBuilder();
	void setOriginalMapSize(int w, int h);
	void setCompressionRate(int compressionRate);
	void setBlockThreshold(int blockThreshold);
	void setMapData(const std::vector<signed char>& data);

	std::vector<std::vector<bool>> getSimpleMap() const;
	std::vector<std::vector<bool>> getPaddedMap() const;

	bool getPaddedMapValue(int x, int y) const;
	bool getSimpleMapValue(int x, int y) const;

};

#endif







