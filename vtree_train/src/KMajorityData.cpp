#include "KMajorityData.h"

using namespace std;

Data::Data(string _filename, int _DATA_DIMENSION)
	: DATA_DIMENSION(_DATA_DIMENSION), value(vector<unsigned char>(DATA_DIMENSION, 0)), filename(_filename)
{}

Data::Data(const vector<unsigned char>& _value, string _filename, int _DATA_DIMENSION)
	: DATA_DIMENSION(_DATA_DIMENSION), value(_value), filename(_filename)
{}