#ifndef JOINTDATA_H
#define JOINTDATA_H

#include "vector.h"

class localCoord{
public:
	localCoord():_center(), _axisX(), _axisY(), _axisZ(), _bbox() {};

	MPoint	_center;
	MVector _axisX;
	MVector _axisY;
	MVector _axisZ;
	std::pair<MVector, MVector> bbox;
};

#endif