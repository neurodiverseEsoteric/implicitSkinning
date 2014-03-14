#ifndef JOINTDATA_H
#define JOINTDATA_H

#include <vector>
#include "meshData.h"
#include "transform.h"

class jointData {
public:
	jointData():_segDataList(nullptr), _transform(){}
	~jointData(){}
	bool getLocalCoord(const segData& seg);

public:
	std::unique_ptr<std::vector<segData>> _segDataList;
	Transform	_transform;
	int			_parentPos;
	int			_index;
	std::size_t	_hashCode;
};

#endif