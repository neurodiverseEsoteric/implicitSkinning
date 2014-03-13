#ifndef JOINTDATA_H
#define JOINTDATA_H

#include <vector>
#include "meshData.h"
#include "transform.h"

class jointData {
public:
	typedef std::unique_ptr<std::vector<segData>> _segDataList;

	jointData():_segDataList(nullptr){}
	~jointData(){}

	Transform	_transform;
	int			_parentPos;
	int			_index;
	std::size_t	_hashCode;
};

#endif