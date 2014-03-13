#ifndef MESHDATA_H
#define MESHDATA_H

#include <vector>

class segData{
public:
	segData():_segIdxList(nullptr){}

	typedef	std::unique_ptr<int[]> segIdxList;
	segIdxList _segIdxList;
};


class meshData{
public:
	typedef std::unique_ptr<segData> segPtr;
	typedef std::unique_ptr<int[]> intVecPtr;
	typedef std::unique_ptr<float[]> floatVecPtr;

	meshData():_segList(), _neighbourPtr(nullptr), _weightsPtr(nullptr), _posPtr(nullptr) {}

	std::vector<segPtr>	_segList;
	intVecPtr			_neighbourPtr;
	floatVecPtr			_weightsPtr;
	floatVecPtr			_posPtr;

};


#endif