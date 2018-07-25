#pragma once

#include "SM_Vector.h"

namespace sm
{

template <typename T>
class RayT
{
public:
	RayT(const Vector3<T>& origin, const Vector3<T>& dir)
		: origin(origin), dir(dir) {}

public:
	Vector3<T> origin;
	Vector3<T> dir;

}; // RayT

typedef RayT<float> Ray;

}