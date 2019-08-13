#ifndef _SPATIAL_MATH_PLANE_H_
#define _SPATIAL_MATH_PLANE_H_

#include "SM_Vector.h"

namespace sm
{

template <typename T>
class PlaneT
{
public:
	PlaneT() : dist(0) {}
	PlaneT(const Vector3<T>& normal, T d);
	PlaneT(const Vector3<T>& normal, const Vector3<T>& v0);
	PlaneT(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2);

    T GetDistance(const Vector3<T>& v) const;

	void Build(const Vector3<T>& normal, T d);
	void Build(const Vector3<T>& normal, const Vector3<T>& v);
	void Build(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2);

    void Flip();

public:
	Vector3<T> normal;
	T          dist;

}; // PlaneT

typedef PlaneT<float> Plane;

}

#include "SM_Plane.inl"

#endif // _SPATIAL_MATH_PLANE_H_