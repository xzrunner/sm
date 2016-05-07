#ifndef _SPATIAL_MATH_PLANE_H_
#define _SPATIAL_MATH_PLANE_H_

#include "SM_Vector.h"

namespace sm
{

template <typename T>
class PlaneT
{
public:
	PlaneT(const Vector3<T>& normal, T d);
	PlaneT(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2);

	void GetDistance(const Vector3<T>& v) const;

private:
	Vector3<T> m_normal;
	T m_dist;

}; // PlaneT

typedef PlaneT<float> Plane;

}

#include "SM_Plane.inl"

#endif // _SPATIAL_MATH_PLANE_H_