#ifndef _SPATIAL_MATH_PLANE_INL_
#define _SPATIAL_MATH_PLANE_INL_

namespace sm
{

template <typename T>
inline PlaneT<T>::PlaneT(const Vector3<T>& n, T d)
	: normal(n)
{
	Build(n, d);
}

template <typename T>
inline PlaneT<T>::PlaneT(const Vector3<T>& normal, const Vector3<T>& v0)
{
	Build(normal, v0);
}

template <typename T>
inline PlaneT<T>::PlaneT(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2)
{
	Build(v0, v1, v2);
}

template <typename T>
inline T PlaneT<T>::GetDistance(const Vector3<T>& v) const
{
	T d = normal.Dot(v);
	return d + dist;
}

template <typename T>
inline void PlaneT<T>::Build(const Vector3<T>& normal, T d)
{
	T inv_len = 1.0f / normal.Length();
	this->normal = normal * inv_len;
	this->dist = d * inv_len;
}

template <typename T>
inline void PlaneT<T>::Build(const Vector3<T>& normal, const Vector3<T>& v)
{
	this->normal = normal.Normalized();
	this->dist = -v.Dot(normal);
}

template <typename T>
inline void PlaneT<T>::Build(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2)
{
	Vector3<T> a = v1 - v0,
		       b = v2 - v0;
	normal = a.Cross(b);
	normal.Normalize();
	dist = -normal.Dot(v0);
}

template <typename T>
void PlaneT<T>::Flip()
{
    normal = -normal;
    dist   = -dist;
}

}

#endif // _SPATIAL_MATH_PLANE_INL_