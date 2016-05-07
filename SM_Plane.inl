#ifndef _SPATIAL_MATH_PLANE_INL_
#define _SPATIAL_MATH_PLANE_INL_

namespace sm
{

template <typename T> 
inline PlaneT<T>::PlaneT(const Vector3<T>& normal, T d)
	: m_normal(normal)
{
	T inv_len = 1.0f / normal.Length();
	m_normal *= inv_len;
	m_dist = d * inv_len;
}

template <typename T>
inline PlaneT<T>::PlaneT(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2)
{
	Vector3<T> a = v1 - v0,
		 b = v2 - v0;
	m_normal = a.Cross(b);
	m_normal.Normalize();
	m_dist = -m_normal.Dot(v0);
}

template <typename T>
inline void PlaneT<T>::GetDistance(const Vector3<T>& v) const
{
	T d = m_normal.Dot(v);
	return d + m_dist;
}

}

#endif // _SPATIAL_MATH_PLANE_INL_