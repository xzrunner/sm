#ifndef _SPATIAL_MATH_CUBE_INL_
#define _SPATIAL_MATH_CUBE_INL_

#include <limits>
#include <algorithm>

namespace sm
{

template <typename T>
Cube<T>::Cube()
{
	MakeEmpty();
}

template <typename T>
Cube<T>::Cube(T width, T height, T depth)
{
	Build(width, height, depth);
}

template <typename T>
Cube<T>::Cube(const Vector3<T>& center, T width, T height, T depth)
{
	Build(width, height, depth);
	Translate(center);
}

template <typename T>
Cube<T>::Cube(const Vector3<T>& v0, const Vector3<T>& v1)
{
	xmin = (std::min)(v0.x, v1.x);
	ymin = (std::min)(v0.y, v1.y);
	zmin = (std::min)(v0.z, v1.z);

	xmax = (std::max)(v0.x, v1.x);
	ymax = (std::max)(v0.y, v1.y);
	zmax = (std::max)(v0.z, v1.z);
}

template <typename T>
Cube<T>::Cube(T xmin, T ymin, T zmin, T xmax, T ymax, T zmax)
	: xmin(xmin)
	, ymin(ymin)
	, zmin(zmin)
	, xmax(xmax)
	, ymax(ymax)
	, zmax(zmax)
{
}

template <typename T>
bool Cube<T>::operator == (const Cube<T>& r) const
{
	return xmin == r.xmin
		&& xmax == r.xmax
		&& ymin == r.ymin
		&& ymax == r.ymax
		&& zmin == r.zmin
		&& zmax == r.zmax;
}

template <typename T>
bool Cube<T>::operator != (const Cube<T>& r) const
{
	return !(*this == r);
}

template <typename T>
void Cube<T>::Build(T width, T height, T depth)
{
	T hx = width  * 0.5f,
	  hy = height * 0.5f,
	  hz = depth  * 0.5f;
	xmin = -hx;
	xmax =  hx;
	ymin = -hy;
	ymax =  hy;
	zmin = -hz;
	zmax =  hz;
}

template <typename T>
void Cube<T>::MakeEmpty()
{
	xmin = ymin = zmin = std::numeric_limits<T>::max();
	xmax = ymax = zmax = -std::numeric_limits<T>::max();
}

template <typename T>
bool Cube<T>::IsValid() const
{
	return xmin != std::numeric_limits<T>::max()
		&& ymin != std::numeric_limits<T>::max()
		&& zmin != std::numeric_limits<T>::max()
		&& xmax != -std::numeric_limits<T>::max()
		&& ymax != -std::numeric_limits<T>::max()
		&& zmax != -std::numeric_limits<T>::max()
		&& xmin <= xmax
		&& ymin <= ymax
		&& zmin <= zmax;
}

template <typename T>
bool Cube<T>::Combine(const Vector3<T>& v)
{
	bool ret = false;
	if (v.x < xmin) {
		xmin = v.x;
		ret = true;
	}
	if (v.x > xmax) {
		xmax = v.x;
		ret = true;
	}
	if (v.y < ymin) {
		ymin = v.y;
		ret = true;
	}
	if (v.y > ymax) {
		ymax = v.y;
		ret = true;
	}
	if (v.z < zmin) {
		zmin = v.z;
		ret = true;
	}
	if (v.z > zmax) {
		zmax = v.z;
		ret = true;
	}
	return ret;
}

template <typename T>
bool Cube<T>::Combine(const Cube<T>& r)
{
	bool ret = false;
	if (r.xmin < xmin) {
		xmin = r.xmin;
		ret = true;
	}
	if (r.xmax > xmax) {
		xmax = r.xmax;
		ret = true;
	}
	if (r.ymin < ymin) {
		ymin = r.ymin;
		ret = true;
	}
	if (r.ymax > ymax) {
		ymax = r.ymax;
		ret = true;
	}
	if (r.zmin < zmin) {
		zmin = r.zmin;
		ret = true;
	}
	if (r.zmax > zmax) {
		zmax = r.zmax;
		ret = true;
	}
	return ret;
}

template <typename T>
T Cube<T>::Width() const
{
	return xmax - xmin;
}

template <typename T>
T Cube<T>::Height() const
{
	return ymax - ymin;
}

template <typename T>
T Cube<T>::Depth() const
{
	return zmax - zmin;
}

template <typename T>
Vector3<T> Cube<T>::Size() const
{
	return Vector3<T>(xmax - xmin, ymax - ymin, zmax - zmin);
}

template <typename T>
Vector3<T> Cube<T>::Center() const
{
	return Vector3<T>((xmin + xmax) * 0.5f, (ymin + ymax) * 0.5f, (zmin + zmax) * 0.5f);
}

template <typename T>
void Cube<T>::Translate(const Vector3<T>& trans)
{
	xmin += trans.x;
	xmax += trans.x;
	ymin += trans.y;
	ymax += trans.y;
	zmin += trans.z;
	zmax += trans.z;
}

template <typename T>
void Cube<T>::Scale(const Vector3<T>& scale)
{
	xmin *= scale.x;
	xmax *= scale.x;
	ymin *= scale.y;
	ymax *= scale.y;
	zmin *= scale.z;
	zmax *= scale.z;
	if (xmin > xmax) {
		std::swap(xmin, xmax);
	}
	if (ymin > ymax) {
		std::swap(ymin, ymax);
	}
	if (zmin > zmax) {
		std::swap(zmin, zmax);
	}
}

}

#endif // _SPATIAL_MATH_CUBE_INL_