#ifndef _SPATIAL_MATH_RECT_INL_
#define _SPATIAL_MATH_RECT_INL_

#include <limits>
#include <algorithm>

namespace sm
{

template <typename T>
Rect<T>::Rect()
{
	MakeEmpty();
}

template <typename T>
Rect<T>::Rect(T width, T height)
{
	Build(width, height);
}

template <typename T>
Rect<T>::Rect(const Vector2<T>& center, T width, T height)
{
	Build(width, height);
	Translate(center);
}

template <typename T>
Rect<T>::Rect(const Vector2<T>& v0, const Vector2<T>& v1)
{
	xmin = (std::min)(v0.x, v1.x);
	ymin = (std::min)(v0.y, v1.y);
	xmax = (std::max)(v0.x, v1.x);
	ymax = (std::max)(v0.y, v1.y);
}

template <typename T>
Rect<T>::Rect(T xmin, T ymin, T xmax, T ymax)
	: xmin(xmin)
	, ymin(ymin)
	, xmax(xmax)
	, ymax(ymax)
{
}

template <typename T>
bool Rect<T>::operator == (const Rect<T>& r) const
{
	return xmin == r.xmin
		&& xmax == r.xmax
		&& ymin == r.ymin
		&& ymax == r.ymax;
}

template <typename T>
void Rect<T>::Build(T width, T height)
{
	T hw = width * 0.5f,
	  hh = height * 0.5f;
	xmin = -hw;
	xmax =  hw;
	ymin = -hh;
	ymax =  hh;	
}

template <typename T>
void Rect<T>::MakeEmpty()
{
	xmin = ymin = std::numeric_limits<T>::max();
	xmax = ymax = -std::numeric_limits<T>::max();
}

template <typename T>
bool Rect<T>::IsValid() const
{
	return xmin != std::numeric_limits<T>::max() && ymin != std::numeric_limits<T>::max()
		&& xmax != -std::numeric_limits<T>::max() && ymax != -std::numeric_limits<T>::max()
		&& xmin <= xmax && ymin <= ymax;
}

template <typename T>
void Rect<T>::Combine(const Vector2<T>& v)
{
	if (v.x < xmin) xmin = v.x;
	if (v.x > xmax) xmax = v.x;
	if (v.y < ymin) ymin = v.y;
	if (v.y > ymax) ymax = v.y;
}

template <typename T>
void Rect<T>::Combine(const Rect<T>& r)
{
	if (r.xmin < xmin) xmin = r.xmin;
	if (r.xmax > xmax) xmax = r.xmax;
	if (r.ymin < ymin) ymin = r.ymin;
	if (r.ymax > ymax) ymax = r.ymax;
}

template <typename T>
Vector2<T> Rect<T>::Size() const
{
	return Vector2<T>(xmax - xmin, ymax - ymin);
}

template <typename T>
Vector2<T> Rect<T>::Center() const
{
	return Vector2<T>((xmin + xmax) * 0.5f, (ymin + ymax) * 0.5f);	
}

template <typename T>
void Rect<T>::Translate(const Vector2<T>& trans)
{
	xmin += trans.x;
	xmax += trans.x;
	ymin += trans.y;
	ymax += trans.y;
}

template <typename T>
void Rect<T>::Scale(const Vector2<T>& scale)
{
	xmin *= scale.x;
	xmax *= scale.x;
	ymin *= scale.y;
	ymax *= scale.y;
	if (xmin > xmax) {
		std::swap(xmin, xmax);
	}
	if (ymin > ymax) {
		std::swap(ymin, ymax);
	}
}

template <typename T>
void Rect<T>::Shear(const Vector2<T>& shear)
{
	// x' = x + y*shear.x
	// y' = x*shear.y + y
	sm ::vec2 v[4];
	v[0].x = xmin + ymin * shear.x;
	v[0].y = xmin * shear.y + ymin;
	v[1].x = xmax + ymin * shear.x;
	v[1].y = xmax * shear.y + ymin;
	v[2].x = xmax + ymax * shear.x;
	v[2].y = xmax * shear.y + ymax;
	v[3].x = xmin + ymax * shear.x;
	v[3].y = xmin * shear.y + ymax;

	MakeEmpty();
	for (int i = 0; i < 4; ++i) {
		Combine(v[i]);
	}
}

}

#endif // _SPATIAL_MATH_RECT_INL_