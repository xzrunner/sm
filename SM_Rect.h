#ifndef _SPATIAL_MATH_RECT_H_
#define _SPATIAL_MATH_RECT_H_

#include "SM_Vector.h"

#include <stdint.h>

namespace sm
{

template <typename T>
class Rect
{
public:
	T xmin, ymin;
	T xmax, ymax;

public:
	Rect();
	Rect(T width, T height);
	Rect(const Vector2<T>& center, T width, T height);
	Rect(const Vector2<T>& v0, const Vector2<T>& v1);
	Rect(T xmin, T ymin, T xmax, T ymax);

	bool operator == (const Rect& r) const;
	bool operator != (const Rect& r) const;

	void Build(T width, T height);
	void Build(const Vector2<T>& center, T width, T height);

	void MakeEmpty();
	bool IsValid() const;

	bool Combine(const Vector2<T>& v);
	bool Combine(const Rect<T>& r);

	T Width() const;
	T Height() const;
	Vector2<T> Size() const;
	Vector2<T> Center() const;

	void Translate(const Vector2<T>& trans);
	void Scale(const Vector2<T>& scale);
	void Shear(const Vector2<T>& shear);

}; // Rect

typedef Rect<int>     irect;
typedef Rect<int16_t> i16_rect;

typedef Rect<float> rect;

}

#include "SM_Rect.inl"

#endif // _SPATIAL_MATH_RECT_H_