#ifndef _SPATIAL_MATH_RECT_H_
#define _SPATIAL_MATH_RECT_H_

#include "SM_Vector.h"

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
	Rect(const Vector2<T>& center, T width, T height);
	Rect(const Vector2<T>& v0, const Vector2<T>& v1);

	bool operator == (const Rect& r) const;

	void MakeEmpty();
	bool IsValid() const;

	void Combine(const Vector2<T>& v);
	void Combine(const Rect<T>& r);

	Vector2<T> Size() const;
	Vector2<T> Center() const;

	void Translate(const Vector2<T>& offset);
	void Scale(T sx, T sy);
	void Shear(T kx, T ky);

}; // Rect

typedef Rect<float> rect;

}

#include "SM_Rect.inl"

#endif // _SPATIAL_MATH_RECT_H_