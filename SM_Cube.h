#ifndef _SPATIAL_MATH_CUBE_H_
#define _SPATIAL_MATH_CUBE_H_

#include "SM_Vector.h"

#include <stdint.h>

namespace sm
{

template <typename T>
class Cube
{
public:
	T xmin, ymin;
	T xmax, ymax;
	T zmin, zmax;

public:
	Cube();
	Cube(T width, T height, T depth);
	Cube(const Vector3<T>& center, T width, T height, T depth);
	Cube(const Vector3<T>& v0, const Vector3<T>& v1);
	Cube(T xmin, T ymin, T zmin, T xmax, T ymax, T zmax);

	bool operator == (const Cube& r) const;
	bool operator != (const Cube& r) const;

	void Build(T width, T height, T depth);

	void MakeEmpty();
	bool IsValid() const;

	bool Combine(const Vector3<T>& v);
	bool Combine(const Cube<T>& r);

	T Width() const;
	T Height() const;
	T Depth() const;

	Vector3<T> Size() const;
	Vector3<T> Center() const;

	void Translate(const Vector3<T>& trans);
	void Scale(const Vector3<T>& scale);

}; // Cube

typedef Cube<int16_t> i16_cube;

typedef Cube<float> cube;

}

#include "SM_Cube.inl"

#endif // _SPATIAL_MATH_CUBE_H_