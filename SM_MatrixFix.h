#ifndef _SPATIAL_MATH_FIX_MATH_H_
#define _SPATIAL_MATH_FIX_MATH_H_

#include "SM_Vector.h"

namespace sm
{

class MatrixFix
{
public:
	int x[6];

public:
	static const int SCALE           = 1024;
	static const int TRANSLATE_SCALE = 16;
	static const int ROTATE_SCALE    = 4096;

	static const float SCALE_INV;
	static const float TRANSLATE_SCALE_INV;

public:
	MatrixFix();
	MatrixFix(const MatrixFix& mt);
	MatrixFix& operator = (const MatrixFix& mt);

	MatrixFix operator * (const MatrixFix& b) const;
	vec2 operator * (const vec2& v) const;

	void Identity();

	MatrixFix Inverted() const;

	void Translate(float x, float y);
	void Scale(float sx, float sy);
	void Rotate(float angle);

	void SetTransformation(float x, float y, float angle, float sx, float sy, float ox, float oy, float kx, float ky);
	
}; // MatrixFix

}

#include "SM_MatrixFix.inl"

#endif // _SPATIAL_MATH_FIX_MATH_H_