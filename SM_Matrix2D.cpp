#include "SM_Matrix2D.h"
#include "string.h"

namespace sm
{

Matrix2D::Matrix2D()
{
	Identity();
}

Matrix2D::Matrix2D(const mat4& mt)
{
	x[0] = mt.x[0];
	x[1] = mt.x[1];
	x[2] = mt.x[4];
	x[3] = mt.x[5];
	x[4] = mt.x[12];
	x[5] = mt.x[13];
}

Matrix2D::Matrix2D(const Matrix2D& mt)
{
	memcpy(x, mt.x, sizeof(x));
}

Matrix2D& Matrix2D::operator = (const Matrix2D& mt)
{
	memcpy(x, mt.x, sizeof(x));
	return *this;
}

vec2 Matrix2D::operator * (const vec2& v) const
{
	vec2 ret;
	ret.x = (v.x * x[0] + v.y * x[2]) + x[4];
	ret.y = (v.x * x[1] + v.y * x[3]) + x[5];
	return ret;
}

mat4 Matrix2D::ToMat4() const
{
	mat4 mt;
	mt.x[0]  = x[0];
	mt.x[1]  = x[1];
	mt.x[4]  = x[2];
	mt.x[5]  = x[3];
	mt.x[12] = x[4];
	mt.x[13] = x[5];
	return mt;
}

}
