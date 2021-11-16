#include "SM_Calc.h"

namespace sm
{

sm::vec3 calc_plane_mirror(const Plane& plane, const sm::vec3& pos)
{
	float len_s = plane.normal.LengthSquared();
	auto v1 = pos;
	float k = (-plane.normal.Dot(v1) - plane.dist) / len_s;
	auto v2 = plane.normal * k + v1;
	return v2 * 2 - v1;
}

}