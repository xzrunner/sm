#include "SM_Calc.h"
#include "SM_Vector.h"
#include "sm_c_vector.h"

namespace sm
{

extern "C"
void sm_rotate_vector_right_angle(const struct sm_vec2* v, bool turn_left, struct sm_vec2* ret)
{
	vec2 r = rotate_vector_right_angle(*(vec2*)v, turn_left);
	ret->x = r.x;
	ret->y = r.y;
}

extern "C"
float sm_get_line_angle(struct sm_vec2* s, struct sm_vec2* e)
{
	return get_line_angle(*(vec2*)s, *(vec2*)e);
}

extern "C"
bool sm_intersect_line_line(const struct sm_vec2* s0, const struct sm_vec2* e0, const struct sm_vec2* s1, const struct sm_vec2* e1, struct sm_vec2* cross)
{
	vec2 _cross;
	bool ret = intersect_line_line(*(vec2*)s0, *(vec2*)e0, *(vec2*)s1, *(vec2*)e1, &_cross);
	cross->x = _cross.x;
	cross->y = _cross.y;
	return ret;
}

}