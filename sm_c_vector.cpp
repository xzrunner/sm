#include "SM_Vector.h"

namespace sm
{

extern "C"
struct sm_vec3* sm_vec3_vector(struct sm_vec3* v, const struct sm_vec3* p1, const struct sm_vec3* p2) 
{
	*(vec3*)v = (*(const vec3*)p1) - (*(const vec3*)p2);
	return v;
}

extern "C"
struct sm_vec3* sm_vec3_cross(struct sm_vec3* v, const struct sm_vec3* a, const struct sm_vec3* b)
{
	*(vec3*)v = (*(const vec3*)a).Cross(*(const vec3*)b);
	return v;
}

extern "C"
struct sm_vec3* sm_vec3_normalize(struct sm_vec3* v)
{
	((vec3*)v)->Normalize();
	return v;
}

}