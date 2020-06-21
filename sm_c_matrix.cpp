#include "SM_Matrix.h"

namespace sm
{

extern "C"
union sm_mat4* sm_mat4_mul(union sm_mat4* m, const union sm_mat4* m1, const union sm_mat4* m2)
{
	*(mat4*)m = (*(mat4*)m1) * (*(mat4*)m2);
	return m;
}

extern "C"
union sm_mat4* sm_mat4_rotxmat(union sm_mat4* m, float degrees)
{
	*(mat4*)m = ((mat4*)m)->RotatedX(degrees);
	return m;
}

extern "C"
union sm_mat4* sm_mat4_identity(union sm_mat4* m)
{
	((mat4*)m)->Identity();
	return m;
}

extern "C"
union sm_mat4* sm_mat4_trans(union sm_mat4* m, float x, float y, float z)
{
	((mat4*)m)->Translate(x, y, z);
	return m;
}

extern "C"
union sm_mat4* sm_mat4_perspective(union sm_mat4* m, float fovy, float aspect, float znear, float zfar)
{
	*(mat4*)m = ((mat4*)m)->Perspective(fovy, aspect, znear, zfar);
	return m;
}

extern "C"
struct sm_vec3* sm_vec3_mul(struct sm_vec3* v, const union sm_mat4* m)
{
	*(vec3*)v = *((mat4*)m) * (*(vec3*)v);
	return v;
}

}