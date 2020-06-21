#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_c_matrix_h
#define spatial_math_c_matrix_h

union sm_mat3 {
	float c[3][3];
	float x[9];
};

union sm_mat4 {
	float c[4][4];
	float x[16];
};

union sm_mat4* sm_mat4_mul(union sm_mat4* m, const union sm_mat4* m1, const union sm_mat4* m2);
union sm_mat4* sm_mat4_rotxmat(union sm_mat4* m, float degrees);
union sm_mat4* sm_mat4_identity(union sm_mat4* m);
union sm_mat4* sm_mat4_trans(union sm_mat4* m, float x, float y, float z);
union sm_mat4* sm_mat4_perspective(union sm_mat4 *m, float fovy, float aspect, float znear, float zfar);
struct sm_vec3* sm_vec3_mul(struct sm_vec3* v, const union sm_mat4* m);

#endif // spatial_math_matrix_h

#ifdef __cplusplus
}
#endif