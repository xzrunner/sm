#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_c_vector_h
#define spatial_math_c_vector_h

struct sm_ivec2 {
	int x, y;
};

struct sm_vec2 {
	float x, y;
};

struct sm_vec3 {
	float x, y, z;
};

struct sm_vec4 {
	float x, y, z, w;
};

struct sm_vec2* sm_vec2_vector(struct sm_vec2* v, const struct sm_vec2* p1, const struct sm_vec2* p2);
struct sm_vec2* sm_vec2_add(struct sm_vec2* v, const struct sm_vec2* p1, const struct sm_vec2* p2);
struct sm_vec2* sm_vec2_normalize(struct sm_vec2* v);

struct sm_vec3* sm_vec3_vector(struct sm_vec3* v, const struct sm_vec3* p1, const struct sm_vec3* p2);
struct sm_vec3* sm_vec3_cross(struct sm_vec3* v, const struct sm_vec3* a, const struct sm_vec3* b);
struct sm_vec3* sm_vec3_normalize(struct sm_vec3* v);

#endif // spatial_math_c_vector_h

#ifdef __cplusplus
}
#endif