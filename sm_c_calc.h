#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_c_calc_h
#define spatial_math_c_calc_h

void sm_rotate_vector_right_angle(const struct sm_vec2* v, bool turn_left, struct sm_vec2* ret);

float sm_get_line_angle(struct sm_vec2* s, struct sm_vec2* e);

bool sm_intersect_line_line(const struct sm_vec2* s0, const struct sm_vec2* e0, const struct sm_vec2* s1, const struct sm_vec2* e1, struct sm_vec2* cross);

#endif // spatial_math_c_calc_h

#ifdef __cplusplus
}
#endif