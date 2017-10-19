#include "SM_Test.h"
#include "SM_Calc.h"

namespace sm
{

bool is_rect_intersect_segment(const rect& r, const vec2& s, const vec2& e)
{
	unsigned char type_s, type_e;
	type_s = type_e = 0;
	if (s.x < r.xmin)      // left:  1000
		type_s |= 0x8; 
	else if (s.x > r.xmax) // right: 0100
		type_s |= 0x4; 
	if (s.y < r.ymin)      // down:  0001
		type_s |= 0x1; 
	else if (s.y > r.ymax) // up:    0010
		type_s |= 0x2; 

	if (e.x < r.xmin)      // left:  1000
		type_e |= 0x8; 
	else if (e.x > r.xmax) // right: 0100
		type_e |= 0x4; 
	if (e.y < r.ymin)      // down:  0001
		type_e |= 0x1; 
	else if (e.y > r.ymax) // up:    0010
		type_e |= 0x2; 

	unsigned char comp;
	comp = type_s & type_e;
	if (comp != 0)		// must be outside, so must intersect
		return false;
	comp = type_s | type_e;
	if (comp == 0)		// must be inside, so must not intersect
		return true;

	// test each edge
	if (comp & 0x8)		// 1000, left edge
	{
		float cross_y;
		cross_y = find_y_on_seg(s, e, r.xmin);
		if (cross_y >= r.ymin && cross_y <= r.ymax)
			return true;
	}
	if (comp & 0x4)		// 0100, right edge
	{
		float cross_y;
		cross_y = find_y_on_seg(s, e, r.xmax);
		if (cross_y >= r.ymin && cross_y <= r.ymax)
			return true;
	}
	if (comp & 0x1)		// 0001, down edge
	{
		float cross_x;
		cross_x = find_x_on_seg(s, e, r.ymin);
		if (cross_x >= r.xmin && cross_x <= r.xmax)
			return true;
	}
	if (comp & 0x2)		// 0010, up edge
	{
		float cross_x;
		cross_x = find_x_on_seg(s, e, r.ymax);
		if (cross_x >= r.xmin && cross_x <= r.xmax)
			return true;
	}

	return false;
}

static inline
void project_convex(const CU_VEC<vec2>& c, float angle, float* min, float* max)
{
	*min = FLT_MAX;
	*max = -FLT_MAX;
	for (int i = 0, n = c.size(); i < n; ++i) {
		vec2 v = rotate_vector(c[i], angle);
		if (v.x < *min) {
			*min = v.x;
		}
		if (v.x > *max) {
			*max = v.x;
		}
	}
}

static inline
bool is_project_intersect(float min0, float max0, float min1, float max1)
{
	return !(max1 <= min0 || min1 >= max0);
}

static inline
bool is_convex_intersect_convex(const CU_VEC<vec2>& c0, const CU_VEC<vec2>& c1, float angle)
{
	float min0, max0, min1, max1;
	project_convex(c0, angle, &min0, &max0);
	project_convex(c1, angle, &min1, &max1);
	return is_project_intersect(min0, max0, min1, max1);
}

static inline
bool is_convex_intersect_convex(const CU_VEC<vec2>& c0, const CU_VEC<vec2>& c1, const vec2& v0, const vec2& v1)
{
	float angle = SM_PI * 0.5f - atan2(v1.y - v0.y, v1.x - v0.x);
	return is_convex_intersect_convex(c0, c1, angle);
}

static inline
bool is_convex_intersect_convex(const CU_VEC<vec2>& c0, const CU_VEC<vec2>& c1, const CU_VEC<vec2>& proj)
{
	for (int i = 0, n = c0.size() - 1; i < n; ++i) {
		if (!is_convex_intersect_convex(c0, c1, proj[i], proj[i+1])) {
			return false;
		}				
	}
	if (!is_convex_intersect_convex(c0, c1, proj[c0.size() - 1], proj[0])) {
		return false;
	}
	return true;
}

bool is_convex_intersect_convex(const CU_VEC<vec2>& c0, const CU_VEC<vec2>& c1)
{
	if (c0.size() < 3 || c1.size() < 3) {
		return false;
	}
	if (!is_convex_intersect_convex(c0, c1, c0) ||
		!is_convex_intersect_convex(c0, c1, c1)) {
		return false;
	} else {
		return true;
	}
}

static inline 
bool is_polygon_colckwise(const CU_VEC<vec2>& poly)
{
	if (poly.size() < 3) {
		return false;
	}

	float left = FLT_MAX;
	int left_idx = 0;
	int sz = poly.size();
	for (int i = 0; i < sz; ++i) {
		if (poly[i].x < left) {
			left = poly[i].x;
			left_idx = i;
		}
	}

	const vec2& curr = poly[left_idx];
	const vec2& next = poly[(left_idx+1)%sz];
	const vec2& prev = poly[(left_idx+sz-1)%sz];
	vec2 up(curr.x, curr.y + 1);
	return get_angle(curr, up, next) < get_angle(curr, up, prev);
}

static inline
int get_next_idx_in_ring(int sz, int curr, int step) 
{
	return (curr + sz + step) % sz;
}

static inline
bool is_two_points_same(const vec2& p0, const vec2& p1) 
{
	return fabs(p0.x - p1.x) < SM_LARGE_EPSILON
		&& fabs(p0.y - p1.y) < SM_LARGE_EPSILON;
}

bool is_segment_intersect_polyline(const vec2& s, const vec2& e, const CU_VEC<vec2>& poly)
{
	if (poly.size() < 2) {
		return false;
	} else if (poly.size() < 3) {
		vec2 cross;
		if (intersect_segment_segment(s, e, poly[0], poly[1], &cross)) {
			if (!is_two_points_same(s, cross) && !is_two_points_same(e, cross) &&
				!is_two_points_same(poly[0], cross) && !is_two_points_same(poly[1], cross)) {
					return true;
			}
		}
		return false;
	}

	int sz = poly.size();
	for (int i = 0; i < sz; ++i) 
	{
		const vec2& start = poly[i];
		int end_idx = get_next_idx_in_ring(sz, i, 1);
		const vec2& end = poly[end_idx];
		vec2 cross; 
		if (intersect_segment_segment(s, e, start, end, &cross)) {
			if (is_two_points_same(s, cross) || is_two_points_same(e, cross)) {
				continue;
			}

			if (is_two_points_same(start, cross)) {
				const vec2& start_prev = poly[get_next_idx_in_ring(sz, i, -1)];
				float angle = get_angle(start, end, start_prev);
				if (angle > get_angle(start, end, e) ||
					angle > get_angle(start, end, s)) {
						return true;
				}
			} else if (is_two_points_same(end, cross)) {
				const vec2& end_next = poly[get_next_idx_in_ring(sz, end_idx, 1)];
				float angle = get_angle(end, end_next, start);
				if (angle > get_angle(end, end_next, e) ||
					angle > get_angle(end, end_next, s)) {
						return true;
				}

			} else {
				return true;
			}
		}
	}

	return false;
}

bool is_polygon_intersect_polygon(const CU_VEC<vec2>& poly0, const CU_VEC<vec2>& poly1)
{
	if (poly0.size() < 3 || poly1.size() < 3) {
		return false;
	}

	int sz0 = poly0.size(),
		sz1 = poly1.size();

	int step0 = is_polygon_colckwise(poly0) ? 1 : -1,
		step1 = is_polygon_colckwise(poly1) ? 1 : -1;
	int idx0 = 0;
	for (int i = 0; i < sz0; ++i) {
		int idx1 = 0;
		const vec2& start0 = poly0[idx0];
		int next_idx0 = get_next_idx_in_ring(sz0, idx0, step0);
		const vec2& end0 = poly0[next_idx0];
		for (int i = 0; i < sz1; ++i) {
			const vec2& start1 = poly1[idx1];
			int next_idx1 = get_next_idx_in_ring(sz1, idx1, step1);
			const vec2& end1 = poly1[next_idx1];

			vec2 cross;
			if (intersect_segment_segment(start0, end0, start1, end1, &cross)) {
				// test if cross is end
				bool is_cross0 = is_two_points_same(cross, end0),
					 is_cross1 = is_two_points_same(cross, end1);
				if (is_cross0 && is_cross1) {
					const vec2& end_next0 = poly0[get_next_idx_in_ring(sz0, next_idx0, step0)];
					const vec2& end_next1 = poly1[get_next_idx_in_ring(sz1, next_idx1, step1)];
					float angle0 = get_angle(end0, end_next0, start0);
					if (angle0 > get_angle(end0, end_next0, start1) ||
						angle0 > get_angle(end0, end_next0, end_next1)) {
							return true;
					}
					float angle1 = get_angle(end1, end_next1, start1);
					if (angle1 > get_angle(end1, end_next1, start0) ||
						angle1 > get_angle(end1, end_next1, end_next0)) {
							return true;
					}

					//////////////////////////////////////////////////////////////////////////

					// 					vec2 seg00 = start0 - end0;
					// 					seg00.normalize();
					// 					const vec2& end_next0 = poly0[get_next_idx_in_ring(sz0, next_idx0, step0)];
					// 					vec2 seg01 = end_next0 - end0;
					// 					seg01.normalize();
					// 
					// 					vec2 seg10 = start1 - end1;
					// 					seg10.normalize();
					// 					const vec2& end_next1 = poly0[get_next_idx_in_ring(sz1, next_idx1, step1)];
					// 					vec2 seg11 = end_next1 - end1;
					// 					seg11.normalize();
					// 
					// 					if (IsTwoSegmentIntersect(seg00, seg01, seg10, seg11) &&
					// 						!is_two_points_same(seg00, seg10) && !is_two_points_same(seg00, seg11) && 
					// 						!is_two_points_same(seg01, seg10) && !is_two_points_same(seg01, seg11)) {
					// 							return true;
					// 					}
				} else if (is_cross0) {
					const vec2& end_next0 = poly0[get_next_idx_in_ring(sz0, next_idx0, step0)];
					if (is_turn_left(end_next0, end0, end1)) { 
						return true; 
					}
				} else if (is_cross1) {
					const vec2& end_next1 = poly0[get_next_idx_in_ring(sz1, next_idx1, step1)];
					if (is_turn_left(end_next1, end1, end0)) { 
						return true; 
					}
				} else if (!is_two_points_same(cross, start0) 
					&& !is_two_points_same(cross, start1)) {
						return true;
				}
			}
			idx1 = next_idx1;
		}
		idx0 = next_idx0;
	}

	for (int i = 0; i < sz0; ++i) {
		if (is_point_intersect_polyline(poly0[i], poly1)) { continue; }
		return is_point_in_area(poly0[i], poly1);
	}
	for (int i = 0; i < sz1; ++i) {
		if (is_point_intersect_polyline(poly1[i], poly0)) { continue; }
		return is_point_in_area(poly1[i], poly0);
	}
	return false;
}

bool is_polygon_in_polygon(const CU_VEC<vec2>& in, const CU_VEC<vec2>& out)
{
	if (in.size() < 3 || out.size() < 3) {
		return false;
	}

	int sz0 = (int)in.size(),
		sz1 = (int)out.size();
	for (int i = 0; i < sz0; ++i) {
		if (is_point_intersect_polyline(in[i], out)) { continue; }
		if (!is_point_in_area(in[i], out)) {
			return false;
		} else {
			break;
		}
	}

	int step0 = is_polygon_colckwise(in) ? 1 : -1,
		step1 = is_polygon_colckwise(out) ? 1 : -1;
	int idx0 = 0;
	for (int i = 0; i < sz0; ++i) {
		int idx1 = 0;
		const vec2& start0 = in[idx0];
		int next_idx0 = get_next_idx_in_ring(sz0, idx0, step0);
		const vec2& end0 = in[next_idx0];
		for (int j = 0; j < sz1; ++j) {
			const vec2& start1 = out[idx1];
			int next_idx1 = get_next_idx_in_ring(sz1, idx1, step1);
			const vec2& end1 = out[next_idx1];

			vec2 cross;
			if (intersect_segment_segment(start0, end0, start1, end1, &cross)) {
				// test if cross is end
				bool is_cross0 = is_two_points_same(cross, end0),
					is_cross1 = is_two_points_same(cross, end1);
				if (is_cross0 && is_cross1) {
					const vec2& end_next0 = in[get_next_idx_in_ring(sz0, next_idx0, step0)];
					const vec2& end_next1 = out[get_next_idx_in_ring(sz1, next_idx1, step1)];
					float angle0 = get_angle(end0, end_next0, start0);

					float angle_start1 = get_angle(end0, end_next0, start1),
						angle_end_next1 = get_angle(end0, end_next0, end_next1);
					if ((angle0 > angle_start1 && angle_start1) != 0 ||
						(angle0 > angle_end_next1 && angle_end_next1 != 0)) {
							return false;
					}
				} else if (is_cross0) {
					const vec2& end_next0 = in[get_next_idx_in_ring(sz0, next_idx0, step0)];
					if (is_turn_left(end_next0, end0, end1)) { 
						return false; 
					}
				} else if (is_cross1) {
					const vec2& end_next1 = in[get_next_idx_in_ring(sz1, next_idx1, step1)];
					if (is_turn_left(end_next1, end1, end0)) { 
						return false; 
					}
				} else if (!is_two_points_same(cross, start0) 
					&& !is_two_points_same(cross, start1)) {
						return false;
				}
			}
			idx1 = next_idx1;
		}
		idx0 = next_idx0;
	}

	return true;	
}

}