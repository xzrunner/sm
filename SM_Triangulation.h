#ifndef _SPATIAL_MATH_TRIANGULATION_H_
#define _SPATIAL_MATH_TRIANGULATION_H_

#include "SM_Vector.h"

#include <cu/cu_stl.h>

namespace sm
{

enum TriangulateConstrained
{
	TC_CONSTRAINED,
	TC_CONFORMING,
	TC_CONSTRAINED_CONFORMING_ANGLE,
	TC_CONSTRAINED_CONFORMING_AREA,
	TC_CONSTRAINED_CONFORMING_COUNT
};

void triangulate_normal(const CU_VEC<vec2>& bound,
						CU_VEC<vec2>& result,
						TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_holes(const CU_VEC<vec2>& bound,
					   const CU_VEC<CU_VEC<vec2>>& holes,
					   CU_VEC<vec2>& result,
					   TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_points(const CU_VEC<vec2>& bound,
						const CU_VEC<vec2>& points,
						CU_VEC<vec2>& result,
						TriangulateConstrained tc = TC_CONSTRAINED);
void triangulate_points(const CU_VEC<vec2>& bound,
						const CU_VEC<vec2>& points,
						CU_VEC<vec2>& out_vertices,
						CU_VEC<int>& out_triangles,
						TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_lines(const CU_VEC<vec2>& bound,
					   const CU_VEC<vec2>& lines,
					   CU_VEC<vec2>& result,
					   TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_points_and_lines(const CU_VEC<vec2>& bound,
								  const CU_VEC<vec2>& points,
								  const CU_VEC<vec2>& lines,
								  CU_VEC<vec2>& result,
								  TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_lines_and_loops(const CU_VEC<vec2>& bound,
								 const CU_VEC<vec2>& lines,
								 const CU_VEC<CU_VEC<vec2> >& loops,
								 CU_VEC<vec2>& result,
								 TriangulateConstrained tc = TC_CONSTRAINED);

}

#endif // _SPATIAL_MATH_TRIANGULATION_H_