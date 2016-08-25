#ifndef _SPATIAL_MATH_TRIANGULATION_H_
#define _SPATIAL_MATH_TRIANGULATION_H_

#include "SM_Vector.h"

#include <vector>

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

void triangulate_normal(const std::vector<vec2>& bound, 
						std::vector<vec2>& result, 
						TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_holes(const std::vector<vec2>& bound, 
					   const std::vector<std::vector<vec2> >& holes,
					   std::vector<vec2>& result, 
					   TriangulateConstrained tc = TC_CONSTRAINED);
void triangulate_holes_new(const std::vector<vec2>& bound, 
						   const std::vector<vec2>& hole,
						   std::vector<vec2>& result, 
						   TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_points(const std::vector<vec2>& bound, 
						const std::vector<vec2>& points,
						std::vector<vec2>& result, 
						TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_lines(const std::vector<vec2>& bound, 
					   const std::vector<vec2>& lines, 
					   std::vector<vec2>& result, 
					   TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_points_and_lines(const std::vector<vec2>& bound, 
								  const std::vector<vec2>& points,
								  const std::vector<vec2>& lines, 
								  std::vector<vec2>& result, 
								  TriangulateConstrained tc = TC_CONSTRAINED);

void triangulate_lines_and_loops(const std::vector<vec2>& bound, 
								 const std::vector<vec2>& lines,
								 const std::vector<std::vector<vec2> >& loops, 
								 std::vector<vec2>& result, 
								 TriangulateConstrained tc = TC_CONSTRAINED);

}

#endif // _SPATIAL_MATH_TRIANGULATION_H_