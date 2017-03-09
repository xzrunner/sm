#include "SM_Triangulation.h"
#include "SM_Calc.h"
#include "SM_Test.h"

#include "external/triangle.h"

#include <assert.h>

namespace sm
{

static void init(struct triangulateio& in, struct triangulateio& out)
{
	in.pointlist = NULL;
	in.pointattributelist = NULL;
	in.pointmarkerlist = NULL;
	in.trianglelist = NULL;
	in.triangleattributelist = NULL;
	in.trianglearealist = NULL;
	in.segmentlist = NULL;
	in.segmentmarkerlist = NULL;
	in.holelist = NULL;
	in.regionlist = NULL;

	out.pointlist = NULL;
	out.pointattributelist = NULL;
	out.pointmarkerlist = NULL;
	out.trianglelist = NULL;
	out.triangleattributelist = NULL;
	out.trianglearealist = NULL;
	out.neighborlist = NULL;
	out.segmentlist = NULL;
	out.segmentmarkerlist = NULL;
	out.edgelist = NULL;
	out.edgemarkerlist = NULL;
	out.normlist = NULL;
}

static void implement(struct triangulateio& in, struct triangulateio& out, TriangulateConstrained tc)
{
	switch (tc)
	{
	case TC_CONSTRAINED:
		triangulate("pz", &in, &out, (struct triangulateio *) NULL);
		break;
	case TC_CONFORMING:
		triangulate("pzD", &in, &out, (struct triangulateio *) NULL);
		break;
	case TC_CONSTRAINED_CONFORMING_ANGLE:
		triangulate("pzq", &in, &out, (struct triangulateio *) NULL);
		break;
	case TC_CONSTRAINED_CONFORMING_AREA:
		triangulate("pza10000", &in, &out, (struct triangulateio *) NULL);
		break;
	case TC_CONSTRAINED_CONFORMING_COUNT:
		triangulate("pzu100", &in, &out, (struct triangulateio *) NULL);
		break;
	default:
		assert(0);
	}
}

static void finish(struct triangulateio& in, struct triangulateio& out)
{
	trifree((void*)in.pointlist);
	trifree((void*)in.pointattributelist);
	trifree((void*)in.pointmarkerlist);
	trifree((void*)in.trianglelist);
	trifree((void*)in.triangleattributelist);
	trifree((void*)in.trianglearealist);
	trifree((void*)in.segmentlist);
	trifree((void*)in.segmentmarkerlist);
	trifree((void*)in.holelist);
	trifree((void*)in.regionlist);

	trifree((void*)out.pointlist);
	trifree((void*)out.pointattributelist);
	trifree((void*)out.pointmarkerlist);
	trifree((void*)out.trianglelist);
	trifree((void*)out.triangleattributelist);
	trifree((void*)out.trianglearealist);
	trifree((void*)out.neighborlist);
	trifree((void*)out.segmentlist);
	trifree((void*)out.segmentmarkerlist);
	trifree((void*)out.edgelist);
	trifree((void*)out.edgemarkerlist);
	trifree((void*)out.normlist);
}

static void finish(struct triangulateio& in, 
				   struct triangulateio& out, 
				   const std::vector<vec2>& bound, 
				   std::vector<vec2>& result)
{
	int index = 0;
	for (int i = 0; i < out.numberoftriangles; ++i)
	{
		std::vector<vec2> tri;
		for (int j = 0; j < out.numberofcorners; ++j)
		{
			int pIndex = out.trianglelist[index++];

			vec2 p;
			p.x = out.pointlist[pIndex * 2];
			p.y = out.pointlist[pIndex * 2 + 1];
			tri.push_back(p);
		}

		vec2 center = get_tri_gravity_center(tri[0], tri[1], tri[2]);
		if (is_point_in_area(center, bound)) {
			copy(tri.begin(), tri.end(), back_inserter(result));
		}
	}

	finish(in, out);
}

static void finish(struct triangulateio& in, 
                   struct triangulateio& out, 
				   const std::vector<vec2>& bound, 
				   std::vector<vec2>& out_vertices,
				   std::vector<int>& out_triangles)
{
	out_vertices.reserve(out.numberofpoints);
	int ptr = 0;
	for (int i = 0; i < out.numberofpoints; ++i) 
	{
		float x = out.pointlist[ptr++],
			  y = out.pointlist[ptr++];
		out_vertices.push_back(vec2(x, y));
	}

	int index = 0;
	out_triangles.reserve(out.numberoftriangles);
	for (int i = 0; i < out.numberoftriangles; ++i) 
	{
		std::vector<int> tri;
		for (int j = 0; j < out.numberofcorners; ++j) {
			tri.push_back(out.trianglelist[index++]);
		}

		vec2 center = get_tri_gravity_center(out_vertices[tri[0]], out_vertices[tri[1]], out_vertices[tri[2]]);
		if (is_point_in_area(center, bound)) {
			copy(tri.begin(), tri.end(), back_inserter(out_triangles));
		}
	}

	finish(in, out);
}

static void verify_bound(const std::vector<vec2>& src, std::vector<vec2>& dst)
{
   for (int i = 0, n = src.size(); i < n; ++i)
   {
	   bool skip = false;
	   for (int j = 0, m = dst.size(); j < m; ++j) {
		   if (src[i] == dst[j]) {
			   skip = true;
			   break;
		   }
	   }
	   if (skip) {
		   continue;
	   }
	   dst.push_back(src[i]);
   }
}

static void verify_inner(const std::vector<vec2>& outer, std::vector<vec2>& inner)
{
	std::vector<vec2>::iterator itr = inner.begin();
	for ( ; itr != inner.end(); ) 
	{
		bool find = false;
		for (int i = 0, n = outer.size(); i < n; ++i) {
			if (*itr == outer[i]) {
				find = true;
				itr = inner.erase(itr);
				break;
			}
		}
		if (!find) {
			++itr;
		}
	}
}

void triangulate_normal(const std::vector<vec2>& bound, 
						std::vector<vec2>& result, 
						TriangulateConstrained tc)
{
	struct triangulateio in, out;
	init(in, out);

	std::vector<vec2> bound_fixed;
	verify_bound(bound, bound_fixed);

	in.numberofpoints = bound_fixed.size();
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
	in.pointmarkerlist = (int *) NULL;
	int index = 0;
	for (int i = 0; i < in.numberofpoints; ++i)
	{
		in.pointlist[index++] = bound_fixed[i].x;
		in.pointlist[index++] = bound_fixed[i].y;
	}

	in.numberofsegments = in.numberofpoints;
	in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
	index = 0;
	for (int i = 0; i < in.numberofsegments - 1; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = in.numberofsegments - 1;
	in.segmentlist[index++] = 0;

	in.segmentmarkerlist = (int *) NULL;

	in.numberofholes = 0;
	in.numberofregions = 0;

	implement(in, out, tc);
	finish(in, out, bound_fixed, result);
}

void triangulate_holes(const std::vector<vec2>& bound, 
					   const std::vector<std::vector<vec2> >& holes,
					   std::vector<vec2>& result, 
					   TriangulateConstrained tc)
{
	if (!holes.empty()) {
		return triangulate_holes_new(bound, holes[0], result, tc);
	}

	struct triangulateio in, out;
	init(in, out);

	std::vector<vec2> bound_fixed;
	verify_bound(bound, bound_fixed);

	in.numberofpoints = bound_fixed.size();
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
	in.pointmarkerlist = (int *) NULL;
	int index = 0;
	for (int i = 0; i < in.numberofpoints; ++i)
	{
		in.pointlist[index++] = bound_fixed[i].x;
		in.pointlist[index++] = bound_fixed[i].y;
	}

	in.numberofsegments = in.numberofpoints;
	in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
	index = 0;
	for (int i = 0; i < in.numberofsegments - 1; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = in.numberofsegments - 1;
	in.segmentlist[index++] = 0;

	in.segmentmarkerlist = (int *) NULL;

	//in.numberofholes = 0;

	in.numberofholes = 1;
	in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));
	in.holelist[0] = 0;
	in.holelist[1] = 0;

	// 	std::vector<vec2> hole_fixed;
	// 	verify_bound(holes[0], hole_fixed);
	// 	in.numberofholes = hole_fixed.size();
	// 	in.holelist = (REAL*)malloc(in.numberofholes * 2 * sizeof(REAL));
	// 	index = 0;
	// 	for (int i = 0; i < in.numberofholes; ++i)
	// 	{
	// 		in.holelist[index++] = hole_fixed[i].x;
	// 		in.holelist[index++] = hole_fixed[i].y;
	// 	}

	in.numberofregions = 0;

	implement(in, out, tc);
	finish(in, out, bound_fixed, result);
}

void triangulate_holes_new(const std::vector<vec2>& bound, 
						   const std::vector<vec2>& hole,
						   std::vector<vec2>& result, 
						   TriangulateConstrained tc)
{
	struct triangulateio in, out;
	init(in, out);

	std::vector<vec2> bound_fixed, hole_fixed;
	verify_bound(bound, bound_fixed);
	verify_bound(hole, hole_fixed);

	in.numberofpoints = bound_fixed.size() + hole_fixed.size();
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
	in.pointmarkerlist = (int *) NULL;
	int index = 0;
	for (int i = 0, n = bound_fixed.size(); i < n; ++i)
	{
		in.pointlist[index++] = bound_fixed[i].x;
		in.pointlist[index++] = bound_fixed[i].y;
	}
	vec2 hold_center;
	for (int i = 0, n = hole_fixed.size(); i < n; ++i)
	{
		in.pointlist[index++] = hole_fixed[i].x;
		in.pointlist[index++] = hole_fixed[i].y;

		hold_center += hole_fixed[i];
	}
	hold_center /= hole_fixed.size();

	in.numberofsegments = in.numberofpoints;
	in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
	index = 0;
	for (int i = 0; i < bound_fixed.size() - 1; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = bound_fixed.size() - 1;
	in.segmentlist[index++] = 0;

	for (int i = 0; i < hole_fixed.size() - 1; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = in.numberofpoints - 1;
	in.segmentlist[index++] = bound_fixed.size();

	in.segmentmarkerlist = (int *) NULL;

	// 	in.numberofholes = 0;
	// 	in.holelist = NULL;

	in.numberofholes = 1;
	in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));
	in.holelist[0] = hold_center.x;
	in.holelist[1] = hold_center.y;

	// 	std::vector<vec2> hole_fixed;
	// 	verify_bound(holes[0], hole_fixed);
	// 	in.numberofholes = hole_fixed.size();
	// 	in.holelist = (REAL*)malloc(in.numberofholes * 2 * sizeof(REAL));
	// 	index = 0;
	// 	for (int i = 0; i < in.numberofholes; ++i)
	// 	{
	// 		in.holelist[index++] = hole_fixed[i].x;
	// 		in.holelist[index++] = hole_fixed[i].y;
	// 	}

	in.numberofregions = 0;

	implement(in, out, tc);
	finish(in, out, bound_fixed, result);
}

static void triangulate_points_prepare(triangulateio& in, 
									   triangulateio& out, 
									   const std::vector<vec2>& bound,
									   const std::vector<vec2>& points)
{
	init(in, out);

	in.numberofpoints = bound.size() + points.size();
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
	in.pointmarkerlist = (int *) NULL;
	int index = 0;
	for (int i = 0, n = bound.size(); i < n; ++i)
	{
		in.pointlist[index++] = bound[i].x;
		in.pointlist[index++] = bound[i].y;
	}
	for (int i = 0, n = points.size(); i < n; ++i)
	{
		in.pointlist[index++] = points[i].x;
		in.pointlist[index++] = points[i].y;
	}

	in.numberofsegments = bound.size();
	in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));

	index = 0;
	for (int i = 0, n = bound.size() - 1; i < n; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = bound.size() - 1;
	in.segmentlist[index++] = 0;

	in.segmentmarkerlist = (int *) NULL;

	in.numberofholes = 0;
	in.numberofregions = 0;
}

void triangulate_points(const std::vector<vec2>& bound, 
						const std::vector<vec2>& points,
						std::vector<vec2>& result, 
						TriangulateConstrained tc)
{
	std::vector<vec2> bound_fixed;
	verify_bound(bound, bound_fixed);

	std::vector<vec2> points_fixed(points);
	verify_inner(bound_fixed, points_fixed);

	struct triangulateio in, out;
	triangulate_points_prepare(in, out, bound_fixed, points_fixed);

	implement(in, out, tc);

	finish(in, out, bound_fixed, result);
}

void triangulate_points(const std::vector<vec2>& bound, 
						const std::vector<vec2>& points,
						std::vector<vec2>& out_vertices,
						std::vector<int>& out_triangles,
						TriangulateConstrained tc)
{
	std::vector<vec2> bound_fixed;
	verify_bound(bound, bound_fixed);

	std::vector<vec2> points_fixed(points);
	verify_inner(bound_fixed, points_fixed);

	struct triangulateio in, out;
	triangulate_points_prepare(in, out, bound_fixed, points_fixed);

	implement(in, out, tc);

	finish(in, out, bound_fixed, out_vertices, out_triangles);
}

void triangulate_lines(const std::vector<vec2>& bound, 
					   const std::vector<vec2>& lines,
					   std::vector<vec2>& result, 
					   TriangulateConstrained tc)
{
	struct triangulateio in, out;
	init(in, out);

	std::vector<vec2> bound_fixed;
	verify_bound(bound, bound_fixed);

	in.numberofpoints = bound_fixed.size() + lines.size();
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
	in.pointmarkerlist = (int *) NULL;
	int index = 0;
	for (int i = 0, n = bound_fixed.size(); i < n; ++i)
	{
		in.pointlist[index++] = bound_fixed[i].x;
		in.pointlist[index++] = bound_fixed[i].y;
	}
	for (int i = 0, n = lines.size(); i < n; ++i)
	{
		in.pointlist[index++] = lines[i].x;
		in.pointlist[index++] = lines[i].y;
	}

	in.numberofsegments = bound_fixed.size() + lines.size() / 2;
	in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));

	index = 0;
	for (int i = 0, n = bound_fixed.size() - 1; i < n; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = bound_fixed.size() - 1;
	in.segmentlist[index++] = 0;

	int lineIndex = bound_fixed.size();
	for (int i = 0, n = lines.size() / 2; i < n; ++i)
	{
		in.segmentlist[index++] = lineIndex++;
		in.segmentlist[index++] = lineIndex++;
	}

	in.segmentmarkerlist = (int *) NULL;

	in.numberofholes = 0;
	in.numberofregions = 0;

	implement(in, out, tc);
	finish(in, out, bound_fixed, result);
}

void triangulate_points_and_lines(const std::vector<vec2>& bound, 
								  const std::vector<vec2>& points,
								  const std::vector<vec2>& lines, 
								  std::vector<vec2>& result, 
								  TriangulateConstrained tc)
{
	struct triangulateio in, out;
	init(in, out);

	std::vector<vec2> bound_fixed;
	verify_bound(bound, bound_fixed);

	std::vector<vec2> points_fixed(points);
	verify_inner(bound_fixed, points_fixed);

	in.numberofpoints = bound_fixed.size() + lines.size() + points_fixed.size();
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
	in.pointmarkerlist = (int *) NULL;
	int index = 0;
	for (int i = 0, n = bound_fixed.size(); i < n; ++i)
	{
		in.pointlist[index++] = bound_fixed[i].x;
		in.pointlist[index++] = bound_fixed[i].y;
	}
	for (int i = 0, n = lines.size(); i < n; ++i)
	{
		in.pointlist[index++] = lines[i].x;
		in.pointlist[index++] = lines[i].y;
	}
	for (int i = 0, n = points_fixed.size(); i < n; ++i)
	{
		in.pointlist[index++] = points_fixed[i].x;
		in.pointlist[index++] = points_fixed[i].y;
	}

	in.numberofsegments = bound_fixed.size() + lines.size() / 2;
	in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));

	index = 0;
	for (int i = 0, n = bound_fixed.size() - 1; i < n; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = bound_fixed.size() - 1;
	in.segmentlist[index++] = 0;

	int lineIndex = bound_fixed.size();
	for (int i = 0, n = lines.size() / 2; i < n; ++i)
	{
		in.segmentlist[index++] = lineIndex++;
		in.segmentlist[index++] = lineIndex++;
	}

	in.segmentmarkerlist = (int *) NULL;

	in.numberofholes = 0;
	in.numberofregions = 0;

	implement(in, out, tc);
	finish(in, out, bound_fixed, result);
}

void triangulate_lines_and_loops(const std::vector<vec2>& bound, 
								 const std::vector<vec2>& lines,
								 const std::vector<std::vector<vec2> >& loops, 
								 std::vector<vec2>& result, 
								 TriangulateConstrained tc)
{
	struct triangulateio in, out;
	init(in, out);

	int loopSize = 0;
	for (int i = 0, n = loops.size(); i < n; ++i)
		loopSize += loops[i].size();

	std::vector<vec2> bound_fixed;
	verify_bound(bound, bound_fixed);

	in.numberofpoints = bound_fixed.size() + lines.size() + loopSize;
	in.numberofpointattributes = 0;
	in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
	in.pointmarkerlist = (int *) NULL;
	int index = 0;
	for (int i = 0, n = bound_fixed.size(); i < n; ++i)
	{
		in.pointlist[index++] = bound_fixed[i].x;
		in.pointlist[index++] = bound_fixed[i].y;
	}
	for (int i = 0, n = lines.size(); i < n; ++i)
	{
		in.pointlist[index++] = lines[i].x;
		in.pointlist[index++] = lines[i].y;
	}
	for (int i = 0, n = loops.size(); i < n; ++i)
	{
		for (int j = 0, m = loops[i].size(); j < m; ++j)
		{
			in.pointlist[index++] = loops[i][j].x;
			in.pointlist[index++] = loops[i][j].y;
		}
	}

	in.numberofsegments = bound_fixed.size() + lines.size() / 2 + loopSize;
	in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));

	index = 0;
	for (int i = 0, n = bound_fixed.size() - 1; i < n; ++i)
	{
		in.segmentlist[index++] = i;
		in.segmentlist[index++] = i + 1;
	}
	in.segmentlist[index++] = bound_fixed.size() - 1;
	in.segmentlist[index++] = 0;

	int lineIndex = bound_fixed.size();
	for (int i = 0, n = lines.size() / 2; i < n; ++i)
	{
		in.segmentlist[index++] = lineIndex++;
		in.segmentlist[index++] = lineIndex++;
	}

	int loopIndex = bound_fixed.size() + lines.size();
	for (int i = 0, n = loops.size(); i < n; ++i)
	{
		int start = loopIndex;
		for (int j = 0, m = loops[i].size() - 1; j < m; ++j)
		{
			in.segmentlist[index++] = loopIndex;
			in.segmentlist[index++] = loopIndex + 1;
			++loopIndex;
		}
		in.segmentlist[index++] = loopIndex;
		in.segmentlist[index++] = start;
		++loopIndex;
	}

	in.segmentmarkerlist = (int *) NULL;

	in.numberofholes = 0;
	in.numberofregions = 0;

	implement(in, out, tc);
	finish(in, out, bound_fixed, result);
}

}