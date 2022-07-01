#include "SM_Polygon.h"

#include <clipper/clipper.hpp>

namespace
{

const float SCALE = 10000.0f;

ClipperLib::Path create_path(const std::vector<sm::vec2>& poly)
{
	ClipperLib::Path path;

	path.resize(poly.size());
	for (size_t i = 0, n = poly.size(); i < n; ++i)
	{
		path[i].X = static_cast<int>(poly[i].x * SCALE);
		path[i].Y = static_cast<int>(poly[i].y * SCALE);
	}

	return path;
}

std::vector<sm::vec2> parser_path(const ClipperLib::Path& path)
{
	std::vector<sm::vec2> poly;

	poly.resize(path.size());
	for (size_t i = 0, n = path.size(); i < n; ++i)
	{
		poly[i].x = path[i].X / SCALE;
		poly[i].y = path[i].Y / SCALE;
	}

	return poly;
}

std::vector<std::vector<sm::vec2>>
do_clipper(const std::vector<std::vector<sm::vec2>>& subject, const std::vector<sm::vec2>& clip, ClipperLib::ClipType type)
{
	ClipperLib::Paths sub_paths, clp_paths;
	ClipperLib::Paths sol;

	sub_paths.resize(subject.size());
	for (size_t i = 0, n = subject.size(); i < n; ++i) {
		sub_paths[i] = create_path(subject[i]);
	}

	clp_paths.push_back(create_path(clip));

	ClipperLib::Clipper clipper;

	clipper.AddPaths(sub_paths, ClipperLib::ptSubject, true);
	clipper.AddPaths(clp_paths, ClipperLib::ptClip, true);

	clipper.Execute(type, sol, ClipperLib::pftEvenOdd);	

	std::vector<std::vector<sm::vec2>> result;
	result.resize(sol.size());
	for (size_t i = 0, n = sol.size(); i < n; ++i) {
		result[i] = parser_path(sol[i]);
	}

	return result;
}

}

namespace sm
{

std::vector<std::vector<vec2>> 
polygon_intersection(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip)
{
	return do_clipper(subject, clip, ClipperLib::ctIntersection);
}

std::vector<std::vector<vec2>>
polygon_union(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip)
{
	return do_clipper(subject, clip, ClipperLib::ctUnion);
}

std::vector<std::vector<vec2>>
polygon_difference(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip)
{
	return do_clipper(subject, clip, ClipperLib::ctDifference);
}

std::vector<std::vector<vec2>>
polygon_xor(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip)
{
	return do_clipper(subject, clip, ClipperLib::ctXor);
}

}