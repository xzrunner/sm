#include "SM_Polyline.h"
#include "SM_Test.h"

#include <cavc/polylineoffset.hpp>

namespace sm
{

std::vector<std::vector<vec2>> 
polyline_offset(const std::vector<vec2>& polyline, float distance, bool is_closed)
{
	cavc::Polyline<float> input;
	input.isClosed() = is_closed;

	const size_t n = (is_closed && polyline.front() == polyline.back()) ? 
		polyline.size() - 1 : polyline.size();
	for (int i = 0; i < n; ++i) {
		input.addVertex(polyline[i].x, polyline[i].y, 0.0f);
	}

	auto results = cavc::parallelOffset(input, distance);

	std::vector<std::vector<vec2>> polylines;
	for (auto& src : results)
	{
		std::vector<vec2> dst;
		for (auto& pos : src.vertexes()) {
			dst.push_back(vec2(pos.x(), pos.y()));
		}
		polylines.push_back(dst);
	}

	return polylines;
}

std::vector<std::vector<vec2>>
polyline_expand(const std::vector<vec2>& polyline, float distance, bool is_closed)
{
	std::vector<std::vector<vec2>> ret;

	if (is_closed)
	{
		std::vector<std::vector<vec2>> border, holes;
		if (!is_polygon_clockwise(polyline))
		{
			auto fixed = polyline;
			std::reverse(fixed.begin(), fixed.end());
			border = polyline_offset(fixed, distance, is_closed);
			holes = polyline_offset(fixed, -distance, is_closed);
		}
		else
		{
			border = polyline_offset(polyline, distance, is_closed);
			holes = polyline_offset(polyline, -distance, is_closed);
		}

		assert(border.size() == 1);
		ret.push_back(border[0]);

		for (auto& hole : holes) {
			ret.push_back(hole);
		}
	}
	else
	{
		auto off0 = polyline_offset(polyline, distance, is_closed);
		auto off1 = polyline_offset(polyline, -distance, is_closed);

		if (off0.size() != off1.size() || off0.empty()) {
			return ret;
		}

		for (int i = 0, n = off0.size(); i < n; ++i)
		{
			auto off(off0[i]);
			std::copy(off1[i].rbegin(), off1[i].rend(), std::back_inserter(off));
			ret.push_back(off);
		}
	}

	return ret;
}

}