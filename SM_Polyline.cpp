#include "SM_Polyline.h"

#include <cavc/polylineoffset.hpp>

namespace sm
{

std::vector<std::vector<vec2>> 
polyline_offset(const std::vector<vec2>& polyline, float distance, bool is_closed)
{
	cavc::Polyline<float> input;
	input.isClosed() = is_closed;

	for (int i = 0, n = polyline.size(); i < n; ++i) {
		input.addVertex(polyline[i].x, polyline[i].y, 0.0f);
	}

	auto results = cavc::parallelOffset(input, distance);

	std::vector<std::vector<vec2>> polylines;
	for (auto& src : results)
	{
		std::vector<vec2> dst;
		for (auto& pos : src.vertexes()) {
			dst.push_back(sm::vec2(pos.x(), pos.y()));
		}
		polylines.push_back(dst);
	}

	return polylines;
}

std::vector<std::vector<vec2>>
polyline_expand(const std::vector<vec2>& polyline, float offste, bool is_closed)
{
	std::vector<std::vector<sm::vec2>> ret;

	auto off0 = polyline_offset(polyline, offste, is_closed);
	auto off1 = polyline_offset(polyline, -offste, is_closed);

	if (off0.size() != off1.size() || off0.empty()) {
		return ret;
	}

	for (int i = 0, n = off0.size(); i < n; ++i)
	{
		auto off(off0[i]);
		std::copy(off1[i].rbegin(), off1[i].rend(), std::back_inserter(off));
		ret.push_back(off);
	}

	return ret;
}

}