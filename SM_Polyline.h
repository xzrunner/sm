#pragma once

#include "SM_Vector.h"

#include <vector>

namespace sm
{

std::vector<std::vector<vec2>> 
polyline_offset(const std::vector<vec2>& polyline, float distance, bool is_closed);

std::vector<std::vector<vec2>>
polyline_expand(const std::vector<vec2>& polyline, float distance, bool is_closed);

}