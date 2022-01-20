#pragma once

#include "SM_Vector.h"

#include <vector>

namespace sm
{

std::vector<std::vector<vec2>> 
polygon_intersection(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip);

std::vector<std::vector<vec2>>
polygon_union(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip);

std::vector<std::vector<vec2>>
polygon_difference(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip);

std::vector<std::vector<vec2>>
polygon_xor(const std::vector<std::vector<vec2>>& subject, const std::vector<vec2>& clip);

}