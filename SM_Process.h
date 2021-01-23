#ifndef _SPATIAL_MATH_PROCESS_H_
#define _SPATIAL_MATH_PROCESS_H_

#include "SM_Vector.h"

#include <vector>

namespace sm
{

class Matrix2D;
class MatrixFix;

void rm_duplicate_nodes(const std::vector<vec2>& src, std::vector<vec2>& dst);

void trans_vertices(const Matrix2D& mt, const std::vector<vec2>& src, std::vector<vec2>& dst);
void trans_vertices(const MatrixFix& mt, const std::vector<vec2>& src, std::vector<vec2>& dst);

}

#include "SM_Process.inl"

#endif // _SPATIAL_MATH_PROCESS_H_