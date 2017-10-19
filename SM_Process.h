#ifndef _SPATIAL_MATH_PROCESS_H_
#define _SPATIAL_MATH_PROCESS_H_

#include "SM_Vector.h"

#include <cu/cu_stl.h>

namespace sm
{

class Matrix2D;
class MatrixFix;

void rm_duplicate_nodes(const CU_VEC<vec2>& src, CU_VEC<vec2>& dst);

void trans_vertices(const Matrix2D& mt, const CU_VEC<vec2>& src, CU_VEC<vec2>& dst);
void trans_vertices(const MatrixFix& mt, const CU_VEC<vec2>& src, CU_VEC<vec2>& dst);

}

#include "SM_Process.inl"

#endif // _SPATIAL_MATH_PROCESS_H_