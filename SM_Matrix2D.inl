#include "SM_Matrix2D.h"

namespace sm
{

inline
void Matrix2D::Identity()
{
	x[0] = 1;
	x[1] = 0;
	x[2] = 0;
	x[3] = 1;
	x[4] = 0;
	x[5] = 0;
}

}