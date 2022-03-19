#include "SM_Math.h"

#include <string.h>

namespace
{

float i2f(unsigned int i)
{
	float ret;
	memcpy(&ret, &i, sizeof(float));
	return ret;
}

}

namespace sm
{

// from glm
float f16_to_f32(unsigned short value)
{
	int s = (value >> 15) & 0x00000001;
	int e = (value >> 10) & 0x0000001f;
	int m =  value        & 0x000003ff;

	if(e == 0)
	{
		if(m == 0)
		{
			//
			// Plus or minus zero
			//

			return i2f(static_cast<unsigned int>(s << 31));
		}
		else
		{
			//
			// Denormalized number -- renormalize it
			//

			while(!(m & 0x00000400))
			{
				m <<= 1;
				e -=  1;
			}

			e += 1;
			m &= ~0x00000400;
		}
	}
	else if(e == 31)
	{
		if(m == 0)
		{
			//
			// Positive or negative infinity
			//

			return i2f(static_cast<unsigned int>((s << 31) | 0x7f800000));
		}
		else
		{
			//
			// Nan -- preserve sign and significand bits
			//

			return i2f(static_cast<unsigned int>((s << 31) | 0x7f800000 | (m << 13)));
		}
	}

	//
	// Normalized number
	//

	e = e + (127 - 15);
	m = m << 13;

	//
	// Assemble s, e and m.
	//

	return i2f(static_cast<unsigned int>((s << 31) | (e << 23) | m));
}

}