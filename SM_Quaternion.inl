namespace sm
{

template <typename T>
QuaternionT<T>::QuaternionT()
	: x(0)
	, y(0)
	, z(0)
	, w(1)
{
}

template <typename T>
QuaternionT<T>::QuaternionT(T x, T y, T z)
{
	QuaternionT<T> roll(sinf( x * 0.5f ), 0, 0, cosf( x * 0.5f )),
		           pitch(0, sinf( y * 0.5f ), 0, cosf( y * 0.5f )),
				   yaw(0, 0, sinf( z * 0.5f ), cosf( z * 0.5f ));
	// Order: y * x * z
	*this = pitch * roll * yaw;
}

template <typename T>
QuaternionT<T>::QuaternionT(T x, T y, T z, T w)
	: x(x)
	, y(y)
	, z(z)
	, w(w)
{
}

template <typename T>
bool QuaternionT<T>::operator == (const QuaternionT<T>& q) const
{
	return x == q.x && y == q.y && z == q.z && w == q.w;
}

template <typename T>
bool QuaternionT<T>::operator != (const QuaternionT<T>& q) const
{
	return !(*this == q);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator - () const
{
	return QuaternionT<T>(-x, -y, -z, -w);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator + (const QuaternionT<T>& q) const
{
	return QuaternionT<T>(x + q.x, y + q.y, z + q.z, w + q.w);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator - (const QuaternionT<T>& q) const
{
	return QuaternionT<T>(x - q.x, y - q.y, z - q.z, w - q.w);
}

template <typename T>
QuaternionT<T> QuaternionT<T>::operator * (const QuaternionT<T>& q) const
{
	T x = y * q.z - z * q.y + q.x * w + x * q.w;
	T y = z * q.x - x * q.z + q.y * w + y * q.w;
	T z = x * q.y - y * q.x + q.z * w + z * q.w;
	T w = w * q.w - (x * q.x + y * q.y + z * q.z);
	return QuaternionT<T>(x, y, z, w);
}

template <typename T>
void QuaternionT<T>::Normalize()
{
	*this = Scaled(1 / sqrt(Dot(*this)));
}

template <typename T>
void QuaternionT<T>::Slerp(const QuaternionT<T>& a, const QuaternionT<T>& b, float t)
{
	T cos_theta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	if (cos_theta < 0) {
		cos_theta = -cos_theta; 
		x = -b.x; y = -b.y;
		z = -b.z; w = -b.w;
	} else {
		*this = *b;
	}
	T scale0 = 1 - t, scale1 = t;
	if( (1 - cos_theta) > 0.001f ) {
		// use spherical interpolation
		T theta = acosf( cos_theta );
		T sin_theta = sinf( theta );
		scale0 = sinf( (1 - t) * theta ) / sin_theta;
		scale1 = sinf( t * theta ) / sin_theta;
	}

	x = a.x * scale0 + x * scale1;
	y = a.y * scale0 + y * scale1;
	z = a.z * scale0 + z * scale1;
	w = a.w * scale0 + w * scale1;

}

template <typename T>
void QuaternionT<T>::NSlerp(const QuaternionT<T>& a, const QuaternionT<T>& b, float t)
{
	// Normalized linear sm_quaternion interpolation
	// Note: NLERP is faster than SLERP and commutative but does not yield constant velocity

	T cos_theta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

	if( cos_theta < 0 ) {
		x = a.x + (-b.x - a.x) * t;
		y = a.y + (-b.y - a.y) * t;
		z = a.z + (-b.z - a.z) * t;
		w = a.w + (-b.w - a.w) * t;
	} else {
		x = a.x + (b.x - a.x) * t;
		y = a.y + (b.y - a.y) * t;
		z = a.z + (b.z - a.z) * t;
		w = a.w + (b.w - a.w) * t;
	}

	T inv_len = 1.0f / sqrtf( x * x + y * y + z * z + w * w );
	x *= inv_len;
	y *= inv_len;
	z *= inv_len;
	w *= inv_len;
}

template <typename T>
void QuaternionT<T>::Inverted()
{
	float len = x * x + y * y + z * z + w * w;
	if( len > 0 ) 
	{
		float invLen = - 1.0f / len;
		x *= invLen;
		y *= invLen;
		z *= invLen;
		w *= invLen;
		w = -w;
	} 
	else 
	{
		x = y = z = w = 0;
	}
}

template <typename T>
void QuaternionT<T>::Rotate(const QuaternionT<T>& q)
{
	w = w * q.w - x * q.x - y * q.y - z * q.z;
	x = w * q.x + x * q.w + y * q.z - z * q.y;
	y = w * q.y + y * q.w + z * q.x - x * q.z;
	z = w * q.z + z * q.w + x * q.y - y * q.x;
	Normalize();
}

template <typename T>
void QuaternionT<T>::Scale(T scale)
{

}

}