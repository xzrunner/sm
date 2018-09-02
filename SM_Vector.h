#ifndef _SPATIAL_MATH_VECTOR_H_
#define _SPATIAL_MATH_VECTOR_H_

#include <float.h>
#include <stdint.h>

namespace sm
{

/**
 *  @brief
 *    vector2
 */
template <typename T>
class Vector2
{
public:
	union
	{
		struct
		{
			T x;
			T y;
		};

		T xy[2];
	};

public:
	Vector2();
	Vector2(T x, T y);
	Vector2(const Vector2& v);

	Vector2& operator = (const Vector2& v);
	void Set(T x, T y);

	void MakeInvalid();
	bool IsValid() const;

	bool operator == (const Vector2& v) const;
	bool operator != (const Vector2& v) const;

	Vector2<T> operator - () const;

	void operator += (const Vector2& v);
	void operator -= (const Vector2& v);
	void operator *= (const Vector2& v);
	void operator /= (const Vector2& v);
	void operator *= (T f);
	void operator /= (T f);

	Vector2 operator + (const Vector2& v) const;
	Vector2 operator - (const Vector2& v) const;
	Vector2 operator * (const Vector2& v) const;
	Vector2 operator / (const Vector2& v) const;
	Vector2 operator * (T f) const;
	Vector2 operator / (T f) const;

	T Length() const;
	T LengthSquared() const;
	void Normalize();
	Vector2 Normalized() const;

	T Cross(const Vector2& v) const;
	T Dot(const Vector2& v) const;

}; // Vector2

class Vector2Cmp
{
public:
	template <typename T>
	bool operator () (const Vector2<T> & p0, const Vector2<T> & p1) const;
}; // Vector2Cmp

class Vector2CmpX
{
public:
	template <typename T>
	bool operator () (const Vector2<T> & p0, const Vector2<T> & p1) const;
}; // Vector2CmpX

class Vector2CmpY
{
public:
	template <typename T>
	bool operator () (const Vector2<T> & p0, const Vector2<T> & p1) const;
}; // Vector2CmpY

/**
 *  @brief
 *    vector3
 */
template <typename T>
class Vector3
{
public:
	union
	{
		struct
		{
			T x;
			T y;
			T z;
		};

		T xyz[3];
	};

public:
	Vector3();
	Vector3(const T* xyz);
	Vector3(T x, T y, T z);
	Vector3(const Vector3& v);

	T operator[](size_t i) const;
	T& operator[](size_t i);

	Vector3& operator = (const Vector3& v);
	void Set(T x, T y, T z);

	void MakeInvalid();
	bool IsValid() const;

	bool operator == (const Vector3& v) const;
	bool operator != (const Vector3& v) const;

	Vector3 operator - () const;

	void operator += (const Vector3& v);
	void operator -= (const Vector3& v);
	void operator *= (T f);
	void operator /= (T f);

	Vector3 operator + (const Vector3& v) const;
	Vector3 operator - (const Vector3& v) const;
	Vector3 operator * (T f) const;
	Vector3 operator / (T f) const;

	T Length() const;
	T LengthSquared() const;
	void Normalize();
	Vector3 Normalized() const;

	Vector3 Cross(const Vector3& v) const;
	T Dot(const Vector3& v) const;

}; // Vector3

class Vector3Cmp
{
public:
	template <typename T>
	bool operator () (const Vector3<T> & p0, const Vector3<T> & p1) const;
}; // Vector3Cmp

/**
 *  @brief
 *    vector4
 */
template <typename T>
class Vector4
{
public:
	union
	{
		struct
		{
			T x;
			T y;
			T z;
			T w;
		};

		T xyzw[4];
	};

public:
	Vector4();
	Vector4(T x, T y, T z, T w);
	Vector4(const Vector4& v);

	T operator[](size_t i) const;
	T& operator[](size_t i);

	Vector4& operator = (const Vector4& v);
	void Assign(T x, T y, T z, T w);

	bool operator == (const Vector4& v) const;
	bool operator != (const Vector4& v) const;

	void operator += (const Vector4& v);
	void operator -= (const Vector4& v);
	void operator *= (T f);
	void operator /= (T f);

	Vector4 operator + (const Vector4& v) const;
	Vector4 operator - (const Vector4& v) const;
	Vector4 operator * (T f) const;
	Vector4 operator / (T f) const;

	T Dot(const Vector4& v) const;

}; // Vector4

typedef Vector2<bool> bvec2;

typedef Vector2<uint16_t> i16_vec2;

typedef Vector2<int> ivec2;
typedef Vector3<int> ivec3;
typedef Vector4<int> ivec4;

typedef Vector2<float> vec2;
typedef Vector3<float> vec3;
typedef Vector4<float> vec4;

}

#include "SM_Vector.inl"

#endif // _SPATIAL_MATH_VECTOR_H_
