#ifndef _SPATIAL_MATH_VECTOR_H_
#define _SPATIAL_MATH_VECTOR_H_

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

	bool operator != (const Vector2& v) const;
	bool operator == (const Vector2& v) const;

	Vector2<T> operator - () const;

	void operator += (const Vector2& v);
	void operator -= (const Vector2& v);
	void operator *= (T f);
	void operator /= (T f);

	Vector2 operator + (const Vector2& v) const;
	Vector2 operator - (const Vector2& v) const;
	Vector2 operator * (T f) const;
	Vector2 operator / (T f) const;

	T Length() const;
	T LengthSquared() const;
	void Normalize();
	Vector2 Normalized() const;

	T Cross(const Vector2& v) const;
	T Dot(const Vector2& v) const;

}; // Vector2

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
	Vector3(T x, T y, T z);
	Vector3(const Vector3& v);

	Vector3& operator = (const Vector3& v);
	void Assign(T x, T y, T z);

	bool operator != (const Vector3& v) const;
	bool operator == (const Vector3& v) const;

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

	Vector4& operator = (const Vector4& v);
	void Assign(T x, T y, T z, T w);

	bool operator != (const Vector4& v) const;
	bool operator == (const Vector4& v) const;

	T Dot(const Vector4& v) const;

}; // Vector4

typedef Vector2<bool> bvec2;

typedef Vector2<int> ivec2;
typedef Vector3<int> ivec3;
typedef Vector4<int> ivec4;

typedef Vector2<float> vec2;
typedef Vector3<float> vec3;
typedef Vector4<float> vec4;

}

#include "SM_Vector.inl"

#endif // _SPATIAL_MATH_VECTOR_H_