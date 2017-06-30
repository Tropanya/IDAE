#ifndef IDAEMATH_H
#define IDAEMATH_H

#include <cassert>
#include <cmath>
#include <iostream>

namespace idaem
{
	class CQuaternion;

	class CVector2;
	class CVector3;
	class CVector4;

	class CMatrix2x2;
	class CMatrix3x3;
	class CMatrix4x4;

	/*---------------------------------------Constants---------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	const float PI_VALUE = 3.141592654f;

	const std::size_t QUATERNION_SIZE = 4;

	const std::size_t VECTOR2_SIZE = 2;
	const std::size_t VECTOR3_SIZE = 3;
	const std::size_t VECTOR4_SIZE = 4;

	const std::size_t MATRIX_2X2_SIZE = 4;
	const std::size_t MATRIX_3X3_SIZE = 9;
	const std::size_t MATRIX_4X4_SIZE = 16;
	/*---------------------------------------Constants---------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*---------------------------------------Functions---------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	float ToRadians(const float& degrees);
	float ToDegrees(const float& radians);

	float Sin(const float& degrees);
	float Cos(const float& degrees);
	float Tan(const float& degrees);

	float Sinh(const float& degrees);
	float Cosh(const float& degrees);
	float Tanh(const float& degrees);

	float Asin(const float& degrees);
	float Acos(const float& degrees);
	float Atan(const float& degrees);

	float Asinh(const float& degrees);
	float Acosh(const float& degrees);
	float Atanh(const float& degrees);
	/*---------------------------------------Functions---------------------------------------*/
	/*--------------------------------------Quaternions--------------------------------------*/
	CQuaternion Cross(const CQuaternion& q1, const CQuaternion& q2);
	float Dot(const CQuaternion& q1, const CQuaternion& q2);
	float GetLengthSqrd(const CQuaternion& q);
	float GetLength(const CQuaternion& q);
	CQuaternion Normalize(const CQuaternion& q);

	CQuaternion Conjugate(const CQuaternion& q);
	CQuaternion Inverse(const CQuaternion& q);
	/*---------------------------------------Functions---------------------------------------*/
	/*----------------------------------------Vector2----------------------------------------*/
	CVector2 Cross(const CVector2& v1, const CVector2& v2);
	float Dot(const CVector2& v1, const CVector2& v2);
	float GetLengthSqrd(const CVector2& v);
	float GetLength(const CVector2& v);
	CVector2 Normalize(const CVector2& v);
	/*---------------------------------------Functions---------------------------------------*/
	/*----------------------------------------Vector3----------------------------------------*/
	CVector3 Cross(const CVector3& v1, const CVector3& v2);
	float Dot(const CVector3& v1, const CVector3& v2);
	float GetLengthSqrd(const CVector3& v);
	float GetLength(const CVector3& v);
	CVector3 Normal(const CVector3& v1, const CVector3& v2, const CVector3& v3);
	CVector3 Normalize(const CVector3& v);
	/*---------------------------------------Functions---------------------------------------*/
	/*----------------------------------------Vector4----------------------------------------*/
	CVector4 Cross(const CVector4& v1, const CVector4& v2);
	float Dot(const CVector4& v1, const CVector4& v2);
	float GetLengthSqrd(const CVector4& v);
	float GetLength(const CVector4& v);
	CVector4 Normal(const CVector4& v1, const CVector4& v2, const CVector4& v3);
	CVector4 Normalize(const CVector4& v);
	/*---------------------------------------Functions---------------------------------------*/
	/*----------------------------------------Matrix2----------------------------------------*/
	float Det2(const CMatrix2x2& m);
	float Det2(const float& r11, const float& r12,
			   const float& r21, const float& r22);

	CMatrix2x2 Inverse(const CMatrix2x2& m);
	CMatrix2x2 Transpose(const CMatrix2x2& m);

	CMatrix2x2 Scale(const CMatrix2x2& m, const CVector2& v);//
	CMatrix2x2 Scale(const CMatrix2x2& m, const float& s);//
	CMatrix2x2 Scale(const CMatrix2x2& m, const float& s1, const float& s2);//
	/*---------------------------------------Functions---------------------------------------*/
	/*----------------------------------------Matrix3----------------------------------------*/
	float Det3(const CMatrix3x3& m);
	float Det3(const float& r11, const float& r12, const float& r13,
			   const float& r21, const float& r22, const float& r23,
			   const float& r31, const float& r32, const float& r33);

	CMatrix3x3 Inverse(const CMatrix3x3& m);
	CMatrix3x3 Transpose(const CMatrix3x3& m);

	CMatrix3x3 Scale(const CMatrix3x3& m, const CVector2& v);//
	CMatrix3x3 Scale(const CMatrix3x3& m, const float& s1, const float& s2);//
	CMatrix3x3 Scale(const CMatrix3x3& m, const CVector3& v);
	CMatrix3x3 Scale(const CMatrix3x3& m, const float& s);
	CMatrix3x3 Scale(const CMatrix3x3& m, const float& s1, const float& s2, const float& s3);
	/*---------------------------------------Functions---------------------------------------*/
	/*----------------------------------------Matrix4----------------------------------------*/
	float Det4(const CMatrix4x4& m);
	float Det4(const float& r11, const float& r12, const float& r13, const float& r14,
			   const float& r21, const float& r22, const float& r23, const float& r24,
			   const float& r31, const float& r32, const float& r33, const float& r34,
			   const float& r41, const float& r42, const float& r43, const float& r44);

	CMatrix4x4 Inverse(const CMatrix4x4& m);
	CMatrix4x4 Transpose(const CMatrix4x4& m);

	CMatrix4x4 Translate(const CMatrix4x4& m, const float& s);
	CMatrix4x4 Translate(const CMatrix4x4& m, const CVector3& v);
	CMatrix4x4 Translate(const CMatrix4x4& m, const CVector4& v);
	CMatrix4x4 Translate(const CMatrix4x4& m, const float& s1, const float& s2, const float& s3);

	CMatrix4x4 Scale(const CMatrix4x4& m, const CVector3& v);
	CMatrix4x4 Scale(const CMatrix4x4& m, const CVector4& v);
	CMatrix4x4 Scale(const CMatrix4x4& m, const float& s);
	CMatrix4x4 Scale(const CMatrix4x4& m, const float& s1, const float& s2, const float& s3);
	/*---------------------------------------Functions---------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*--------------------------------------Quaternions--------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	class CQuaternion
	{
	public:
		union
		{
			float element[4];

			struct
			{
				float w, x, y, z;
			};
		};
	public:
		CQuaternion();
		CQuaternion(const float& s, const CVector3& v);
		CQuaternion(const float& w, const float& x, const float& y, const float& z);
		CQuaternion(const CQuaternion& q);

		CQuaternion& Add(const CQuaternion& q);
		CQuaternion& Multiply(const CQuaternion& q);
		CQuaternion& Multiply(const float& s);
		CQuaternion& Divide(const float& s);

		CQuaternion& operator=(const CQuaternion& q);

		float& operator[](int i);
		const float& operator[](int i) const;

		CQuaternion& operator+=(const CQuaternion& q);
		CQuaternion& operator*=(const CQuaternion& q);
		CQuaternion& operator*=(const float& s);
		CQuaternion& operator/=(const float& s);

		friend CQuaternion operator+(const CQuaternion& q);
		friend CQuaternion operator-(const CQuaternion& q);

		friend CQuaternion operator+(const CQuaternion& q1, const CQuaternion& q2);
		friend CQuaternion operator*(const CQuaternion& q1, const CQuaternion& q2);
		friend CVector3 operator*(const CQuaternion& q, const CVector3& v);
		friend CVector3 operator*(const CVector3& v, const CQuaternion& q);
		friend CVector4 operator*(const CQuaternion& q, const CVector4& v);
		friend CVector4 operator*(const CVector4& v, const CQuaternion& q);
		friend CQuaternion operator*(const CQuaternion& q, const float& s);
		friend CQuaternion operator*(const float& s, const CQuaternion& q);
		friend CQuaternion operator/(const CQuaternion& q, const float& s);

		friend bool operator==(const CQuaternion& leftQuaternion, const CQuaternion& rightQuaternion);
		friend bool operator!=(const CQuaternion& leftQuaternion, const CQuaternion& rightQuaternion);

		friend std::ostream& operator<<(std::ostream& stream, const CQuaternion& q);
	} typedef quat;
	/*--------------------------------------Quaternions--------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Vector2----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	class CVector2
	{
	public:
		union
		{
			float element[2];

			struct
			{
				float x, y;
			};
		};
	public:
		CVector2();
		CVector2(const float& s);
		CVector2(const float& x, const float& y);
		CVector2(const CVector2& v);
		CVector2(const CVector3& v);
		CVector2(const CVector4& v);

		CVector2& Add(const CVector2& v);
		CVector2& Add(const float& s);
		CVector2& Subtract(const CVector2& v);
		CVector2& Subtract(const float& s);
		CVector2& Multiply(const CVector2& v);
		CVector2& Multiply(const float& s);
		CVector2& Divide(const CVector2& v);
		CVector2& Divide(const float& s);

		void Clear();

		CVector2& operator=(const CVector2& v);

		float& operator[](int i);
		const float& operator[](int i) const;

		CVector2& operator+=(const CVector2& v);
		CVector2& operator+=(const float& s);
		CVector2& operator-=(const CVector2& v);
		CVector2& operator-=(const float& s);
		CVector2& operator*=(const CVector2& v);
		CVector2& operator*=(const float& s);
		CVector2& operator/=(const CVector2& v);
		CVector2& operator/=(const float& s);

		CVector2& operator++();
		CVector2& operator--();

		friend CVector2 operator+(const CVector2& v);
		friend CVector2 operator-(const CVector2& v);

		friend CVector2 operator+(const CVector2& v, const float& s);
		friend CVector2 operator+(const float& s, const CVector2& v);
		friend CVector2 operator+(const CVector2& v1, const CVector2& v2);
		friend CVector2 operator-(const CVector2& v, const float& s);
		friend CVector2 operator-(const float& s, const CVector2& v);
		friend CVector2 operator-(const CVector2& v1, const CVector2& v2);
		friend CVector2 operator*(const CVector2& v, const float& s);
		friend CVector2 operator*(const float& s, const CVector2& v);
		friend CVector2 operator*(const CVector2& v1, const CVector2& v2);
		friend CVector2 operator/(const CVector2& v, const float& s);
		friend CVector2 operator/(const float& s, const CVector2& v);
		friend CVector2 operator/(const CVector2& v1, const CVector2& v2);

		friend bool operator==(const CVector2& leftVector, const CVector2& rightVector);
		friend bool operator!=(const CVector2& leftVector, const CVector2& rightVector);

		friend std::ostream& operator<<(std::ostream& stream, const CVector2& v);
	} typedef vec2;
	/*----------------------------------------Vector2----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Vector3----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	class CVector3
	{
	public:
		union
		{
			float element[3];

			struct
			{
				float x, y, z;
			};
		};
	public:
		CVector3();
		CVector3(const float& s);
		CVector3(const float& x, const float& y, const float& z);
		CVector3(const CVector3& v);
		CVector3(const CVector2& v);
		CVector3(const CVector2& v, const float& s);
		CVector3(const CVector4& v);

		CVector3& Add(const CVector3& v);
		CVector3& Add(const float& s);
		CVector3& Subtract(const CVector3& v);
		CVector3& Subtract(const float& s);
		CVector3& Multiply(const CVector3& v);
		CVector3& Multiply(const float& s);
		CVector3& Divide(const CVector3& v);
		CVector3& Divide(const float& s);

		void Clear();

		CVector3& operator=(const CVector3& v);

		float& operator[](int i);
		const float& operator[](int i) const;

		CVector3& operator+=(const CVector3& v);
		CVector3& operator+=(const float& s);
		CVector3& operator-=(const CVector3& v);
		CVector3& operator-=(const float& s);
		CVector3& operator*=(const CVector3& v);
		CVector3& operator*=(const float& s);
		CVector3& operator/=(const CVector3& v);
		CVector3& operator/=(const float& s);

		CVector3& operator++();
		CVector3& operator--();

		friend CVector3 operator+(const CVector3& v);
		friend CVector3 operator-(const CVector3& v);

		friend CVector3 operator+(const CVector3& v, const float& s);
		friend CVector3 operator+(const float& s, const CVector3& v);
		friend CVector3 operator+(const CVector3& v1, const CVector3& v2);
		friend CVector3 operator-(const CVector3& v, const float& s);
		friend CVector3 operator-(const float& s, const CVector3& v);
		friend CVector3 operator-(const CVector3& v1, const CVector3& v2);
		friend CVector3 operator*(const CVector3& v, const float& s);
		friend CVector3 operator*(const float& s, const CVector3& v);
		friend CVector3 operator*(const CVector3& v1, const CVector3& v2);
		friend CVector3 operator/(const CVector3& v, const float& s);
		friend CVector3 operator/(const float& s, const CVector3& v);
		friend CVector3 operator/(const CVector3& v1, const CVector3& v2);

		friend bool operator==(const CVector3& leftVector, const CVector3& rightVector);
		friend bool operator!=(const CVector3& leftVector, const CVector3& rightVector);

		friend std::ostream& operator<<(std::ostream& stream, const CVector3& v);
	} typedef vec3;
	/*----------------------------------------Vector3----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Vector4----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	class CVector4
	{
	public:
		union
		{
			float element[4];

			struct
			{
				float x, y, z, w;
			};
		};
	public:
		CVector4();
		CVector4(const float& s);
		CVector4(const float& x, const float& y, const float& z, const float& w);
		CVector4(const CVector4& v);
		CVector4(const CVector2& v);
		CVector4(const CVector2& v, const float& s);
		CVector4(const CVector2& v, const float& s1, const float& s2);
		CVector4(const CVector3& v);
		CVector4(const CVector3& v, const float& s);

		CVector4& Add(const CVector4& v);
		CVector4& Add(const float& s);
		CVector4& Subtract(const CVector4& v);
		CVector4& Subtract(const float& s);
		CVector4& Multiply(const CVector4& v);
		CVector4& Multiply(const float& s);
		CVector4& Divide(const CVector4& v);
		CVector4& Divide(const float& s);

		void Clear();

		CVector4& operator=(const CVector4& v);

		float& operator[](int i);
		const float& operator[](int i) const;

		CVector4& operator+=(const CVector4& v);
		CVector4& operator+=(const float& s);
		CVector4& operator-=(const CVector4& v);
		CVector4& operator-=(const float& s);
		CVector4& operator*=(const CVector4& v);
		CVector4& operator*=(const float& s);
		CVector4& operator/=(const CVector4& v);
		CVector4& operator/=(const float& s);

		CVector4& operator++();
		CVector4& operator--();

		friend CVector4 operator+(const CVector4& v);
		friend CVector4 operator-(const CVector4& v);

		friend CVector4 operator+(const CVector4& v, const float& s);
		friend CVector4 operator+(const float& s, const CVector4& v);
		friend CVector4 operator+(const CVector4& v1, const CVector4& v2);
		friend CVector4 operator-(const CVector4& v, const float& s);
		friend CVector4 operator-(const float& s, const CVector4& v);
		friend CVector4 operator-(const CVector4& v1, const CVector4& v2);
		friend CVector4 operator*(const CVector4& v, const float& s);
		friend CVector4 operator*(const float& s, const CVector4& v);
		friend CVector4 operator*(const CVector4& v1, const CVector4& v2);
		friend CVector4 operator/(const CVector4& v, const float& s);
		friend CVector4 operator/(const float& s, const CVector4& v);
		friend CVector4 operator/(const CVector4& v1, const CVector4& v2);

		friend bool operator==(const CVector4& leftVector, const CVector4& rightVector);
		friend bool operator!=(const CVector4& leftVector, const CVector4& rightVector);

		friend std::ostream& operator<<(std::ostream& stream, const CVector4& v);
	} typedef vec4;
	/*----------------------------------------Vector2----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Matrix2----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	class CMatrix2x2
	{
	public:
		union
		{
			float _1D[2 * 2];
			float _2D[2][2];
			CVector2 column[2];
		};
	public:
		CMatrix2x2();
		CMatrix2x2(const float& s);
		CMatrix2x2(const float& r11, const float& r12,
				   const float& r21, const float& r22);
		CMatrix2x2(const CVector2& col1, const CVector2& col2);
		CMatrix2x2(const CMatrix2x2& m);
		CMatrix2x2(const CMatrix3x3& m);
		CMatrix2x2(const CMatrix4x4& m);

		CMatrix2x2& Add(const CMatrix2x2& m);
		CMatrix2x2& Add(const float& s);
		CMatrix2x2& Subtract(const CMatrix2x2& m);
		CMatrix2x2& Subtract(const float& s);
		CMatrix2x2& Multiply(const CMatrix2x2& m);
		CMatrix2x2& Multiply(const float& s);
		CMatrix2x2& Divide(const CMatrix2x2& m);
		CMatrix2x2& Divide(const float& s);

		static CMatrix2x2 Identity();

		void Zero();

		CMatrix2x2& operator=(const CMatrix2x2& m);

		float& operator[](int i);
		const float& operator[](int i) const;

		CMatrix2x2& operator+=(const CMatrix2x2& m);
		CMatrix2x2& operator+=(const float& s);
		CMatrix2x2& operator-=(const CMatrix2x2& m);
		CMatrix2x2& operator-=(const float& s);
		CMatrix2x2& operator*=(const CMatrix2x2& m);
		CMatrix2x2& operator*=(const float& s);
		CMatrix2x2& operator/=(const CMatrix2x2& m);
		CMatrix2x2& operator/=(const float& s);

		CMatrix2x2& operator++();
		CMatrix2x2& operator--();

		friend CMatrix2x2 operator+(const CMatrix2x2& m);
		friend CMatrix2x2 operator-(const CMatrix2x2& m);

		friend CMatrix2x2 operator+(const CMatrix2x2& m, const float& s);
		friend CMatrix2x2 operator+(const float& s, const CMatrix2x2& m);
		friend CMatrix2x2 operator+(const CMatrix2x2& m1, const CMatrix2x2& m2);
		friend CMatrix2x2 operator-(const CMatrix2x2& m, const float& s);
		friend CMatrix2x2 operator-(const float& s, const CMatrix2x2& m);
		friend CMatrix2x2 operator-(const CMatrix2x2& m1, const CMatrix2x2& m2);
		friend CMatrix2x2 operator*(const CMatrix2x2& m, const float& s);
		friend CMatrix2x2 operator*(const float& s, const CMatrix2x2& m);
		friend CMatrix2x2 operator*(const CMatrix2x2& m1, const CMatrix2x2& m2);
		friend CMatrix2x2 operator/(const CMatrix2x2& m, const float& s);
		friend CMatrix2x2 operator/(const float& s, const CMatrix2x2& m);
		friend CMatrix2x2 operator/(const CMatrix2x2& m1, const CMatrix2x2& m2);

		friend bool operator==(const CMatrix2x2& leftMatrix, const CMatrix2x2& rightMatrix);
		friend bool operator!=(const CMatrix2x2& leftMatrix, const CMatrix2x2& rightMatrix);

		friend std::ostream& operator<<(std::ostream& stream, const CMatrix2x2& m);
	} typedef mat2;
	/*----------------------------------------Matrix2----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Matrix3----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	class CMatrix3x3
	{
	public:
		union
		{
			float _1D[3 * 3];
			float _2D[3][3];
			CVector3 column[3];
		};
	public:
		CMatrix3x3();
		CMatrix3x3(const float& s);
		CMatrix3x3(const float& r11, const float& r12, const float& r13,
				   const float& r21, const float& r22, const float& r23,
				   const float& r31, const float& r32, const float& r33);
		CMatrix3x3(const CVector3& col1, const CVector3& col2, const CVector3& col3);
		CMatrix3x3(const CMatrix3x3& m);
		CMatrix3x3(const CMatrix2x2& m);
		CMatrix3x3(const CMatrix4x4& m);

		CMatrix3x3& Add(const CMatrix3x3& m);
		CMatrix3x3& Add(const float& s);
		CMatrix3x3& Subtract(const CMatrix3x3& m);
		CMatrix3x3& Subtract(const float& s);
		CMatrix3x3& Multiply(const CMatrix3x3& m);
		CMatrix3x3& Multiply(const float& s);
		CMatrix3x3& Divide(const CMatrix3x3& m);
		CMatrix3x3& Divide(const float& s);

		static CMatrix3x3 Identity();

		void Zero();

		CMatrix3x3& operator=(const CMatrix3x3& m);

		float& operator[](int i);
		const float& operator[](int i) const;

		CMatrix3x3& operator+=(const CMatrix3x3& m);
		CMatrix3x3& operator+=(const float& s);
		CMatrix3x3& operator-=(const CMatrix3x3& m);
		CMatrix3x3& operator-=(const float& s);
		CMatrix3x3& operator*=(const CMatrix3x3& m);
		CMatrix3x3& operator*=(const float& s);
		CMatrix3x3& operator/=(const CMatrix3x3& m);
		CMatrix3x3& operator/=(const float& s);

		CMatrix3x3& operator++();
		CMatrix3x3& operator--();

		friend CMatrix3x3 operator+(const CMatrix3x3& m);
		friend CMatrix3x3 operator-(const CMatrix3x3& m);

		friend CMatrix3x3 operator+(const CMatrix3x3& m, const float& s);
		friend CMatrix3x3 operator+(const float& s, const CMatrix3x3& m);
		friend CMatrix3x3 operator+(const CMatrix3x3& m1, const CMatrix3x3& m2);
		friend CMatrix3x3 operator-(const CMatrix3x3& m, const float& s);
		friend CMatrix3x3 operator-(const float& s, const CMatrix3x3& m);
		friend CMatrix3x3 operator-(const CMatrix3x3& m1, const CMatrix3x3& m2);
		friend CMatrix3x3 operator*(const CMatrix3x3& m, const float& s);
		friend CMatrix3x3 operator*(const float& s, const CMatrix3x3& m);
		friend CMatrix3x3 operator*(const CMatrix3x3& m1, const CMatrix3x3& m2);
		friend CMatrix3x3 operator/(const CMatrix3x3& m, const float& s);
		friend CMatrix3x3 operator/(const float& s, const CMatrix3x3& m);
		friend CMatrix3x3 operator/(const CMatrix3x3& m1, const CMatrix3x3& m2);

		friend bool operator==(const CMatrix3x3& leftMatrix, const CMatrix3x3& rightMatrix);
		friend bool operator!=(const CMatrix3x3& leftMatrix, const CMatrix3x3& rightMatrix);

		friend std::ostream& operator<<(std::ostream& stream, const CMatrix3x3& m);
	} typedef mat3;
	/*----------------------------------------Matrix3----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Matrix4----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	class CMatrix4x4
	{
	public:
		union
		{
			float _1D[4 * 4];
			float _2D[4][4];
			CVector4 column[4];
		};
	public:
		CMatrix4x4();
		CMatrix4x4(const float& s);
		CMatrix4x4(const float& r11, const float& r12, const float& r13, const float& r14,
				   const float& r21, const float& r22, const float& r23, const float& r24,
				   const float& r31, const float& r32, const float& r33, const float& r34,
				   const float& r41, const float& r42, const float& r43, const float& r44);
		CMatrix4x4(const CVector4& col1, const CVector4& col2, const CVector4& col3, const CVector4& col4);
		CMatrix4x4(const CMatrix4x4& m);
		CMatrix4x4(const CMatrix2x2& m);
		CMatrix4x4(const CMatrix3x3& m);

		CMatrix4x4& Add(const CMatrix4x4& m);
		CMatrix4x4& Add(const float& s);
		CMatrix4x4& Subtract(const CMatrix4x4& m);
		CMatrix4x4& Subtract(const float& s);
		CMatrix4x4& Multiply(const CMatrix4x4& m);
		CMatrix4x4& Multiply(const float& s);
		CMatrix4x4& Divide(const CMatrix4x4& m);
		CMatrix4x4& Divide(const float& s);

		static CMatrix4x4 Identity();

		void Zero();

		CMatrix4x4& operator=(const CMatrix4x4& m);

		float& operator[](int i);
		const float& operator[](int i) const;

		CMatrix4x4& operator+=(const CMatrix4x4& m);
		CMatrix4x4& operator+=(const float& s);
		CMatrix4x4& operator-=(const CMatrix4x4& m);
		CMatrix4x4& operator-=(const float& s);
		CMatrix4x4& operator*=(const CMatrix4x4& m);
		CMatrix4x4& operator*=(const float& s);
		CMatrix4x4& operator/=(const CMatrix4x4& m);
		CMatrix4x4& operator/=(const float& s);

		CMatrix4x4& operator++();
		CMatrix4x4& operator--();

		friend CMatrix4x4 operator+(const CMatrix4x4& m);
		friend CMatrix4x4 operator-(const CMatrix4x4& m);

		friend CMatrix4x4 operator+(const CMatrix4x4& m, const float& s);
		friend CMatrix4x4 operator+(const float& s, const CMatrix4x4& m);
		friend CMatrix4x4 operator+(const CMatrix4x4& m1, const CMatrix4x4& m2);
		friend CMatrix4x4 operator-(const CMatrix4x4& m, const float& s);
		friend CMatrix4x4 operator-(const float& s, const CMatrix4x4& m);
		friend CMatrix4x4 operator-(const CMatrix4x4& m1, const CMatrix4x4& m2);
		friend CMatrix4x4 operator*(const CMatrix4x4& m, const float& s);
		friend CMatrix4x4 operator*(const float& s, const CMatrix4x4& m);
		friend CMatrix4x4 operator*(const CMatrix4x4& m1, const CMatrix4x4& m2);
		friend CMatrix4x4 operator/(const CMatrix4x4& m, const float& s);
		friend CMatrix4x4 operator/(const float& s, const CMatrix4x4& m);
		friend CMatrix4x4 operator/(const CMatrix4x4& m1, const CMatrix4x4& m2);

		friend bool operator==(const CMatrix4x4& leftMatrix, const CMatrix4x4& rightMatrix);
		friend bool operator!=(const CMatrix4x4& leftMatrix, const CMatrix4x4& rightMatrix);

		friend std::ostream& operator<<(std::ostream& stream, const CMatrix4x4& m);
	} typedef mat4;
	/*----------------------------------------Matrix4----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

} // namespace idaem

#endif // IDAEMATH_H