#include "IDAEMath.h"

namespace idaem
{
	/*---------------------------------------Functions---------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	float ToRadians(const float& degrees) { return (degrees * (PI_VALUE / 180.0f)); }

	float ToDegrees(const float& radians) { return (radians * (180.0f / PI_VALUE)); }

	float Sin(const float& degrees) { return sin(ToRadians(degrees)); }

	float Cos(const float& degrees) { return cos(ToRadians(degrees)); }

	float Tan(const float& degrees) { return tan(ToRadians(degrees)); }

	float Sinh(const float& s) { return sinh(s); }

	float Cosh(const float& s) { return cosh(s); }

	float Tanh(const float& s) { return tanh(s); }

	float Asin(const float& degrees) { return asin(ToRadians(degrees)); }

	float Acos(const float& degrees) { return acos(ToRadians(degrees)); }

	float Atan(const float& degrees) { return atan(ToRadians(degrees)); }

	float Asinh(const float & degrees) { return asinh(ToRadians(degrees)); }

	float Acosh(const float & degrees) { return acosh(ToRadians(degrees)); }

	float Atanh(const float & degrees) { return atanh(ToRadians(degrees)); }

	CQuaternion Cross(const CQuaternion& q1, const CQuaternion& q2)
	{
		return CQuaternion(q1.element[0] * q2.element[0] - q1.element[1] * q2.element[1] - q1.element[2] * q2.element[2] - q1.element[3] * q2.element[3],
						   q1.element[0] * q2.element[1] + q1.element[1] * q2.element[0] + q1.element[2] * q2.element[3] - q1.element[3] * q2.element[2],
						   q1.element[0] * q2.element[2] + q1.element[2] * q2.element[0] + q1.element[3] * q2.element[1] - q1.element[1] * q2.element[3],
						   q1.element[0] * q2.element[3] + q1.element[3] * q2.element[0] + q1.element[1] * q2.element[2] - q1.element[2] * q2.element[1]);
	}

	float Dot(const CQuaternion& q1, const CQuaternion& q2)
	{
		return (q1.element[0] * q2.element[0]) +
			   (q1.element[1] * q2.element[1]);
	}

	float GetLengthSqrd(const CQuaternion& q)
	{
		return (q.element[0] * q.element[0]) +
			   (q.element[1] * q.element[1]);
	}

	float GetLength(const CQuaternion& q)
	{
		return sqrt(GetLengthSqrd(q));
	}

	CQuaternion Normalize(const CQuaternion& q)
	{
		float length = GetLength(q);

		if (length <= 0.0f)
			return CQuaternion();

		return q / length;
	}

	CQuaternion Conjugate(const CQuaternion& q)
	{
		return CQuaternion(q.element[0], -q.element[1], -q.element[2], -q.element[3]);
	}

	CQuaternion Inverse(const CQuaternion& q)
	{
		return Conjugate(q) / Dot(q, q);
	}
	
	CVector2 Cross(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(v1.element[0] * v2.element[1] - v2.element[0] * v1.element[1]);
	}

	float Dot(const CVector2& v1, const CVector2& v2)
	{
		return (v1.element[0] * v2.element[0]) +
			   (v1.element[1] * v2.element[1]);
	}

	float GetLengthSqrd(const CVector2& v)
	{
		return (v.element[0] * v.element[0]) +
			   (v.element[1] * v.element[1]);
	}

	float GetLength(const CVector2& v)
	{
		return sqrt(GetLengthSqrd(v));
	}

	CVector2 Normalize(const CVector2& v)
	{
		return v / GetLength(v);
	}
	
	CVector3 Cross(const CVector3& v1, const CVector3& v2)
	{
		return CVector3((v1.element[1] * v2.element[2]) - (v1.element[2] * v2.element[1]),
						(v1.element[2] * v2.element[0]) - (v1.element[0] * v2.element[2]),
						(v1.element[0] * v2.element[1]) - (v1.element[1] * v2.element[0]));
	}

	float Dot(const CVector3& v1, const CVector3& v2)
	{
		return (v1.element[0] * v2.element[0]) +
			   (v1.element[1] * v2.element[1]) +
			   (v1.element[2] * v2.element[2]);
	}

	float GetLengthSqrd(const CVector3& v)
	{
		return (v.element[0] * v.element[0]) +
			   (v.element[1] * v.element[1]) +
			   (v.element[2] * v.element[2]);
	}

	float GetLength(const CVector3& v)
	{
		return sqrt(GetLengthSqrd(v));
	}

	CVector3 Normal(const CVector3& v1, const CVector3& v2, const CVector3& v3)
	{
		return Normalize(Cross(v2 - v1, v3 - v1));
	}

	CVector3 Normalize(const CVector3& v)
	{
		return v / GetLength(v);
	}

	CVector4 Cross(const CVector4& v1, const CVector4& v2)
	{
		CVector3 result((v1.element[1] * v2.element[2]) - (v1.element[2] * v2.element[1]),
						(v1.element[2] * v2.element[0]) - (v1.element[0] * v2.element[2]),
						(v1.element[0] * v2.element[1]) - (v1.element[1] * v2.element[0]));

		return CVector4(result);
	}

	float Dot(const CVector4& v1, const CVector4& v2)
	{
		return (v1.element[0] * v2.element[0]) +
			   (v1.element[1] * v2.element[1]) +
			   (v1.element[2] * v2.element[2]) +
			   (v1.element[3] * v2.element[3]);
	}

	float GetLengthSqrd(const CVector4& v)
	{
		return (v.element[0] * v.element[0]) +
			   (v.element[1] * v.element[1]) +
			   (v.element[2] * v.element[2]) +
			   (v.element[3] * v.element[3]);
	}

	float GetLength(const CVector4& v)
	{
		return sqrt(GetLengthSqrd(v));
	}

	CVector4 Normal(const CVector4& v1, const CVector4& v2, const CVector4& v3)
	{
		return Normalize(Cross(v2 - v1, v3 - v1));
	}

	CVector4 Normalize(const CVector4& v)
	{
		return v / GetLength(v);
	}

	float Det2(const CMatrix2x2& m)
	{
		return (m._1D[0] * m._1D[3]) - (m._1D[2] * m._1D[1]);
	}

	float Det2(const float& r11, const float& r12,
			   const float& r21, const float& r22)
	{
		return (r11 * r22) - (r21 * r12);
	}

	CMatrix2x2 Inverse(const CMatrix2x2& m)
	{
		float det = Det2(m);

		if (det == 0.0f)
			return m;

		float invDet = 1.0f / det;

		return CMatrix2x2(+m._1D[3] * invDet, -m._1D[1] * invDet,
						  -m._1D[2] * invDet, +m._1D[0] * invDet);
	}

	CMatrix2x2 Transpose(const CMatrix2x2& m)
	{
		return CMatrix2x2(m._1D[0], m._1D[2],
						  m._1D[1], m._1D[3]);
	}

	CMatrix2x2 Scale(const CMatrix2x2& m, const CVector2& v)
	{
		return CMatrix2x2(m.column[0] * v.element[0],
						  m.column[1] * v.element[1]);
	}

	CMatrix2x2 Scale(const CMatrix2x2& m, const float& s)
	{
		return CMatrix2x2(m.column[0] * s,
						  m.column[1] * s);
	}

	CMatrix2x2 Scale(const CMatrix2x2& m, const float& s1, const float& s2)
	{
		return CMatrix2x2(m.column[0] * s1,
						  m.column[1] * s2);
	}

	CMatrix2x2 Rotate(const float& angle)
	{
		return CMatrix2x2(Cos(angle), -Sin(angle),
						  Sin(angle),  Cos(angle));
	}

	float Det3(const CMatrix3x3& m)
	{
		return (m._1D[0] * m._1D[4] * m._1D[8]) + (m._1D[3] * m._1D[7] * m._1D[2]) + (m._1D[6] * m._1D[1] * m._1D[5]) -
			   (m._1D[6] * m._1D[4] * m._1D[2]) - (m._1D[7] * m._1D[5] * m._1D[0]) - (m._1D[3] * m._1D[1] * m._1D[8]);
	}

	float Det3(const float& r11, const float& r12, const float& r13,
			   const float& r21, const float& r22, const float& r23,
			   const float& r31, const float& r32, const float& r33)
	{
		return (r11 * r22 * r33) + (r21 * r32 * r13) + (r12 * r23 * r31) -
			   (r31 * r22 * r13) - (r21 * r12 * r33) - (r11 * r32 * r23);
	}

	CMatrix3x3 Inverse(const CMatrix3x3& m)
	{
		float det = Det3(m);

		if (det == 0.0f)
			return m;

		float invDet = 1.0f / det;

		return CMatrix3x3(+Det2(m._1D[4], m._1D[5], m._1D[7], m._1D[8]) * invDet,
						  -Det2(m._1D[1], m._1D[2], m._1D[7], m._1D[8]) * invDet,
						  +Det2(m._1D[1], m._1D[2], m._1D[4], m._1D[5]) * invDet,
						  
						  -Det2(m._1D[3], m._1D[5], m._1D[6], m._1D[8]) * invDet,
						  +Det2(m._1D[0], m._1D[2], m._1D[6], m._1D[8]) * invDet,
						  -Det2(m._1D[0], m._1D[2], m._1D[3], m._1D[5]) * invDet,
						  
						  +Det2(m._1D[3], m._1D[4], m._1D[6], m._1D[7]) * invDet,
						  -Det2(m._1D[0], m._1D[1], m._1D[6], m._1D[7]) * invDet,
						  +Det2(m._1D[0], m._1D[1], m._1D[3], m._1D[4]) * invDet);
	}

	CMatrix3x3 Transpose(const CMatrix3x3& m)
	{
		return CMatrix3x3(m._1D[0], m._1D[3], m._1D[6],
						  m._1D[1], m._1D[4], m._1D[7],
						  m._1D[2], m._1D[5], m._1D[8]);
	}

	CMatrix3x3 Translate(const CMatrix3x3& m, const float& s)
	{
		CMatrix3x3 result(m);

		result.column[2] = m.column[0] * s +
						   m.column[1] * s +
						   m.column[2];

		return result;
	}

	CMatrix3x3 Translate(const CMatrix3x3& m, const CVector2& v)
	{
		CMatrix3x3 result(m);

		result.column[2] = m.column[0] * v.element[0] +
						   m.column[1] * v.element[1] +
						   m.column[2];

		return result;
	}

	CMatrix3x3 Translate(const CMatrix3x3& m, const CVector3& v)
	{
		CMatrix3x3 result(m);

		result.column[2] = m.column[0] * v.element[0] +
						   m.column[1] * v.element[1] +
						   m.column[2];

		return result;
	}

	CMatrix3x3 Translate(const CMatrix3x3& m, const float& s1, const float& s2)
	{
		CMatrix3x3 result(m);

		result.column[2] = m.column[0] * s1 +
						   m.column[1] * s2 +
						   m.column[2];

		return result;
	}

	CMatrix3x3 Scale(const CMatrix3x3& m, const CVector3& v)
	{
		return CMatrix3x3(m.column[0] * v.element[0],
						  m.column[1] * v.element[1],
						  m.column[2] * v.element[2]);
	}

	CMatrix3x3 Scale(const CMatrix3x3& m, const float& s)
	{
		return CMatrix3x3(m.column[0] * s,
						  m.column[1] * s,
						  m.column[2] * s);
	}

	CMatrix3x3 Scale(const CMatrix3x3& m, const float& s1, const float& s2, const float& s3)
	{
		return CMatrix3x3(m.column[0] * s1,
						  m.column[1] * s2,
						  m.column[2] * s3);
	}

	float Det4(const CMatrix4x4& m)
	{
		return m._1D[0]  * Det3(m._1D[5], m._1D[6], m._1D[7], m._1D[9], m._1D[10], m._1D[11], m._1D[13], m._1D[14], m._1D[15]) -
			   m._1D[4]  * Det3(m._1D[1], m._1D[2], m._1D[3], m._1D[9], m._1D[10], m._1D[11], m._1D[13], m._1D[14], m._1D[15]) +
			   m._1D[8]  * Det3(m._1D[1], m._1D[2], m._1D[3], m._1D[5], m._1D[6],  m._1D[7],  m._1D[13], m._1D[14], m._1D[15]) -
			   m._1D[12] * Det3(m._1D[1], m._1D[2], m._1D[3], m._1D[5], m._1D[6],  m._1D[7],  m._1D[9],  m._1D[10], m._1D[11]);
	}

	float Det4(const float& r11, const float& r12, const float& r13, const float& r14,
			   const float& r21, const float& r22, const float& r23, const float& r24,
			   const float& r31, const float& r32, const float& r33, const float& r34,
			   const float& r41, const float& r42, const float& r43, const float& r44)
	{
		return r11 * Det3(r22, r23, r24, r32, r33, r34, r42, r43, r44) -
			   r21 * Det3(r12, r13, r14, r32, r33, r34, r42, r43, r44) +
			   r31 * Det3(r12, r13, r14, r22, r23, r24, r42, r43, r44) -
			   r41 * Det3(r12, r13, r14, r22, r23, r24, r32, r33, r34);
	}

	CMatrix4x4 Inverse(const CMatrix4x4& m)
	{
		float det = Det4(m);

		if (det == 0.0f)
			return m;

		float invDet = 1.0f / det;

		return CMatrix4x4(+Det3(m._1D[5], m._1D[6], m._1D[7], m._1D[9], m._1D[10], m._1D[11], m._1D[13], m._1D[14], m._1D[15]) * invDet,
						  -Det3(m._1D[1], m._1D[2], m._1D[3], m._1D[9], m._1D[10], m._1D[11], m._1D[13], m._1D[14], m._1D[15]) * invDet,
						  +Det3(m._1D[1], m._1D[2], m._1D[3], m._1D[5], m._1D[6],  m._1D[7],  m._1D[13], m._1D[14], m._1D[15]) * invDet,
						  -Det3(m._1D[1], m._1D[2], m._1D[3], m._1D[5], m._1D[6],  m._1D[7],  m._1D[9],  m._1D[10], m._1D[11]) * invDet,
						  
						  -Det3(m._1D[4], m._1D[6], m._1D[7], m._1D[8], m._1D[10], m._1D[11], m._1D[12], m._1D[14], m._1D[15]) * invDet,
						  +Det3(m._1D[0], m._1D[2], m._1D[3], m._1D[8], m._1D[10], m._1D[11], m._1D[12], m._1D[14], m._1D[15]) * invDet,
						  -Det3(m._1D[0], m._1D[2], m._1D[3], m._1D[4], m._1D[6],  m._1D[7],  m._1D[12], m._1D[14], m._1D[15]) * invDet,
						  +Det3(m._1D[0], m._1D[2], m._1D[3], m._1D[4], m._1D[6],  m._1D[7],  m._1D[8],  m._1D[10], m._1D[11]) * invDet,
						  
						  +Det3(m._1D[4], m._1D[5], m._1D[7], m._1D[8], m._1D[9],  m._1D[11], m._1D[12], m._1D[13], m._1D[15]) * invDet,
						  -Det3(m._1D[0], m._1D[1], m._1D[3], m._1D[8], m._1D[9],  m._1D[11], m._1D[12], m._1D[13], m._1D[15]) * invDet,
						  +Det3(m._1D[0], m._1D[1], m._1D[3], m._1D[4], m._1D[5],  m._1D[7],  m._1D[12], m._1D[13], m._1D[15]) * invDet,
						  -Det3(m._1D[0], m._1D[1], m._1D[3], m._1D[4], m._1D[5],  m._1D[7],  m._1D[8],  m._1D[9],  m._1D[11]) * invDet,
						  
						  -Det3(m._1D[4], m._1D[5], m._1D[6], m._1D[8], m._1D[9],  m._1D[10], m._1D[12], m._1D[13], m._1D[14]) * invDet,
						  +Det3(m._1D[0], m._1D[1], m._1D[2], m._1D[8], m._1D[9],  m._1D[10], m._1D[12], m._1D[13], m._1D[14]) * invDet,
						  -Det3(m._1D[0], m._1D[1], m._1D[2], m._1D[4], m._1D[5],  m._1D[6],  m._1D[12], m._1D[13], m._1D[14]) * invDet,
						  +Det3(m._1D[0], m._1D[1], m._1D[2], m._1D[4], m._1D[5],  m._1D[6],  m._1D[8],  m._1D[9],  m._1D[10]) * invDet);
	}

	CMatrix4x4 Transpose(const CMatrix4x4& m)
	{
		return CMatrix4x4(m._1D[0], m._1D[4], m._1D[8],  m._1D[12],
						  m._1D[1], m._1D[5], m._1D[9],  m._1D[13],
						  m._1D[2], m._1D[6], m._1D[10], m._1D[14],
						  m._1D[3], m._1D[7], m._1D[11], m._1D[15]);
	}

	CMatrix4x4 Translate(const CMatrix4x4& m, const CVector3& v)
	{
		CMatrix4x4 result(m);

		result.column[3] = m.column[0] * v.element[0] +
						   m.column[1] * v.element[1] +
						   m.column[2] * v.element[2] +
						   m.column[3];

		return result;
	}

	CMatrix4x4 Translate(const CMatrix4x4& m, const CVector4& v)
	{
		CMatrix4x4 result(m);

		result.column[3] = m.column[0] * v.element[0] +
						   m.column[1] * v.element[1] +
						   m.column[2] * v.element[2] +
						   m.column[3];

		return result;
	}

	CMatrix4x4 Translate(const CMatrix4x4& m, const float& s)
	{
		CMatrix4x4 result(m);

		result.column[3] = m.column[0] * s +
						   m.column[1] * s +
					       m.column[2] * s +
						   m.column[3];

		return result;
	}

	CMatrix4x4 Translate(const CMatrix4x4& m, const float& s1, const float& s2, const float& s3)
	{
		CMatrix4x4 result(m);

		result.column[3] = m.column[0] * s1 +
						   m.column[1] * s2 +
						   m.column[2] * s3 +
						   m.column[3];

		return result;
	}

	CMatrix4x4 Scale(const CMatrix4x4& m, const float& s)
	{
		return CMatrix4x4(m.column[0] * s,
						  m.column[1] * s,
						  m.column[2] * s,
						  m.column[3]);
	}

	CMatrix4x4 Scale(const CMatrix4x4& m, const CVector3& v)
	{
		return CMatrix4x4(m.column[0] * v.element[0],
						  m.column[1] * v.element[1],
						  m.column[2] * v.element[2],
						  m.column[3]);
	}

	CMatrix4x4 Scale(const CMatrix4x4& m, const CVector4& v)
	{
		return CMatrix4x4(m.column[0] * v.element[0],
						  m.column[1] * v.element[1],
						  m.column[2] * v.element[2],
						  m.column[3]);
	}

	CMatrix4x4 Scale(const CMatrix4x4& m, const float& s1, const float& s2, const float& s3)
	{
		return CMatrix4x4(m.column[0] * s1,
						  m.column[1] * s2,
						  m.column[2] * s3,
						  m.column[3]);
	}

	CMatrix4x4 Ortho(float left, float right, float bottom, float top)
	{
		CMatrix4x4 result(1.0f);

		result._1D[0]  =   2.0f / (right - left);
		result._1D[5]  =   2.0f / (top - bottom);
		result._1D[10] = - 1.0f;
		result._1D[12] = - (right + left) / (right - left);
		result._1D[13] = - (top + bottom) / (top - bottom);

		return result;
	}

	CMatrix4x4 Ortho(float left, float right, float bottom, float top, float near, float far)
	{
		CMatrix4x4 result(1.0f);

		result._1D[0]  =   2.0f / (right - left);
		result._1D[5]  =   2.0f / (top - bottom);
		result._1D[10] = - 2.0f / (far - near);
		result._1D[12] = - (right + left) / (right - left);
		result._1D[13] = - (top + bottom) / (top - bottom);
		result._1D[14] = - (far + near) / (far - near);

		return result;
	}
	CMatrix4x4 Perspective(float fov, float aspect, float near, float far)
	{
		CMatrix4x4 result(1.0f);
		
		const float tanHalfFov = tan(ToRadians(0.5f * fov));

		result._1D[0] = 1.0f / (aspect * tanHalfFov);
		result._1D[5] = 1.0f / (tanHalfFov);
		result._1D[10] = (near + far) / (far - near);
		result._1D[11] = 1.0f;
		result._1D[14] = (2.0f * near * far) / (near - far);

		return result;
	}
	/*---------------------------------------Functions---------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*--------------------------------------Quaternions--------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CQuaternion::CQuaternion():
		x(0.0f), y(0.0f), z(0.0f), w(1.0f)
	{ }

	CQuaternion::CQuaternion(const float& s, const CVector3& v) :
		x(v.x), y(v.y), z(v.z), w(s)
	{ }

	CQuaternion::CQuaternion(const float& w, const float& x, const float& y, const float& z):
		x(x), y(y), z(z), w(w)
	{ }

	CQuaternion::CQuaternion(const CQuaternion& q):
		x(q.x), y(q.y), z(q.z), w(q.w)
	{ }

	CQuaternion& CQuaternion::Add(const CQuaternion& q)
	{
		this->w += q.element[0];
		this->x += q.element[1];
		this->y += q.element[2];
		this->z += q.element[3];

		return(*this);
	}

	CQuaternion& CQuaternion::Multiply(const CQuaternion& q)
	{
		CQuaternion const p(*this);
		CQuaternion const q1(q);

		this->w = p.element[0] * q1.element[0] - p.element[1] * q1.element[1] - p.element[2] * q1.element[2] - p.element[3] * q1.element[3];
		this->x = p.element[0] * q1.element[1] + p.element[1] * q1.element[0] + p.element[2] * q1.element[3] - p.element[3] * q1.element[2];
		this->y = p.element[0] * q1.element[2] + p.element[2] * q1.element[0] + p.element[3] * q1.element[1] - p.element[1] * q1.element[3];
		this->z = p.element[0] * q1.element[3] + p.element[3] * q1.element[0] + p.element[1] * q1.element[2] - p.element[2] * q1.element[1];

		return (*this);
	}

	CQuaternion& CQuaternion::Multiply(const float& s)
	{
		this->w *= s;
		this->x *= s;
		this->y *= s;
		this->z *= s;

		return (*this);
	}

	CQuaternion& CQuaternion::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->w /= s;
		this->x /= s;
		this->y /= s;
		this->z /= s;

		return (*this);
	}

	CQuaternion& CQuaternion::operator=(const CQuaternion& q)
	{
		this->w = q.element[0];
		this->x = q.element[1];
		this->y = q.element[2];
		this->z = q.element[3];

		return (*this);
	}

	float& CQuaternion::operator[](int i)
	{
		assert((i >= 0) && (i < QUATERNION_SIZE));

		return element[i];
	}

	const float& CQuaternion::operator[](int i) const
	{
		assert((i >= 0) && (i < QUATERNION_SIZE));

		return element[i];
	}

	CQuaternion& CQuaternion::operator+=(const CQuaternion& q)
	{
		this->w += q.element[0];
		this->x += q.element[1];
		this->y += q.element[2];
		this->z += q.element[3];

		return (*this);
	}

	CQuaternion& CQuaternion::operator*=(const CQuaternion& q)
	{
		CQuaternion const p(*this);
		CQuaternion const q1(q);

		this->w = p.element[0] * q1.element[0] - p.element[1] * q1.element[1] - p.element[2] * q1.element[2] - p.element[3] * q1.element[3];
		this->x = p.element[0] * q1.element[1] + p.element[1] * q1.element[0] + p.element[2] * q1.element[3] - p.element[3] * q1.element[2];
		this->y = p.element[0] * q1.element[2] + p.element[2] * q1.element[0] + p.element[3] * q1.element[1] - p.element[1] * q1.element[3];
		this->z = p.element[0] * q1.element[3] + p.element[3] * q1.element[0] + p.element[1] * q1.element[2] - p.element[2] * q1.element[1];

		return (*this);
	}

	CQuaternion& CQuaternion::operator*=(const float& s)
	{
		this->w *= s;
		this->x *= s;
		this->y *= s;
		this->z *= s;

		return (*this);
	}

	CQuaternion& CQuaternion::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->w /= s;
		this->x /= s;
		this->y /= s;
		this->z /= s;

		return (*this);
	}

	CQuaternion operator+(const CQuaternion& q)
	{
		return q;
	}

	CQuaternion operator-(const CQuaternion& q)
	{
		return CQuaternion(-q.element[0], -q.element[1], -q.element[2], -q.element[3]);
	}

	CQuaternion operator+(const CQuaternion& q1, const CQuaternion& q2)
	{
		return CQuaternion(q1) += q2;
	}

	CQuaternion operator*(const CQuaternion& q1, const CQuaternion& q2)
	{
		return CQuaternion(q1) *= q2;
	}

	CVector3 operator*(const CQuaternion& q, const CVector3& v)
	{
		const CVector3 QuatVector(q.x, q.y, q.z);
		const CVector3 cqvv(Cross(QuatVector, v));
		const CVector3 ccqvvqv(Cross(QuatVector, cqvv));

		return v + ((cqvv * q.w) + ccqvvqv) * 2.0f;
	}

	CVector3 operator*(const CVector3& v, const CQuaternion& q)
	{
		return Inverse(q) * v;
	}

	CVector4 operator*(const CQuaternion& q, const CVector4& v)
	{
		return CVector4(q * CVector3(v), v.w);
	}

	CVector4 operator*(const CVector4& v, const CQuaternion& q)
	{
		return Inverse(q) * v;
	}

	CQuaternion operator*(const CQuaternion& q, const float& s)
	{
		return CQuaternion(q.element[0] * s,
						   q.element[1] * s,
						   q.element[2] * s,
						   q.element[3] * s);
	}

	CQuaternion operator*(const float& s, const CQuaternion& q)
	{
		return q * s;
	}

	CQuaternion operator/(const CQuaternion& q, const float& s)
	{
		assert(s != 0.0f);

		return CQuaternion(q.element[0] / s,
						   q.element[1] / s,
						   q.element[2] / s,
						   q.element[3] / s);
	}

	bool operator==(const CQuaternion& leftQuaternion, const CQuaternion& rightQuaternion)
	{
		return (leftQuaternion.element[0] == rightQuaternion.element[0]) &&
			   (leftQuaternion.element[1] == rightQuaternion.element[1]) &&
			   (leftQuaternion.element[2] == rightQuaternion.element[2]) &&
			   (leftQuaternion.element[3] == rightQuaternion.element[3]);
	}

	bool operator!=(const CQuaternion& leftQuaternion, const CQuaternion& rightQuaternion)
	{
		return (leftQuaternion.element[0] == rightQuaternion.element[0]) ||
			   (leftQuaternion.element[1] == rightQuaternion.element[1]) ||
			   (leftQuaternion.element[2] == rightQuaternion.element[2]) ||
			   (leftQuaternion.element[3] == rightQuaternion.element[3]);
	}

	std::ostream& operator<<(std::ostream& stream, const CQuaternion& q)
	{
		return stream << "Quaternion: (" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
	}
	/*--------------------------------------Quaternions--------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Vector2----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CVector2::CVector2():
		x(0.0f), y(0.0f)
	{ }

	CVector2::CVector2(const float& s):
		x(s), y(s)
	{ }

	CVector2::CVector2(const float& x, const float& y):
		x(x), y(y)
	{ }

	CVector2::CVector2(const CVector2& v):
		x(v.x), y(v.y)
	{ }

	CVector2::CVector2(const CVector3& v):
		x(v.x), y(v.y)
	{ }

	CVector2::CVector2(const CVector4& v):
		x(v.x), y(v.y)
	{ }

	CVector2& CVector2::Add(const CVector2& v)
	{
		this->element[0] += v.element[0];
		this->element[1] += v.element[1];

		return (*this);
	}

	CVector2& CVector2::Add(const float& s)
	{
		this->element[0] += s;
		this->element[1] += s;

		return (*this);
	}

	CVector2& CVector2::Subtract(const CVector2& v)
	{
		this->element[0] -= v.element[0];
		this->element[1] -= v.element[1];

		return (*this);
	}

	CVector2& CVector2::Subtract(const float& s)
	{
		this->element[0] -= s;
		this->element[1] -= s;

		return (*this);
	}

	CVector2& CVector2::Multiply(const CVector2& v)
	{
		this->element[0] *= v.element[0];
		this->element[1] *= v.element[1];

		return (*this);
	}

	CVector2& CVector2::Multiply(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;

		return (*this);
	}

	CVector2& CVector2::Divide(const CVector2& v)
	{
		assert(v != CVector2(0.0f));

		this->element[0] /= v.element[0];
		this->element[1] /= v.element[1];

		return (*this);
	}

	CVector2& CVector2::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;

		return (*this);
	}

	void CVector2::Clear()
	{
		this->element[0] = 0.0f;
		this->element[1] = 0.0f;
	}

	CVector2& CVector2::operator=(const CVector2& v)
	{
		this->element[0] = v.element[0];
		this->element[1] = v.element[1];

		return (*this);
	}

	float& CVector2::operator[](int i)
	{
		assert((i >= 0) && (i < VECTOR2_SIZE));

		return element[i];
	}

	const float& CVector2::operator[](int i) const
	{
		assert((i >= 0) && (i < VECTOR2_SIZE));

		return element[i];
	}

	CVector2& CVector2::operator+=(const CVector2& v)
	{
		this->element[0] += v.element[0];
		this->element[1] += v.element[1];

		return (*this);
	}

	CVector2& CVector2::operator+=(const float& s)
	{
		this->element[0] += s;
		this->element[1] += s;

		return (*this);
	}

	CVector2& CVector2::operator-=(const CVector2& v)
	{
		this->element[0] -= v.element[0];
		this->element[1] -= v.element[1];

		return (*this);
	}

	CVector2& CVector2::operator-=(const float& s)
	{
		this->element[0] -= s;
		this->element[1] -= s;

		return (*this);
	}

	CVector2& CVector2::operator*=(const CVector2& v)
	{
		this->element[0] *= v.element[0];
		this->element[1] *= v.element[1];

		return (*this);
	}

	CVector2& CVector2::operator*=(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;

		return (*this);
	}

	CVector2& CVector2::operator/=(const CVector2& v)
	{
		assert(v != CVector2(0.0f));

		this->element[0] /= v.element[0];
		this->element[1] /= v.element[1];

		return (*this);
	}

	CVector2& CVector2::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;

		return (*this);
	}

	CVector2& CVector2::operator++()
	{
		++this->element[0];
		++this->element[1];

		return (*this);
	}

	CVector2& CVector2::operator--()
	{
		--this->element[0];
		--this->element[1];

		return (*this);
	}

	CVector2 operator+(const CVector2& v)
	{
		return v;
	}

	CVector2 operator-(const CVector2& v)
	{
		return CVector2(-v.element[0], -v.element[1]);
	}

	CVector2 operator+(const CVector2& v, const float& s)
	{
		return CVector2(v.element[0] + s,
						v.element[1] + s);
	}

	CVector2 operator+(const float& s, const CVector2& v)
	{
		return CVector2(s + v.element[0],
						s + v.element[1]);
	}

	CVector2 operator+(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(v1.element[0] + v2.element[0],
						v1.element[1] + v2.element[1]);
	}

	CVector2 operator-(const CVector2& v, const float& s)
	{
		return CVector2(v.element[0] - s,
						v.element[1] - s);
	}

	CVector2 operator-(const float& s, const CVector2& v)
	{
		return CVector2(s - v.element[0],
						s - v.element[1]);
	}

	CVector2 operator-(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(v1.element[0] - v2.element[0],
						v1.element[1] - v2.element[1]);
	}

	CVector2 operator*(const CVector2& v, const float& s)
	{
		return CVector2(v.element[0] * s,
						v.element[1] * s);
	}

	CVector2 operator*(const float& s, const CVector2& v)
	{
		return CVector2(s * v.element[0],
						s * v.element[1]);
	}

	CVector2 operator*(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(v1.element[0] * v2.element[0],
						v1.element[1] * v2.element[1]);
	}

	CVector2 operator/(const CVector2& v, const float& s)
	{
		assert(s != 0.0f);

		return CVector2(v.element[0] / s,
						v.element[1] / s);
	}

	CVector2 operator/(const float& s, const CVector2& v)
	{
		assert(v != CVector2(0.0f));

		return CVector2(s / v.element[0],
						s / v.element[1]);
	}

	CVector2 operator/(const CVector2& v1, const CVector2& v2)
	{
		assert(v2 != CVector2(0.0f));

		return CVector2(v1.element[0] / v2.element[0],
						v1.element[1] / v2.element[1]);
	}

	bool operator==(const CVector2& leftVector, const CVector2& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]);
	}

	bool operator!=(const CVector2& leftVector, const CVector2& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]);
	}

	std::ostream& operator<<(std::ostream& stream, const CVector2& v)
	{
		return stream << "Vector2: (" << v.x << ", " << v.y << ")";
	}
	/*----------------------------------------Vector2----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Vector3----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CVector3::CVector3():
		x(0.0f), y(0.0f), z(0.0f)
	{ }

	CVector3::CVector3(const float& s):
		x(s), y(s), z(s)
	{ }

	CVector3::CVector3(const float& x, const float& y, const float& z):
		x(x), y(y), z(z)
	{ }

	CVector3::CVector3(const CVector3& v):
		x(v.x), y(v.y), z(v.z)
	{ }

	CVector3::CVector3(const CVector2& v):
		x(v.x), y(v.y), z(0.0f)
	{ }

	CVector3::CVector3(const CVector2& v, const float& s):
		x(v.x), y(v.y), z(s)
	{ }

	CVector3::CVector3(const CVector4& v):
		x(v.x), y(v.y), z(v.z)
	{ }

	CVector3& CVector3::Add(const CVector3& v)
	{
		this->element[0] += v.element[0];
		this->element[1] += v.element[1];
		this->element[2] += v.element[2];

		return (*this);
	}

	CVector3& CVector3::Add(const float& s)
	{
		this->element[0] += s;
		this->element[1] += s;
		this->element[2] += s;

		return (*this);
	}

	CVector3& CVector3::Subtract(const CVector3& v)
	{
		this->element[0] -= v.element[0];
		this->element[1] -= v.element[1];
		this->element[2] -= v.element[2];

		return (*this);
	}

	CVector3& CVector3::Subtract(const float& s)
	{
		this->element[0] -= s;
		this->element[1] -= s;
		this->element[2] -= s;

		return (*this);
	}

	CVector3& CVector3::Multiply(const CVector3& v)
	{
		this->element[0] *= v.element[0];
		this->element[1] *= v.element[1];
		this->element[2] *= v.element[2];

		return (*this);
	}

	CVector3& CVector3::Multiply(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;
		this->element[2] *= s;

		return (*this);
	}

	CVector3& CVector3::Divide(const CVector3& v)
	{
		assert(v != CVector3(0.0f));

		this->element[0] /= v.element[0];
		this->element[1] /= v.element[1];
		this->element[2] /= v.element[2];

		return (*this);
	}

	CVector3& CVector3::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;
		this->element[2] /= s;

		return (*this);
	}

	void CVector3::Clear()
	{
		this->element[0] = 0.0f;
		this->element[1] = 0.0f;
		this->element[2] = 0.0f;
	}

	CVector3& CVector3::operator=(const CVector3& v)
	{
		this->element[0] = v.element[0];
		this->element[1] = v.element[1];
		this->element[2] = v.element[2];

		return (*this);
	}

	float& CVector3::operator[](int i)
	{
		assert((i >= 0) && (i < VECTOR3_SIZE));

		return element[i];
	}

	const float& CVector3::operator[](int i) const
	{
		assert((i >= 0) && (i < VECTOR3_SIZE));

		return element[i];
	}

	CVector3& CVector3::operator+=(const CVector3& v)
	{
		this->element[0] += v.element[0];
		this->element[1] += v.element[1];
		this->element[2] += v.element[2];

		return (*this);
	}

	CVector3& CVector3::operator+=(const float& s)
	{
		this->element[0] += s;
		this->element[1] += s;
		this->element[2] += s;

		return (*this);
	}

	CVector3& CVector3::operator-=(const CVector3& v)
	{
		this->element[0] -= v.element[0];
		this->element[1] -= v.element[1];
		this->element[2] -= v.element[2];

		return (*this);
	}

	CVector3& CVector3::operator-=(const float& s)
	{
		this->element[0] -= s;
		this->element[1] -= s;
		this->element[2] -= s;

		return (*this);
	}

	CVector3& CVector3::operator*=(const CVector3& v)
	{
		this->element[0] *= v.element[0];
		this->element[1] *= v.element[1];
		this->element[2] *= v.element[2];

		return (*this);
	}

	CVector3& CVector3::operator*=(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;
		this->element[2] *= s;

		return (*this);
	}

	CVector3& CVector3::operator/=(const CVector3& v)
	{
		assert(v != CVector3(0.0f));

		this->element[0] /= v.element[0];
		this->element[1] /= v.element[1];
		this->element[2] /= v.element[2];

		return (*this);
	}

	CVector3& CVector3::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;
		this->element[2] /= s;

		return (*this);
	}

	CVector3& CVector3::operator++()
	{
		++this->element[0];
		++this->element[1];
		++this->element[2];

		return (*this);
	}

	CVector3& CVector3::operator--()
	{
		--this->element[0];
		--this->element[1];
		--this->element[2];

		return (*this);
	}

	CVector3 operator+(const CVector3& v)
	{
		return v;
	}

	CVector3 operator-(const CVector3& v)
	{
		return CVector3(-v.element[0], -v.element[1], -v.element[2]);
	}

	CVector3 operator+(const CVector3& v, const float& s)
	{
		return CVector3(v.element[0] + s,
						v.element[1] + s,
						v.element[2] + s);
	}

	CVector3 operator+(const float& s, const CVector3& v)
	{
		return CVector3(s + v.element[0],
						s + v.element[1],
						s + v.element[2]);
	}

	CVector3 operator+(const CVector3& v1, const CVector3& v2)
	{
		return CVector3(v1.element[0] + v2.element[0],
						v1.element[1] + v2.element[1],
						v1.element[2] + v2.element[2]);
	}

	CVector3 operator-(const CVector3& v, const float& s)
	{
		return CVector3(v.element[0] - s,
						v.element[1] - s,
						v.element[2] - s);
	}

	CVector3 operator-(const float& s, const CVector3& v)
	{
		return CVector3(s - v.element[0],
						s - v.element[1],
						s - v.element[2]);
	}

	CVector3 operator-(const CVector3& v1, const CVector3& v2)
	{
		return CVector3(v1.element[0] - v2.element[0],
						v1.element[1] - v2.element[1],
						v1.element[2] - v2.element[2]);
	}

	CVector3 operator*(const CVector3& v, const float& s)
	{
		return CVector3(v.element[0] * s,
						v.element[1] * s,
						v.element[2] * s);
	}

	CVector3 operator*(const float& s, const CVector3& v)
	{
		return CVector3(s * v.element[0],
						s * v.element[1],
						s * v.element[2]);
	}

	CVector3 operator*(const CVector3& v1, const CVector3& v2)
	{
		return CVector3(v1.element[0] * v2.element[0],
						v1.element[1] * v2.element[1],
						v1.element[2] * v2.element[2]);
	}

	CVector3 operator/(const CVector3& v, const float& s)
	{
		assert(s != 0.0f);

		return CVector3(v.element[0] / s,
						v.element[1] / s,
						v.element[2] / s);
	}

	CVector3 operator/(const float& s, const CVector3& v)
	{
		assert(v != CVector3(0.0f));

		return CVector3(s / v.element[0],
						s / v.element[1],
						s / v.element[2]);
	}

	CVector3 operator/(const CVector3& v1, const CVector3& v2)
	{
		assert(v2 != CVector3(0.0f));

		return CVector3(v1.element[0] / v2.element[0],
						v1.element[1] / v2.element[1],
						v1.element[2] / v2.element[2]);
	}

	bool operator==(const CVector3& leftVector, const CVector3& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]) &&
			   (leftVector.element[2] == rightVector.element[2]);
	}

	bool operator!=(const CVector3& leftVector, const CVector3& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]) ||
			   (leftVector.element[2] != rightVector.element[2]);
	}

	std::ostream& operator<<(std::ostream& stream, const CVector3& v)
	{
		return stream << "Vector3: (" << v.x << ", " << v.y << ", " << v.z << ")";
	}
	/*----------------------------------------Vector3----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Vector4----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CVector4::CVector4():
		x(0.0f), y(0.0f), z(0.0f), w(0.0f)
	{ }

	CVector4::CVector4(const float& s):
		x(s), y(s), z(s), w(s)
	{ }

	CVector4::CVector4(const float& x, const float& y, const float& z, const float& w):
		x(x), y(y), z(z), w(w)
	{ }

	CVector4::CVector4(const CVector4& v):
		x(v.x), y(v.y), z(v.z), w(v.w)
	{ }

	CVector4::CVector4(const CVector2& v):
		x(v.x), y(v.y), z(0.0f), w(0.0f)
	{ }

	CVector4::CVector4(const CVector2& v, const float& s):
		x(v.x), y(v.y), z(s), w(s)
	{ }

	CVector4::CVector4(const CVector2& v, const float& s1, const float& s2):
		x(v.x), y(v.y), z(s1), w(s2)
	{ }

	CVector4::CVector4(const CVector3& v):
		x(v.x), y(v.y), z(v.z), w(0.0f)
	{ }

	CVector4::CVector4(const CVector3& v, const float& s):
		x(v.x), y(v.y), z(v.z), w(s)
	{ }

	CVector4& CVector4::Add(const CVector4& v)
	{
		this->element[0] += v.element[0];
		this->element[1] += v.element[1];
		this->element[2] += v.element[2];
		this->element[3] += v.element[3];

		return (*this);
	}

	CVector4& CVector4::Add(const float& s)
	{
		this->element[0] += s;
		this->element[1] += s;
		this->element[2] += s;
		this->element[3] += s;

		return (*this);
	}

	CVector4& CVector4::Subtract(const CVector4& v)
	{
		this->element[0] -= v.element[0];
		this->element[1] -= v.element[1];
		this->element[2] -= v.element[2];
		this->element[3] -= v.element[3];

		return (*this);
	}

	CVector4& CVector4::Subtract(const float& s)
	{
		this->element[0] -= s;
		this->element[1] -= s;
		this->element[2] -= s;
		this->element[3] -= s;

		return (*this);
	}

	CVector4& CVector4::Multiply(const CVector4& v)
	{
		this->element[0] *= v.element[0];
		this->element[1] *= v.element[1];
		this->element[2] *= v.element[2];
		this->element[3] *= v.element[3];

		return (*this);
	}

	CVector4& CVector4::Multiply(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;
		this->element[2] *= s;
		this->element[3] *= s;

		return (*this);
	}

	CVector4& CVector4::Divide(const CVector4& v)
	{
		assert(v != CVector4(0.0f));

		this->element[0] /= v.element[0];
		this->element[1] /= v.element[1];
		this->element[2] /= v.element[2];
		this->element[3] /= v.element[3];

		return (*this);
	}

	CVector4& CVector4::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;
		this->element[2] /= s;
		this->element[3] /= s;

		return (*this);
	}

	void CVector4::Clear()
	{
		this->element[0] = 0.0f;
		this->element[1] = 0.0f;
		this->element[2] = 0.0f;
		this->element[3] = 0.0f;
	}

	CVector4& CVector4::operator=(const CVector4& v)
	{
		this->element[0] = v.element[0];
		this->element[1] = v.element[1];
		this->element[2] = v.element[2];
		this->element[3] = v.element[3];

		return (*this);
	}

	float& CVector4::operator[](int i)
	{
		assert((i >= 0) && (i < VECTOR4_SIZE));

		return element[i];
	}

	const float& CVector4::operator[](int i) const
	{
		assert((i >= 0) && (i < VECTOR4_SIZE));

		return element[i];
	}

	CVector4& CVector4::operator+=(const CVector4& v)
	{
		this->element[0] += v.element[0];
		this->element[1] += v.element[1];
		this->element[2] += v.element[2];
		this->element[3] += v.element[3];

		return (*this);
	}

	CVector4& CVector4::operator+=(const float& s)
	{
		this->element[0] += s;
		this->element[1] += s;
		this->element[2] += s;
		this->element[3] += s;

		return (*this);
	}

	CVector4& CVector4::operator-=(const CVector4& v)
	{
		this->element[0] -= v.element[0];
		this->element[1] -= v.element[1];
		this->element[2] -= v.element[2];
		this->element[3] -= v.element[3];

		return (*this);
	}

	CVector4& CVector4::operator-=(const float& s)
	{
		this->element[0] -= s;
		this->element[1] -= s;
		this->element[2] -= s;
		this->element[3] -= s;

		return (*this);
	}

	CVector4& CVector4::operator*=(const CVector4& v)
	{
		this->element[0] *= v.element[0];
		this->element[1] *= v.element[1];
		this->element[2] *= v.element[2];
		this->element[3] *= v.element[3];

		return (*this);
	}

	CVector4& CVector4::operator*=(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;
		this->element[2] *= s;
		this->element[3] *= s;

		return (*this);
	}

	CVector4& CVector4::operator/=(const CVector4& v)
	{
		assert(v != CVector4(0.0f));

		this->element[0] /= v.element[0];
		this->element[1] /= v.element[1];
		this->element[2] /= v.element[2];
		this->element[3] /= v.element[3];

		return (*this);
	}

	CVector4& CVector4::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;
		this->element[2] /= s;
		this->element[3] /= s;

		return (*this);
	}

	CVector4& CVector4::operator++()
	{
		++this->element[0];
		++this->element[1];
		++this->element[2];
		++this->element[3];

		return (*this);
	}

	CVector4& CVector4::operator--()
	{
		--this->element[0];
		--this->element[1];
		--this->element[2];
		--this->element[3];

		return (*this);
	}

	CVector4 operator+(const CVector4& v)
	{
		return v;
	}

	CVector4 operator-(const CVector4& v)
	{
		return CVector4(-v.element[0], -v.element[1], -v.element[2], -v.element[3]);
	}

	CVector4 operator+(const CVector4& v, const float& s)
	{
		return CVector4(v.element[0] + s,
						v.element[1] + s,
						v.element[2] + s,
						v.element[3] + s);
	}

	CVector4 operator+(const float& s, const CVector4& v)
	{
		return CVector4(s + v.element[0],
						s + v.element[1],
						s + v.element[2],
						s + v.element[3]);
	}

	CVector4 operator+(const CVector4& v1, const CVector4& v2)
	{
		return CVector4(v1.element[0] + v2.element[0],
						v1.element[1] + v2.element[1],
						v1.element[2] + v2.element[2],
						v1.element[3] + v2.element[3]);
	}

	CVector4 operator-(const CVector4& v, const float& s)
	{
		return CVector4(v.element[0] - s,
						v.element[1] - s,
						v.element[2] - s,
						v.element[3] - s);
	}

	CVector4 operator-(const float& s, const CVector4& v)
	{
		return CVector4(s - v.element[0],
						s - v.element[1],
						s - v.element[2],
						s - v.element[3]);
	}

	CVector4 operator-(const CVector4& v1, const CVector4& v2)
	{
		return CVector4(v1.element[0] - v2.element[0],
						v1.element[1] - v2.element[1],
						v1.element[2] - v2.element[2],
						v1.element[3] - v2.element[3]);
	}

	CVector4 operator*(const CVector4& v, const float& s)
	{
		return CVector4(v.element[0] * s,
						v.element[1] * s,
						v.element[2] * s,
						v.element[3] * s);
	}

	CVector4 operator*(const float& s, const CVector4& v)
	{
		return CVector4(s * v.element[0],
						s * v.element[1],
						s * v.element[2],
						s * v.element[3]);
	}

	CVector4 operator*(const CVector4& v1, const CVector4& v2)
	{
		return CVector4(v1.element[0] * v2.element[0],
						v1.element[1] * v2.element[1],
						v1.element[2] * v2.element[2],
						v1.element[3] * v2.element[3]);
	}

	CVector4 operator/(const CVector4& v, const float& s)
	{
		assert(s != 0.0f);

		return CVector4(v.element[0] / s,
						v.element[1] / s,
						v.element[2] / s,
						v.element[3] / s);
	}

	CVector4 operator/(const float& s, const CVector4& v)
	{
		assert(v != CVector4(0.0f));

		return CVector4(s / v.element[0],
						s / v.element[1],
						s / v.element[2],
						s / v.element[3]);
	}

	CVector4 operator/(const CVector4& v1, const CVector4& v2)
	{
		assert(v2 != CVector4(0.0f));

		return CVector4(v1.element[0] / v2.element[0],
						v1.element[1] / v2.element[1],
						v1.element[2] / v2.element[2],
						v1.element[3] / v2.element[3]);
	}

	bool operator==(const CVector4& leftVector, const CVector4& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]) &&
			   (leftVector.element[2] == rightVector.element[2]) &&
			   (leftVector.element[3] == rightVector.element[3]);
	}

	bool operator!=(const CVector4& leftVector, const CVector4& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]) ||
			   (leftVector.element[2] != rightVector.element[2]) ||
			   (leftVector.element[3] != rightVector.element[3]);
	}

	std::ostream& operator<<(std::ostream& stream, const CVector4& v)
	{
		return stream << "Vector4: (" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
	}
	/*----------------------------------------Vector2----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Matrix2----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CMatrix2x2::CMatrix2x2()
	{
		this->_1D[0] = 1.0f;
		this->_1D[1] = 0.0f;

		this->_1D[2] = 0.0f;
		this->_1D[3] = 1.0f;
	}

	CMatrix2x2::CMatrix2x2(const float& s)
	{
		this->_1D[0] = s;
		this->_1D[1] = 0.0f;

		this->_1D[2] = 0.0f;
		this->_1D[3] = s;
	}

	CMatrix2x2::CMatrix2x2(const float& r11, const float& r12,
						   const float& r21, const float& r22)
	{
		this->_1D[0] = r11;
		this->_1D[1] = r12;

		this->_1D[2] = r21;
		this->_1D[3] = r22;
	}

	CMatrix2x2::CMatrix2x2(const CVector2& col1, const CVector2& col2)
	{
		this->column[0] = col1;
		this->column[1] = col2;
	}

	CMatrix2x2::CMatrix2x2(const CMatrix2x2& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
	}

	CMatrix2x2::CMatrix2x2(const CMatrix3x3& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
	}

	CMatrix2x2::CMatrix2x2(const CMatrix4x4& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
	}

	CMatrix2x2& CMatrix2x2::Add(const CMatrix2x2& m)
	{
		this->column[0] += m.column[0];
		this->column[1] += m.column[1];

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::Add(const float& s)
	{
		this->column[0] += s;
		this->column[1] += s;

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::Subtract(const CMatrix2x2& m)
	{
		this->column[0] -= m.column[0];
		this->column[1] -= m.column[1];

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::Subtract(const float& s)
	{
		this->column[0] -= s;
		this->column[1] -= s;

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::Multiply(const CMatrix2x2& m)
	{
		return (*this = *this * m);
	}

	CMatrix2x2& CMatrix2x2::Multiply(const float& s)
	{
		this->column[0] *= s;
		this->column[1] *= s;

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::Divide(const CMatrix2x2& m)
	{
		assert(m != CMatrix2x2(0.0f));

		return (*this = *this * Inverse(m));
	}

	CMatrix2x2& CMatrix2x2::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->column[0] /= s;
		this->column[1] /= s;

		return (*this);
	}

	CMatrix2x2 CMatrix2x2::Identity()
	{
		return CMatrix2x2();
	}

	void CMatrix2x2::Zero()
	{
		this->column[0] = 0.0f;
		this->column[1] = 0.0f;
	}

	CMatrix2x2& CMatrix2x2::operator=(const CMatrix2x2& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];

		return (*this);
	}

	float& CMatrix2x2::operator[](int i)
	{
		assert((i >= 0) && (i < MATRIX_2X2_SIZE));

		return _1D[i];
	}

	const float& CMatrix2x2::operator[](int i) const
	{
		assert((i >= 0) && (i < MATRIX_2X2_SIZE));

		return _1D[i];
	}

	CMatrix2x2& CMatrix2x2::operator+=(const CMatrix2x2& m)
	{
		this->column[0] += m.column[0];
		this->column[1] += m.column[1];

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::operator+=(const float& s)
	{
		this->column[0] += s;
		this->column[1] += s;

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::operator-=(const CMatrix2x2& m)
	{
		this->column[0] -= m.column[0];
		this->column[1] -= m.column[1];

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::operator-=(const float& s)
	{
		this->column[0] -= s;
		this->column[1] -= s;

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::operator*=(const CMatrix2x2& m)
	{
		return (*this = *this * m);
	}

	CMatrix2x2& CMatrix2x2::operator*=(const float& s)
	{
		this->column[0] *= s;
		this->column[1] *= s;

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::operator/=(const CMatrix2x2& m)
	{
		assert(m != CMatrix2x2(0.0f));

		return (*this = *this * Inverse(m));
	}

	CMatrix2x2& CMatrix2x2::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->column[0] /= s;
		this->column[1] /= s;

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::operator++()
	{
		++this->column[0];
		++this->column[1];

		return (*this);
	}

	CMatrix2x2& CMatrix2x2::operator--()
	{
		--this->column[0];
		--this->column[1];

		return (*this);
	}

	CMatrix2x2 operator+(const CMatrix2x2& m)
	{
		return m;
	}

	CMatrix2x2 operator-(const CMatrix2x2& m)
	{
		return CMatrix2x2(-m.column[0], -m.column[1]);
	}

	CMatrix2x2 operator+(const CMatrix2x2& m, const float& s)
	{
		return CMatrix2x2(m.column[0] + s,
						  m.column[1] + s);
	}

	CMatrix2x2 operator+(const float& s, const CMatrix2x2& m)
	{
		return CMatrix2x2(s + m.column[0],
						  s + m.column[1]);
	}

	CMatrix2x2 operator+(const CMatrix2x2& m1, const CMatrix2x2& m2)
	{
		return CMatrix2x2(m1.column[0] + m2.column[0],
						  m1.column[1] + m2.column[1]);
	}

	CMatrix2x2 operator-(const CMatrix2x2& m, const float& s)
	{
		return CMatrix2x2(m.column[0] - s,
						  m.column[1] - s);
	}

	CMatrix2x2 operator-(const float& s, const CMatrix2x2& m)
	{
		return CMatrix2x2(s - m.column[0],
						  s - m.column[1]);
	}

	CMatrix2x2 operator-(const CMatrix2x2& m1, const CMatrix2x2& m2)
	{
		return CMatrix2x2(m1.column[0] - m2.column[0],
						  m1.column[1] - m2.column[1]);
	}

	CMatrix2x2 operator*(const CMatrix2x2& m, const float& s)
	{
		return CMatrix2x2(m.column[0] * s,
						  m.column[1] * s);
	}

	CMatrix2x2 operator*(const float& s, const CMatrix2x2& m)
	{
		return CMatrix2x2(s * m.column[0],
						  s * m.column[1]);
	}

	CMatrix2x2 operator*(const CMatrix2x2& m1, const CMatrix2x2& m2)
	{
		return CMatrix2x2(m1._1D[0] * m2._1D[0] + m1._1D[2] * m2._1D[1],
						  m1._1D[1] * m2._1D[0] + m1._1D[3] * m2._1D[1],
						  m1._1D[0] * m2._1D[2] + m1._1D[2] * m2._1D[3],
						  m1._1D[1] * m2._1D[2] + m1._1D[3] * m2._1D[3]);
	}

	CMatrix2x2 operator/(const CMatrix2x2& m, const float& s)
	{
		assert(s != 0.0f);

		return CMatrix2x2(m.column[0] / s,
						  m.column[1] / s);
	}

	CMatrix2x2 operator/(const float& s, const CMatrix2x2& m)
	{
		assert(m != CMatrix2x2(0.0f));

		return CMatrix2x2(s / m.column[0],
						  s / m.column[1]);
	}

	CMatrix2x2 operator/(const CMatrix2x2& m1, const CMatrix2x2& m2)
	{
		CMatrix2x2 result(m1);
		return result /= m2;
	}

	bool operator==(const CMatrix2x2& leftMatrix, const CMatrix2x2& rightMatrix)
	{
		return (leftMatrix.column[0] == rightMatrix.column[0]) &&
			   (leftMatrix.column[1] == rightMatrix.column[1]);
	}

	bool operator!=(const CMatrix2x2& leftMatrix, const CMatrix2x2& rightMatrix)
	{
		return (leftMatrix.column[0] != rightMatrix.column[0]) ||
			   (leftMatrix.column[1] != rightMatrix.column[1]);
	}

	std::ostream& operator<<(std::ostream& stream, const CMatrix2x2& m)
	{
		return stream << "Matrix2x2:" << std::endl
					  << "|" << m._1D[0] << " " << m._1D[2] << "|" << std::endl
					  << "|" << m._1D[1] << " " << m._1D[3] << "|";
	}
	/*----------------------------------------Matrix2----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/
	
	/*----------------------------------------Matrix3----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CMatrix3x3::CMatrix3x3()
	{
		this->_1D[0] = 1.0f;
		this->_1D[1] = 0.0f;
		this->_1D[2] = 0.0f;

		this->_1D[3] = 0.0f;
		this->_1D[4] = 1.0f;
		this->_1D[5] = 0.0f;

		this->_1D[6] = 0.0f;
		this->_1D[7] = 0.0f;
		this->_1D[8] = 1.0f;
	}

	CMatrix3x3::CMatrix3x3(const float& s)
	{
		this->_1D[0] = s;
		this->_1D[1] = 0.0f;
		this->_1D[2] = 0.0f;

		this->_1D[3] = 0.0f;
		this->_1D[4] = s;
		this->_1D[5] = 0.0f;

		this->_1D[6] = 0.0f;
		this->_1D[7] = 0.0f;
		this->_1D[8] = s;
	}

	CMatrix3x3::CMatrix3x3(const float& r11, const float& r12, const float& r13,
						   const float& r21, const float& r22, const float& r23,
						   const float& r31, const float& r32, const float& r33)
	{
		this->_1D[0] = r11;
		this->_1D[1] = r12;
		this->_1D[2] = r13;

		this->_1D[3] = r21;
		this->_1D[4] = r22;
		this->_1D[5] = r23;

		this->_1D[6] = r31;
		this->_1D[7] = r32;
		this->_1D[8] = r33;
	}

	CMatrix3x3::CMatrix3x3(const CVector3& col1, const CVector3& col2, const CVector3& col3)
	{
		this->column[0] = col1;
		this->column[1] = col2;
		this->column[2] = col3;
	}

	CMatrix3x3::CMatrix3x3(const CMatrix3x3& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = m.column[2];
	}

	CMatrix3x3::CMatrix3x3(const CMatrix2x2& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = 0.0f;

		this->_1D[8] = 1.0f;
	}

	CMatrix3x3::CMatrix3x3(const CMatrix4x4& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = m.column[2];
	}

	CMatrix3x3& CMatrix3x3::Add(const CMatrix3x3& m)
	{
		this->column[0] += m.column[0];
		this->column[1] += m.column[1];
		this->column[2] += m.column[2];

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::Add(const float& s)
	{
		this->column[0] += s;
		this->column[1] += s;
		this->column[2] += s;

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::Subtract(const CMatrix3x3& m)
	{
		this->column[0] -= m.column[0];
		this->column[1] -= m.column[1];
		this->column[2] -= m.column[2];

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::Subtract(const float& s)
	{
		this->column[0] -= s;
		this->column[1] -= s;
		this->column[2] -= s;

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::Multiply(const CMatrix3x3& m)
	{
		return (*this = *this * m);
	}

	CMatrix3x3& CMatrix3x3::Multiply(const float& s)
	{
		this->column[0] *= s;
		this->column[1] *= s;
		this->column[2] *= s;

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::Divide(const CMatrix3x3& m)
	{
		assert(m != CMatrix3x3(0.0f));

		return (*this = *this * Inverse(m));
	}

	CMatrix3x3& CMatrix3x3::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->column[0] /= s;
		this->column[1] /= s;
		this->column[2] /= s;

		return (*this);
	}

	CMatrix3x3 CMatrix3x3::Identity()
	{
		return CMatrix3x3();
	}

	void CMatrix3x3::Zero()
	{
		this->column[0] = 0.0f;
		this->column[1] = 0.0f;
		this->column[2] = 0.0f;
	}

	CMatrix3x3& CMatrix3x3::operator=(const CMatrix3x3& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = m.column[2];

		return (*this);
	}

	float& CMatrix3x3::operator[](int i)
	{
		assert((i >= 0) && (i < MATRIX_3X3_SIZE));

		return _1D[i];
	}

	const float& CMatrix3x3::operator[](int i) const
	{
		assert((i >= 0) && (i < MATRIX_3X3_SIZE));

		return _1D[i];
	}

	CMatrix3x3& CMatrix3x3::operator+=(const CMatrix3x3& m)
	{
		this->column[0] += m.column[0];
		this->column[1] += m.column[1];
		this->column[2] += m.column[2];

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::operator+=(const float& s)
	{
		this->column[0] += s;
		this->column[1] += s;
		this->column[2] += s;

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::operator-=(const CMatrix3x3& m)
	{
		this->column[0] -= m.column[0];
		this->column[1] -= m.column[1];
		this->column[2] -= m.column[2];

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::operator-=(const float& s)
	{
		this->column[0] -= s;
		this->column[1] -= s;
		this->column[2] -= s;

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::operator*=(const CMatrix3x3& m)
	{
		return (*this = *this * m);
	}

	CMatrix3x3& CMatrix3x3::operator*=(const float& s)
	{
		this->column[0] *= s;
		this->column[1] *= s;
		this->column[2] *= s;

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::operator/=(const CMatrix3x3& m)
	{
		assert(m != CMatrix3x3(0.0f));

		return (*this = *this * Inverse(m));
	}

	CMatrix3x3& CMatrix3x3::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->column[0] /= s;
		this->column[1] /= s;
		this->column[2] /= s;

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::operator++()
	{
		++this->column[0];
		++this->column[1];
		++this->column[2];

		return (*this);
	}

	CMatrix3x3& CMatrix3x3::operator--()
	{
		--this->column[0];
		--this->column[1];
		--this->column[2];

		return (*this);
	}

	CMatrix3x3 operator+(const CMatrix3x3& m)
	{
		return m;
	}

	CMatrix3x3 operator-(const CMatrix3x3& m)
	{
		return CMatrix3x3(-m.column[0], -m.column[1], -m.column[2]);
	}

	CMatrix3x3 operator+(const CMatrix3x3& m, const float& s)
	{
		return CMatrix3x3(m.column[0] + s,
						  m.column[1] + s,
						  m.column[2] + s);
	}

	CMatrix3x3 operator+(const float& s, const CMatrix3x3& m)
	{
		return CMatrix3x3(s + m.column[0],
						  s + m.column[1],
						  s + m.column[2]);
	}

	CMatrix3x3 operator+(const CMatrix3x3& m1, const CMatrix3x3& m2)
	{
		return CMatrix3x3(m1.column[0] + m2.column[0],
						  m1.column[1] + m2.column[1],
						  m1.column[2] + m2.column[2]);
	}

	CMatrix3x3 operator-(const CMatrix3x3& m, const float& s)
	{
		return CMatrix3x3(m.column[0] - s,
						  m.column[1] - s,
						  m.column[2] - s);
	}

	CMatrix3x3 operator-(const float& s, const CMatrix3x3& m)
	{
		return CMatrix3x3(s - m.column[0],
						  s - m.column[1],
						  s - m.column[2]);
	}

	CMatrix3x3 operator-(const CMatrix3x3& m1, const CMatrix3x3& m2)
	{
		return CMatrix3x3(m1.column[0] - m2.column[0],
						  m1.column[1] - m2.column[1],
						  m1.column[2] - m2.column[2]);
	}

	CMatrix3x3 operator*(const CMatrix3x3& m, const float& s)
	{
		return CMatrix3x3(m.column[0] * s,
						  m.column[1] * s,
						  m.column[2] * s);
	}

	CMatrix3x3 operator*(const float& s, const CMatrix3x3& m)
	{
		return CMatrix3x3(s * m.column[0],
						  s * m.column[1],
						  s * m.column[2]);
	}

	CMatrix3x3 operator*(const CMatrix3x3& m1, const CMatrix3x3& m2)
	{
		return CMatrix3x3(m1._1D[0] * m2._1D[0] + m1._1D[3] * m2._1D[1] + m1._1D[6] * m2._1D[2],
						  m1._1D[1] * m2._1D[0] + m1._1D[4] * m2._1D[1] + m1._1D[7] * m2._1D[2],
						  m1._1D[2] * m2._1D[0] + m1._1D[5] * m2._1D[1] + m1._1D[8] * m2._1D[2],
						  m1._1D[0] * m2._1D[3] + m1._1D[3] * m2._1D[4] + m1._1D[6] * m2._1D[5],
						  m1._1D[1] * m2._1D[3] + m1._1D[4] * m2._1D[4] + m1._1D[7] * m2._1D[5],
						  m1._1D[2] * m2._1D[3] + m1._1D[5] * m2._1D[4] + m1._1D[8] * m2._1D[5],
						  m1._1D[0] * m2._1D[6] + m1._1D[3] * m2._1D[7] + m1._1D[6] * m2._1D[8],
						  m1._1D[1] * m2._1D[6] + m1._1D[4] * m2._1D[7] + m1._1D[7] * m2._1D[8],
						  m1._1D[2] * m2._1D[6] + m1._1D[5] * m2._1D[7] + m1._1D[8] * m2._1D[8]);
	}

	CMatrix3x3 operator/(const CMatrix3x3& m, const float& s)
	{
		assert(s != 0.0f);

		return CMatrix3x3(m.column[0] / s,
						  m.column[1] / s,
						  m.column[2] / s);
	}

	CMatrix3x3 operator/(const float& s, const CMatrix3x3& m)
	{
		assert(m != CMatrix3x3(0.0f));

		return CMatrix3x3(s / m.column[0],
						  s / m.column[1],
						  s / m.column[2]);
	}

	CMatrix3x3 operator/(const CMatrix3x3& m1, const CMatrix3x3& m2)
	{
		CMatrix3x3 result(m1);
		return result /= m2;
	}

	bool operator==(const CMatrix3x3& leftMatrix, const CMatrix3x3& rightMatrix)
	{
		return (leftMatrix.column[0] == rightMatrix.column[0]) &&
			   (leftMatrix.column[1] == rightMatrix.column[1]) &&
			   (leftMatrix.column[2] == rightMatrix.column[2]);
	}

	bool operator!=(const CMatrix3x3& leftMatrix, const CMatrix3x3& rightMatrix)
	{
		return (leftMatrix.column[0] != rightMatrix.column[0]) ||
			   (leftMatrix.column[1] != rightMatrix.column[1]) ||
			   (leftMatrix.column[2] != rightMatrix.column[2]);
	}

	std::ostream& operator<<(std::ostream& stream, const CMatrix3x3& m)
	{
		return stream << "Matrix3x3:" << std::endl
					  << "|" << m._1D[0] << " " << m._1D[3] << " " << m._1D[6] << "|" << std::endl
					  << "|" << m._1D[1] << " " << m._1D[4] << " " << m._1D[7] << "|" << std::endl
					  << "|" << m._1D[2] << " " << m._1D[5] << " " << m._1D[8] << "|";
	}
	/*----------------------------------------Matrix3----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*----------------------------------------Matrix4----------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CMatrix4x4::CMatrix4x4()
	{
		this->_1D[0]  = 1.0f;
		this->_1D[1]  = 0.0f;
		this->_1D[2]  = 0.0f;
		this->_1D[3]  = 0.0f;

		this->_1D[4]  = 0.0f;
		this->_1D[5]  = 1.0f;
		this->_1D[6]  = 0.0f;
		this->_1D[7]  = 0.0f;

		this->_1D[8]  = 0.0f;
		this->_1D[9]  = 0.0f;
		this->_1D[10] = 1.0f;
		this->_1D[11] = 0.0f;

		this->_1D[12] = 0.0f;
		this->_1D[13] = 0.0f;
		this->_1D[14] = 0.0f;
		this->_1D[15] = 1.0f;
	}

	CMatrix4x4::CMatrix4x4(const float& s)
	{
		this->_1D[0]  = s;
		this->_1D[1]  = 0.0f;
		this->_1D[2]  = 0.0f;
		this->_1D[3]  = 0.0f;

		this->_1D[4]  = 0.0f;
		this->_1D[5]  = s;
		this->_1D[6]  = 0.0f;
		this->_1D[7]  = 0.0f;

		this->_1D[8]  = 0.0f;
		this->_1D[9]  = 0.0f;
		this->_1D[10] = s;
		this->_1D[11] = 0.0f;

		this->_1D[12] = 0.0f;
		this->_1D[13] = 0.0f;
		this->_1D[14] = 0.0f;
		this->_1D[15] = s;
	}

	CMatrix4x4::CMatrix4x4(const float& r11, const float& r12, const float& r13, const float& r14,
						   const float& r21, const float& r22, const float& r23, const float& r24,
						   const float& r31, const float& r32, const float& r33, const float& r34,
						   const float& r41, const float& r42, const float& r43, const float& r44)
	{
		this->_1D[0]  = r11;
		this->_1D[1]  = r12;
		this->_1D[2]  = r13;
		this->_1D[3]  = r14;

		this->_1D[4]  = r21;
		this->_1D[5]  = r22;
		this->_1D[6]  = r23;
		this->_1D[7]  = r24;
		
		this->_1D[8]  = r31;
		this->_1D[9]  = r32;
		this->_1D[10] = r33;
		this->_1D[11] = r34;

		this->_1D[12] = r41;
		this->_1D[13] = r42;
		this->_1D[14] = r43;
		this->_1D[15] = r44;
	}

	CMatrix4x4::CMatrix4x4(const CVector4& col1, const CVector4& col2, const CVector4& col3, const CVector4& col4)
	{
		this->column[0] = col1;
		this->column[1] = col2;
		this->column[2] = col3;
		this->column[3] = col4;
	}

	CMatrix4x4::CMatrix4x4(const CMatrix4x4& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = m.column[2];
		this->column[3] = m.column[3];
	}

	CMatrix4x4::CMatrix4x4(const CMatrix2x2& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = 0.0f;
		this->column[3] = 0.0f;

		this->_1D[10] = 1.0f;
		this->_1D[15] = 1.0f;
	}

	CMatrix4x4::CMatrix4x4(const CMatrix3x3& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = m.column[2];
		this->column[3] = 0.0f;

		this->_1D[15] = 1.0f;
	}

	CMatrix4x4& CMatrix4x4::Add(const CMatrix4x4& m)
	{
		this->column[0] += m.column[0];
		this->column[1] += m.column[1];
		this->column[2] += m.column[2];
		this->column[3] += m.column[3];

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::Add(const float& s)
	{
		this->column[0] += s;
		this->column[1] += s;
		this->column[2] += s;
		this->column[3] += s;

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::Subtract(const CMatrix4x4& m)
	{
		this->column[0] -= m.column[0];
		this->column[1] -= m.column[1];
		this->column[2] -= m.column[2];
		this->column[3] -= m.column[3];

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::Subtract(const float& s)
	{
		this->column[0] -= s;
		this->column[1] -= s;
		this->column[2] -= s;
		this->column[3] -= s;

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::Multiply(const CMatrix4x4& m)
	{
		return (*this = *this * m);
	}

	CMatrix4x4& CMatrix4x4::Multiply(const float& s)
	{
		this->column[0] *= s;
		this->column[1] *= s;
		this->column[2] *= s;
		this->column[3] *= s;

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::Divide(const CMatrix4x4& m)
	{
		assert(m != CMatrix4x4(0.0f));

		return (*this = *this * Inverse(m));
	}

	CMatrix4x4& CMatrix4x4::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->column[0] /= s;
		this->column[1] /= s;
		this->column[2] /= s;
		this->column[3] /= s;

		return (*this);
	}

	CMatrix4x4 CMatrix4x4::Identity()
	{
		return CMatrix4x4();
	}

	void CMatrix4x4::Zero()
	{
		this->column[0] = 0.0f;
		this->column[1] = 0.0f;
		this->column[2] = 0.0f;
		this->column[3] = 0.0f;
	}

	CMatrix4x4& CMatrix4x4::operator=(const CMatrix4x4& m)
	{
		this->column[0] = m.column[0];
		this->column[1] = m.column[1];
		this->column[2] = m.column[2];
		this->column[3] = m.column[3];

		return (*this);
	}

	float& CMatrix4x4::operator[](int i)
	{
		assert((i >= 0) && (i < MATRIX_4X4_SIZE));

		return _1D[i];
	}

	const float& CMatrix4x4::operator[](int i) const
	{
		assert((i >= 0) && (i < MATRIX_4X4_SIZE));

		return _1D[i];
	}

	CMatrix4x4& CMatrix4x4::operator+=(const CMatrix4x4& m)
	{
		this->column[0] += m.column[0];
		this->column[1] += m.column[1];
		this->column[2] += m.column[2];
		this->column[3] += m.column[3];

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::operator+=(const float& s)
	{
		this->column[0] += s;
		this->column[1] += s;
		this->column[2] += s;
		this->column[3] += s;

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::operator-=(const CMatrix4x4& m)
	{
		this->column[0] -= m.column[0];
		this->column[1] -= m.column[1];
		this->column[2] -= m.column[2];
		this->column[3] -= m.column[3];

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::operator-=(const float& s)
	{
		this->column[0] -= s;
		this->column[1] -= s;
		this->column[2] -= s;
		this->column[3] -= s;

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::operator*=(const CMatrix4x4& m)
	{
		return (*this = *this * m);
	}

	CMatrix4x4& CMatrix4x4::operator*=(const float& s)
	{
		this->column[0] *= s;
		this->column[1] *= s;
		this->column[2] *= s;
		this->column[3] *= s;

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::operator/=(const CMatrix4x4& m)
	{
		assert(m != CMatrix4x4(0.0f));

		return (*this = *this * Inverse(m));
	}

	CMatrix4x4& CMatrix4x4::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->column[0] /= s;
		this->column[1] /= s;
		this->column[2] /= s;
		this->column[3] /= s;

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::operator++()
	{
		++this->column[0];
		++this->column[1];
		++this->column[2];
		++this->column[3];

		return (*this);
	}

	CMatrix4x4& CMatrix4x4::operator--()
	{
		--this->column[0];
		--this->column[1];
		--this->column[2];
		--this->column[3];

		return (*this);
	}

	CMatrix4x4 operator+(const CMatrix4x4& m)
	{
		return m;
	}

	CMatrix4x4 operator-(const CMatrix4x4& m)
	{
		return CMatrix4x4(-m.column[0], -m.column[1], -m.column[2], -m.column[2]);
	}

	CMatrix4x4 operator+(const CMatrix4x4& m, const float& s)
	{
		return CMatrix4x4(m.column[0] + s,
						  m.column[1] + s,
						  m.column[2] + s,
						  m.column[3] + s);
	}

	CMatrix4x4 operator+(const float& s, const CMatrix4x4& m)
	{
		return CMatrix4x4(s + m.column[0],
						  s + m.column[1],
						  s + m.column[2],
						  s + m.column[3]);
	}

	CMatrix4x4 operator+(const CMatrix4x4& m1, const CMatrix4x4& m2)
	{
		return CMatrix4x4(m1.column[0] + m2.column[0],
						  m1.column[1] + m2.column[1],
						  m1.column[2] + m2.column[2],
						  m1.column[3] + m2.column[3]);
	}

	CMatrix4x4 operator-(const CMatrix4x4& m, const float& s)
	{
		return CMatrix4x4(m.column[0] - s,
						  m.column[1] - s,
						  m.column[2] - s,
						  m.column[3] - s);
	}

	CMatrix4x4 operator-(const float& s, const CMatrix4x4& m)
	{
		return CMatrix4x4(s - m.column[0],
						  s - m.column[1],
						  s - m.column[2],
						  s - m.column[3]);
	}

	CMatrix4x4 operator-(const CMatrix4x4& m1, const CMatrix4x4& m2)
	{
		return CMatrix4x4(m1.column[0] - m2.column[0],
						  m1.column[1] - m2.column[1],
						  m1.column[2] - m2.column[2],
						  m1.column[3] - m2.column[3]);
	}

	CMatrix4x4 operator*(const CMatrix4x4& m, const float& s)
	{
		return CMatrix4x4(m.column[0] * s,
						  m.column[1] * s,
						  m.column[2] * s,
						  m.column[3] * s);
	}

	CMatrix4x4 operator*(const float& s, const CMatrix4x4& m)
	{
		return CMatrix4x4(s * m.column[0],
						  s * m.column[1],
						  s * m.column[2],
						  s * m.column[3]);
	}

	CMatrix4x4 operator*(const CMatrix4x4& m1, const CMatrix4x4& m2)
	{
		return CMatrix4x4(m1._1D[0] * m2._1D[0]	 + m1._1D[4] * m2._1D[1]  + m1._1D[8]  * m2._1D[2]  + m1._1D[12] * m2._1D[3],
						  m1._1D[1] * m2._1D[0]	 + m1._1D[5] * m2._1D[1]  + m1._1D[9]  * m2._1D[2]  + m1._1D[13] * m2._1D[3],
						  m1._1D[2] * m2._1D[0]	 + m1._1D[6] * m2._1D[1]  + m1._1D[10] * m2._1D[2]  + m1._1D[14] * m2._1D[3],
						  m1._1D[3] * m2._1D[0]	 + m1._1D[7] * m2._1D[1]  + m1._1D[11] * m2._1D[2]  + m1._1D[15] * m2._1D[3],
						  m1._1D[0] * m2._1D[4]	 + m1._1D[4] * m2._1D[5]  + m1._1D[8]  * m2._1D[6]  + m1._1D[12] * m2._1D[7],
						  m1._1D[1] * m2._1D[4]	 + m1._1D[5] * m2._1D[5]  + m1._1D[9]  * m2._1D[6]  + m1._1D[13] * m2._1D[7],
						  m1._1D[2] * m2._1D[4]	 + m1._1D[6] * m2._1D[5]  + m1._1D[10] * m2._1D[6]  + m1._1D[14] * m2._1D[7],
						  m1._1D[3] * m2._1D[4]	 + m1._1D[7] * m2._1D[5]  + m1._1D[11] * m2._1D[6]  + m1._1D[15] * m2._1D[7],
						  m1._1D[0] * m2._1D[8]	 + m1._1D[4] * m2._1D[9]  + m1._1D[8]  * m2._1D[10] + m1._1D[12] * m2._1D[11],
						  m1._1D[1] * m2._1D[8]	 + m1._1D[5] * m2._1D[9]  + m1._1D[9]  * m2._1D[10] + m1._1D[13] * m2._1D[11],
						  m1._1D[2] * m2._1D[8]	 + m1._1D[6] * m2._1D[9]  + m1._1D[10] * m2._1D[10] + m1._1D[14] * m2._1D[11],
						  m1._1D[3] * m2._1D[8]	 + m1._1D[7] * m2._1D[9]  + m1._1D[11] * m2._1D[10] + m1._1D[15] * m2._1D[11],
						  m1._1D[0] * m2._1D[12] + m1._1D[4] * m2._1D[13] + m1._1D[8]  * m2._1D[14] + m1._1D[12] * m2._1D[15],
						  m1._1D[1] * m2._1D[12] + m1._1D[5] * m2._1D[13] + m1._1D[9]  * m2._1D[14] + m1._1D[13] * m2._1D[15],
						  m1._1D[2] * m2._1D[12] + m1._1D[6] * m2._1D[13] + m1._1D[10] * m2._1D[14] + m1._1D[14] * m2._1D[15],
						  m1._1D[3] * m2._1D[12] + m1._1D[7] * m2._1D[13] + m1._1D[11] * m2._1D[14] + m1._1D[15] * m2._1D[15]);
	}

	CMatrix4x4 operator/(const CMatrix4x4& m, const float& s)
	{
		assert(s != 0.0f);

		return CMatrix4x4(m.column[0] / s,
						  m.column[1] / s,
						  m.column[2] / s,
						  m.column[3] / s);
	}

	CMatrix4x4 operator/(const float& s, const CMatrix4x4& m)
	{
		assert(m != CMatrix4x4(0.0f));

		return CMatrix4x4(s / m.column[0],
						  s / m.column[1],
						  s / m.column[2],
						  s / m.column[3]);
	}

	CMatrix4x4 operator/(const CMatrix4x4& m1, const CMatrix4x4& m2)
	{
		CMatrix4x4 result(m1);
		return result /= m2;
	}

	bool operator==(const CMatrix4x4& leftMatrix, const CMatrix4x4& rightMatrix)
	{
		return (leftMatrix.column[0] == rightMatrix.column[0]) &&
			   (leftMatrix.column[1] == rightMatrix.column[1]) &&
			   (leftMatrix.column[2] == rightMatrix.column[2]) &&
			   (leftMatrix.column[3] == rightMatrix.column[3]);
	}

	bool operator!=(const CMatrix4x4& leftMatrix, const CMatrix4x4& rightMatrix)
	{
		return (leftMatrix.column[0] != rightMatrix.column[0]) ||
			   (leftMatrix.column[1] != rightMatrix.column[1]) ||
			   (leftMatrix.column[2] != rightMatrix.column[2]) ||
			   (leftMatrix.column[3] != rightMatrix.column[3]);
	}

	std::ostream& operator<<(std::ostream& stream, const CMatrix4x4& m)
	{
		return stream << "Matrix4x4:" << std::endl
					  << "|" << m._1D[0] << " " << m._1D[4] << " " << m._1D[8]  << " " << m._1D[12] << "|" << std::endl
					  << "|" << m._1D[1] << " " << m._1D[5] << " " << m._1D[9]  << " " << m._1D[13] << "|" << std::endl
					  << "|" << m._1D[2] << " " << m._1D[6] << " " << m._1D[10] << " " << m._1D[14] << "|" << std::endl
					  << "|" << m._1D[3] << " " << m._1D[7] << " " << m._1D[11] << " " << m._1D[15] << "|";
	}
	/*----------------------------------------Matrix4----------------------------------------*/
	/*------------------------------------------End------------------------------------------*/
}