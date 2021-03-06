#include "Math.h"

namespace idaem
{
	/*---------------------------------------Functions---------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	float Abs(const float& s) { return abs(s); }

	CVector2 Abs(const CVector2& v)
	{
		return CVector2(abs(v.element[0]),
						abs(v.element[1]));
	}

	CVector3 Abs(const CVector3& v)
	{
		return CVector3(abs(v.element[0]),
						abs(v.element[1]),
						abs(v.element[2]));
	}

	CVector4 Abs(const CVector4& v)
	{
		return CVector4(abs(v.element[0]),
						abs(v.element[1]),
						abs(v.element[2]),
						abs(v.element[3]));
	}

	float Floor(const float& s) { return floor(s); }

	CVector2 Floor(const CVector2& v)
	{
		return CVector2(floor(v.element[0]),
						floor(v.element[1]));
	}

	CVector3 Floor(const CVector3& v)
	{
		return CVector3(floor(v.element[0]),
						floor(v.element[1]),
						floor(v.element[2]));
	}

	CVector4 Floor(const CVector4& v)
	{
		return CVector4(floor(v.element[0]),
						floor(v.element[1]),
						floor(v.element[2]),
						floor(v.element[3]));
	}

	float Trunc(const float& s) { return trunc(s); }

	CVector2 Trunc(const CVector2& v)
	{
		return CVector2(trunc(v.element[0]),
						trunc(v.element[1]));
	}

	CVector3 Trunc(const CVector3& v)
	{
		return CVector3(trunc(v.element[0]),
						trunc(v.element[1]),
						trunc(v.element[2]));
	}

	CVector4 Trunc(const CVector4& v)
	{
		return CVector4(trunc(v.element[0]),
						trunc(v.element[1]),
						trunc(v.element[2]),
						trunc(v.element[3]));
	}

	float Round(const float& s) { return round(s); }

	CVector2 Round(const CVector2& v)
	{
		return CVector2(round(v.element[0]),
						round(v.element[1]));
	}

	CVector3 Round(const CVector3& v)
	{
		return CVector3(round(v.element[0]),
						round(v.element[1]),
						round(v.element[2]));
	}

	CVector4 Round(const CVector4& v)
	{
		return CVector4(round(v.element[0]),
						round(v.element[1]),
						round(v.element[2]),
						round(v.element[3]));
	}

	float Ceil(const float& s) { return ceil(s); }

	CVector2 Ceil(const CVector2& v)
	{
		return CVector2(ceil(v.element[0]),
						ceil(v.element[1]));
	}

	CVector3 Ceil(const CVector3& v)
	{
		return CVector3(ceil(v.element[0]),
						ceil(v.element[1]),
						ceil(v.element[2]));
	}

	CVector4 Ceil(const CVector4& v)
	{
		return CVector4(ceil(v.element[0]),
						ceil(v.element[1]),
						ceil(v.element[2]),
						ceil(v.element[3]));
	}

	float Fract(const float& s) { return s - Floor(s); }

	CVector2 Fract(const CVector2& v) { return v - Floor(v); }

	CVector3 Fract(const CVector3& v) { return v - Floor(v); }

	CVector4 Fract(const CVector4& v) { return v - Floor(v); }

	float Mod(const float& s1, const float& s2) { return fmod(s1, s2); }

	CVector2 Mod(const CVector2& v, const float& s)
	{
		return CVector2(fmod(v.element[0], s),
						fmod(v.element[1], s));
	}

	CVector3 Mod(const CVector3& v, const float& s)
	{
		return CVector3(fmod(v.element[0], s),
						fmod(v.element[1], s),
						fmod(v.element[2], s));
	}

	CVector4 Mod(const CVector4& v, const float& s)
	{
		return CVector4(fmod(v.element[0], s),
						fmod(v.element[1], s),
						fmod(v.element[2], s),
						fmod(v.element[3], s));
	}

	CVector2 Mod(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(fmod(v1.element[0], v2.element[0]),
						fmod(v1.element[1], v2.element[1]));
	}

	CVector3 Mod(const CVector3& v1, const CVector3& v2)
	{
		return CVector3(fmod(v1.element[0], v2.element[0]),
						fmod(v1.element[1], v2.element[1]),
						fmod(v1.element[2], v2.element[2]));
	}

	CVector4 Mod(const CVector4& v1, const CVector4& v2)
	{
		return CVector4(fmod(v1.element[0], v2.element[0]),
						fmod(v1.element[1], v2.element[1]),
						fmod(v1.element[2], v2.element[2]),
						fmod(v1.element[3], v2.element[3]));
	}

	float Min(const float& s1, const float& s2) { return fmin(s1, s2); }

	CVector2 Min(const CVector2& v, const float& s)
	{
		return CVector2(fmin(v.element[0], s),
						fmin(v.element[1], s));
	}

	CVector3 Min(const CVector3& v, const float& s)
	{
		return CVector3(fmin(v.element[0], s),
						fmin(v.element[1], s),
						fmin(v.element[2], s));
	}

	CVector4 Min(const CVector4& v, const float& s)
	{
		return CVector4(fmin(v.element[0], s),
						fmin(v.element[1], s),
						fmin(v.element[2], s),
						fmin(v.element[3], s));
	}

	CVector2 Min(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(fmin(v1.element[0], v2.element[0]),
						fmin(v1.element[1], v2.element[1]));
	}

	CVector3 Min(const CVector3& v1, const CVector3& v2)
	{
		return CVector3(fmin(v1.element[0], v2.element[0]),
						fmin(v1.element[1], v2.element[1]),
						fmin(v1.element[2], v2.element[2]));
	}

	CVector4 Min(const CVector4& v1, const CVector4& v2)
	{
		return CVector4(fmin(v1.element[0], v2.element[0]),
						fmin(v1.element[1], v2.element[1]),
						fmin(v1.element[2], v2.element[2]),
						fmin(v1.element[3], v2.element[3]));
	}

	float Max(const float& s1, const float& s2) { return fmax(s1, s2); }

	CVector2 Max(const CVector2& v, const float& s)
	{
		return CVector2(fmax(v.element[0], s),
						fmax(v.element[1], s));
	}

	CVector3 Max(const CVector3& v, const float& s)
	{
		return CVector3(fmax(v.element[0], s),
						fmax(v.element[1], s),
						fmax(v.element[2], s));
	}

	CVector4 Max(const CVector4& v, const float& s)
	{
		return CVector4(fmax(v.element[0], s),
						fmax(v.element[1], s),
						fmax(v.element[2], s),
						fmax(v.element[3], s));
	}

	CVector2 Max(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(fmax(v1.element[0], v2.element[0]),
						fmax(v1.element[1], v2.element[1]));
	}

	CVector3 Max(const CVector3& v1, const CVector3& v2)
	{
		return CVector3(fmax(v1.element[0], v2.element[0]),
						fmax(v1.element[1], v2.element[1]),
						fmax(v1.element[2], v2.element[2]));
	}

	CVector4 Max(const CVector4& v1, const CVector4& v2)
	{
		return CVector4(fmax(v1.element[0], v2.element[0]),
						fmax(v1.element[1], v2.element[1]),
						fmax(v1.element[2], v2.element[2]),
						fmax(v1.element[3], v2.element[3]));
	}

	float Clamp(const float& s, const float& minS, const float& maxS) { return Min(Max(s, minS), maxS); }

	CVector2 Clamp(const CVector2& v, const float& minS, const float& maxS) { return Min(Max(v, minS), maxS); }

	CVector3 Clamp(const CVector3& v, const float& minS, const float& maxS) { return Min(Max(v, minS), maxS); }

	CVector4 Clamp(const CVector4& v, const float& minS, const float& maxS) { return Min(Max(v, minS), maxS); }

	CVector2 Clamp(const CVector2& v, const CVector2& minV, const CVector2& maxV) { return Min(Max(v, minV), maxV); }

	CVector3 Clamp(const CVector3& v, const CVector3& minV, const CVector3& maxV) { return Min(Max(v, minV), maxV); }

	CVector4 Clamp(const CVector4& v, const CVector4& minV, const CVector4& maxV) { return Min(Max(v, minV), maxV); }

	float Mix(const float& s1, const float& s2, const float& s3) { return s1 + s3 * (s2 - s1); }

	CVector2 Mix(const CVector2& v1, const CVector2& v2, const float& s) { return v1 + s * (v2 - v1); }

	CVector3 Mix(const CVector3& v1, const CVector3& v2, const float& s) { return v1 + s * (v2 - v1); }

	CVector4 Mix(const CVector4& v1, const CVector4& v2, const float& s) { return v1 + s * (v2 - v1); }

	CVector2 Mix(const CVector2& v1, const CVector2& v2, const CVector2& v3) { return v1 + v3 * (v2 - v1); }

	CVector3 Mix(const CVector3& v1, const CVector3& v2, const CVector3& v3) { return v1 + v3 * (v2 - v1); }

	CVector4 Mix(const CVector4& v1, const CVector4& v2, const CVector4& v3) { return v1 + v3 * (v2 - v1); }

	float Step(const float& edge, const float& s) { return Mix(1.0f, 0.0f, (s < edge)); }

	CVector2 Step(const float& edge, const CVector2& s) { return Mix(1.0f, 0.0f, LessThan(s, edge)); }

	CVector3 Step(const float& edge, const CVector3& s) { return Mix(1.0f, 0.0f, LessThan(s, edge)); }

	CVector4 Step(const float& edge, const CVector4& s) { return Mix(1.0f, 0.0f, LessThan(s, edge)); }

	CVector2 Step(const CVector2& edge, const CVector2& s) { return Mix(1.0f, 0.0f, LessThan(s, edge)); }

	CVector3 Step(const CVector3& edge, const CVector3& s) { return Mix(1.0f, 0.0f, LessThan(s, edge)); }

	CVector4 Step(const CVector4& edge, const CVector4& s) { return Mix(1.0f, 0.0f, LessThan(s, edge)); }

	float SmoothStep(const float& edge1, const float& edge2, const float& s)
	{
		const float tmp(Clamp((s - edge1) / (edge2 - edge1), 0.0f, 1.0f));

		return tmp * tmp * (3.0f - 2.0f * tmp);
	}

	CVector2 SmoothStep(const float& edge1, const float& edge2, const CVector2& v)
	{
		const CVector2 tmp(Clamp((v - edge1) / (edge2 - edge1), 0.0f, 1.0f));

		return tmp * tmp * (3.0f - 2.0f * tmp);
	}

	CVector3 SmoothStep(const float& edge1, const float& edge2, const CVector3& v)
	{
		const CVector3 tmp(Clamp((v - edge1) / (edge2 - edge1), 0.0f, 1.0f));

		return tmp * tmp * (3.0f - 2.0f * tmp);
	}

	CVector4 SmoothStep(const float& edge1, const float& edge2, const CVector4& v)
	{
		const CVector4 tmp(Clamp((v - edge1) / (edge2 - edge1), 0.0f, 1.0f));

		return tmp * tmp * (3.0f - 2.0f * tmp);
	}

	CVector2 SmoothStep(const CVector2& edge1, const CVector2& edge2, const CVector2& v)
	{
		const CVector2 tmp(Clamp((v - edge1) / (edge2 - edge1), 0.0f, 1.0f));

		return tmp * tmp * (3.0f - 2.0f * tmp);
	}

	CVector3 SmoothStep(const CVector3& edge1, const CVector3& edge2, const CVector3& v)
	{
		const CVector3 tmp(Clamp((v - edge1) / (edge2 - edge1), 0.0f, 1.0f));

		return tmp * tmp * (3.0f - 2.0f * tmp);
	}

	CVector4 SmoothStep(const CVector4& edge1, const CVector4& edge2, const CVector4& v)
	{
		const CVector4 tmp(Clamp((v - edge1) / (edge2 - edge1), 0.0f, 1.0f));

		return tmp * tmp * (3.0f - 2.0f * tmp);
	}

	float Fma(const float& s1, const float& s2, const float& s3) { return s1 * s2 + s3; }

	float Pow(const float& s1, const float& s2) { return pow(s1, s2); }

	CVector2 Pow(const CVector2& v, const float& s)
	{
		return CVector2(pow(v.element[0], s),
						pow(v.element[1], s));
	}

	CVector3 Pow(const CVector3& v, const float& s)
	{
		return CVector3(pow(v.element[0], s),
						pow(v.element[1], s),
						pow(v.element[2], s));
	}

	CVector4 Pow(const CVector4& v, const float& s)
	{
		return CVector4(pow(v.element[0], s),
						pow(v.element[1], s),
						pow(v.element[2], s),
						pow(v.element[3], s));
	}

	CVector2 Pow(const CVector2& v1, const CVector2& v2)
	{
		return CVector2(pow(v1.element[0], v2.element[0]),
						pow(v1.element[1], v2.element[1]));
	}

	CVector3 Pow(const CVector3& v1, const CVector3& v2)
	{
		return CVector3(pow(v1.element[0], v2.element[0]),
						pow(v1.element[1], v2.element[1]),
						pow(v1.element[2], v2.element[2]));
	}

	CVector4 Pow(const CVector4& v1, const CVector4& v2)
	{
		return CVector4(pow(v1.element[0], v2.element[0]),
						pow(v1.element[1], v2.element[1]),
						pow(v1.element[2], v2.element[2]),
						pow(v1.element[3], v2.element[3]));
	}

	float Exp(const float& s) { return exp(s); }

	CVector2 Exp(const CVector2& v)
	{
		return CVector2(exp(v.element[0]),
						exp(v.element[1]));
	}

	CVector3 Exp(const CVector3& v)
	{
		return CVector3(exp(v.element[0]),
						exp(v.element[1]),
						exp(v.element[2]));
	}

	CVector4 Exp(const CVector4& v)
	{
		return CVector4(exp(v.element[0]),
						exp(v.element[1]),
						exp(v.element[2]),
						exp(v.element[3]));
	}

	float Exp2(const float& s) { return exp2(s); }

	CVector2 Exp2(const CVector2& v)
	{
		return CVector2(exp2(v.element[0]),
						exp2(v.element[1]));
	}

	CVector3 Exp2(const CVector3& v)
	{
		return CVector3(exp2(v.element[0]),
						exp2(v.element[1]),
						exp2(v.element[2]));
	}

	CVector4 Exp2(const CVector4& v)
	{
		return CVector4(exp2(v.element[0]),
						exp2(v.element[1]),
						exp2(v.element[2]),
						exp2(v.element[3]));
	}

	float Log(const float& s) { return log(s); }

	CVector2 Log(const CVector2& v)
	{
		return CVector2(log(v.element[0]),
						log(v.element[1]));
	}

	CVector3 Log(const CVector3& v)
	{
		return CVector3(log(v.element[0]),
						log(v.element[1]),
						log(v.element[2]));
	}

	CVector4 Log(const CVector4& v)
	{
		return CVector4(log(v.element[0]),
						log(v.element[1]),
						log(v.element[2]),
						log(v.element[3]));
	}

	float Log2(const float& s) { return log2(s); }

	CVector2 Log2(const CVector2& v)
	{
		return CVector2(log2(v.element[0]),
						log2(v.element[1]));
	}

	CVector3 Log2(const CVector3& v)
	{
		return CVector3(log2(v.element[0]),
						log2(v.element[1]),
						log2(v.element[2]));
	}

	CVector4 Log2(const CVector4& v)
	{
		return CVector4(log2(v.element[0]),
						log2(v.element[1]),
						log2(v.element[2]),
						log2(v.element[3]));
	}

	float Sqrt(const float& s) { return sqrt(s); }

	CVector2 Sqrt(const CVector2& v)
	{
		return CVector2(sqrt(v.element[0]),
						sqrt(v.element[1]));
	}

	CVector3 Sqrt(const CVector3& v)
	{
		return CVector3(sqrt(v.element[0]),
						sqrt(v.element[1]),
						sqrt(v.element[2]));
	}

	CVector4 Sqrt(const CVector4& v)
	{
		return CVector4(sqrt(v.element[0]),
						sqrt(v.element[1]),
						sqrt(v.element[2]),
						sqrt(v.element[3]));
	}

	float InvSqrt(const float& s) { return 1.0f / Sqrt(s); }

	CVector2 InvSqrt(const CVector2& v) { return 1.0f / Sqrt(v); }

	CVector3 InvSqrt(const CVector3& v) { return 1.0f / Sqrt(v); }

	CVector4 InvSqrt(const CVector4& v) { return 1.0f / Sqrt(v); }

	float ToRadians(const float& degrees) { return (degrees * (PI_VALUE / 180.0f)); }

	CVector2 ToRadians(const CVector2& degrees)
	{
		return CVector2(ToRadians(degrees.element[0]),
						ToRadians(degrees.element[1]));
	}

	CVector3 ToRadians(const CVector3& degrees)
	{
		return CVector3(ToRadians(degrees.element[0]),
						ToRadians(degrees.element[1]),
						ToRadians(degrees.element[2]));
	}

	CVector4 ToRadians(const CVector4& degrees)
	{
		return CVector4(ToRadians(degrees.element[0]),
						ToRadians(degrees.element[1]),
						ToRadians(degrees.element[2]),
						ToRadians(degrees.element[3]));
	}

	float ToDegrees(const float& radians) { return (radians * (180.0f / PI_VALUE)); }

	CVector2 ToDegrees(const CVector2& radians)
	{
		return CVector2(ToDegrees(radians.element[0]),
						ToDegrees(radians.element[1]));
	}

	CVector3 ToDegrees(const CVector3& radians)
	{
		return CVector3(ToDegrees(radians.element[0]),
						ToDegrees(radians.element[1]),
						ToDegrees(radians.element[2]));
	}

	CVector4 ToDegrees(const CVector4& radians)
	{
		return CVector4(ToDegrees(radians.element[0]),
						ToDegrees(radians.element[1]),
						ToDegrees(radians.element[2]),
						ToDegrees(radians.element[3]));
	}

	float Sin(const float& radians) { return sin(radians); }

	CVector2 Sin(const CVector2& radians)
	{
		return CVector2(sin(radians.element[0]),
						sin(radians.element[1]));
	}

	CVector3 Sin(const CVector3& radians)
	{
		return CVector3(sin(radians.element[0]),
						sin(radians.element[1]),
						sin(radians.element[2]));
	}

	CVector4 Sin(const CVector4& radians)
	{
		return CVector4(sin(radians.element[0]),
						sin(radians.element[1]),
						sin(radians.element[2]),
						sin(radians.element[3]));
	}

	float Cos(const float& degree) { return cos(degree); }

	CVector2 Cos(const CVector2& radians)
	{
		return CVector2(cos(radians.element[0]),
						cos(radians.element[1]));
	}

	CVector3 Cos(const CVector3& radians)
	{
		return CVector3(cos(radians.element[0]),
						cos(radians.element[1]),
						cos(radians.element[2]));
	}

	CVector4 Cos(const CVector4& radians)
	{
		return CVector4(cos(radians.element[0]),
						cos(radians.element[1]),
						cos(radians.element[2]),
						cos(radians.element[3]));
	}

	float Tan(const float& degree) { return tan(degree); }

	CVector2 Tan(const CVector2& radians)
	{
		return CVector2(tan(radians.element[0]),
						tan(radians.element[1]));
	}

	CVector3 Tan(const CVector3& radians)
	{
		return CVector3(tan(radians.element[0]),
						tan(radians.element[1]),
						tan(radians.element[2]));
	}

	CVector4 Tan(const CVector4& radians)
	{
		return CVector4(tan(radians.element[0]),
						tan(radians.element[1]),
						tan(radians.element[2]),
						tan(radians.element[3]));
	}

	float Asin(const float& radians) { return asin(radians); }

	CVector2 Asin(const CVector2& radians)
	{
		return CVector2(asin(radians.element[0]),
						asin(radians.element[1]));
	}

	CVector3 Asin(const CVector3& radians)
	{
		return CVector3(asin(radians.element[0]),
						asin(radians.element[1]),
						asin(radians.element[2]));
	}

	CVector4 Asin(const CVector4& radians)
	{
		return CVector4(asin(radians.element[0]),
						asin(radians.element[1]),
						asin(radians.element[2]),
						asin(radians.element[3]));
	}

	float Acos(const float& radians) { return acos(radians); }

	CVector2 Acos(const CVector2& radians)
	{
		return CVector2(acos(radians.element[0]),
						acos(radians.element[1]));
	}

	CVector3 Acos(const CVector3& radians)
	{
		return CVector3(acos(radians.element[0]),
						acos(radians.element[1]),
						acos(radians.element[2]));
	}

	CVector4 Acos(const CVector4& radians)
	{
		return CVector4(acos(radians.element[0]),
						acos(radians.element[1]),
						acos(radians.element[2]),
						acos(radians.element[3]));
	}

	float Atan(const float& radians) { return atan(radians); }

	CVector2 Atan(const CVector2& radians)
	{
		return CVector2(atan(radians.element[0]),
						atan(radians.element[1]));
	}

	CVector3 Atan(const CVector3& radians)
	{
		return CVector3(atan(radians.element[0]),
						atan(radians.element[1]),
						atan(radians.element[2]));
	}

	CVector4 Atan(const CVector4& radians)
	{
		return CVector4(atan(radians.element[0]),
						atan(radians.element[1]),
						atan(radians.element[2]),
						atan(radians.element[3]));
	}

	float Atan(const float& y, const float& x) { return atan2(y, x); }

	CVector2 Atan(const CVector2& vY, const CVector2& vX)
	{
		return CVector2(atan2(vY.element[0], vX.element[0]),
						atan2(vY.element[1], vX.element[1]));
	}

	CVector3 Atan(const CVector3& vY, const CVector3& vX)
	{
		return CVector3(atan2(vY.element[0], vX.element[0]),
						atan2(vY.element[1], vX.element[1]),
						atan2(vY.element[2], vX.element[2]));
	}

	CVector4 Atan(const CVector4& vY, const CVector4& vX)
	{
		return CVector4(atan2(vY.element[0], vX.element[0]),
						atan2(vY.element[1], vX.element[1]),
						atan2(vY.element[2], vX.element[2]),
						atan2(vY.element[3], vX.element[3]));
	}

	float Sinh(const float& radians) { return sinh(radians); }

	CVector2 Sinh(const CVector2& radians)
	{
		return CVector2(sinh(radians.element[0]),
						sinh(radians.element[1]));
	}

	CVector3 Sinh(const CVector3& radians)
	{
		return CVector3(sinh(radians.element[0]),
						sinh(radians.element[1]),
						sinh(radians.element[2]));
	}

	CVector4 Sinh(const CVector4& radians)
	{
		return CVector4(sinh(radians.element[0]),
						sinh(radians.element[1]),
						sinh(radians.element[2]),
						sinh(radians.element[3]));
	}

	float Cosh(const float& radians) { return cosh(radians); }

	CVector2 Cosh(const CVector2& radians)
	{
		return CVector2(cosh(radians.element[0]),
						cosh(radians.element[1]));
	}

	CVector3 Cosh(const CVector3& radians)
	{
		return CVector3(cosh(radians.element[0]),
						cosh(radians.element[1]),
						cosh(radians.element[2]));
	}

	CVector4 Cosh(const CVector4& radians)
	{
		return CVector4(cosh(radians.element[0]),
						cosh(radians.element[1]),
						cosh(radians.element[2]),
						cosh(radians.element[3]));
	}

	float Tanh(const float& radians) { return tanh(radians); }

	CVector2 Tanh(const CVector2& radians)
	{
		return CVector2(tanh(radians.element[0]),
						tanh(radians.element[1]));
	}

	CVector3 Tanh(const CVector3& radians)
	{
		return CVector3(tanh(radians.element[0]),
						tanh(radians.element[1]),
						tanh(radians.element[2]));
	}

	CVector4 Tanh(const CVector4& radians)
	{
		return CVector4(tanh(radians.element[0]),
						tanh(radians.element[1]),
						tanh(radians.element[2]),
						tanh(radians.element[3]));
	}

	float Asinh(const float& radians) { return asinh(radians); }

	CVector2 Asinh(const CVector2& radians)
	{
		return CVector2(asinh(radians.element[0]),
						asinh(radians.element[1]));
	}

	CVector3 Asinh(const CVector3& radians)
	{
		return CVector3(asinh(radians.element[0]),
						asinh(radians.element[1]),
						asinh(radians.element[2]));
	}

	CVector4 Asinh(const CVector4& radians)
	{
		return CVector4(asinh(radians.element[0]),
						asinh(radians.element[1]),
						asinh(radians.element[2]),
						asinh(radians.element[3]));
	}

	float Acosh(const float& radians) { return acosh(radians); }

	CVector2 Acosh(const CVector2& radians)
	{
		return CVector2(acosh(radians.element[0]),
						acosh(radians.element[1]));
	}

	CVector3 Acosh(const CVector3& radians)
	{
		return CVector3(acosh(radians.element[0]),
						acosh(radians.element[1]),
						acosh(radians.element[2]));
	}

	CVector4 Acosh(const CVector4& radians)
	{
		return CVector4(acosh(radians.element[0]),
						acosh(radians.element[1]),
						acosh(radians.element[2]),
						acosh(radians.element[3]));
	}

	float Atanh(const float& radians) { return atanh(radians); }

	CVector2 Atanh(const CVector2& radians)
	{
		return CVector2(atanh(radians.element[0]),
						atanh(radians.element[1]));
	}

	CVector3 Atanh(const CVector3& radians)
	{
		return CVector3(atanh(radians.element[0]),
						atanh(radians.element[1]),
						atanh(radians.element[2]));
	}

	CVector4 Atanh(const CVector4& radians)
	{
		return CVector4(atanh(radians.element[0]),
						atanh(radians.element[1]),
						atanh(radians.element[2]),
						atanh(radians.element[3]));
	}

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
		return Sqrt(GetLengthSqrd(q));
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

	CQuaternion Mix(const CQuaternion& q1, const CQuaternion& q2, const float& factor)
	{
		float cosTheta = Dot(q1, q2);

		if (cosTheta > 1.0f - std::numeric_limits<float>::epsilon())
		{
			return CQuaternion(Mix(q1.element[0], q2.element[0], factor),
							   Mix(q1.element[1], q2.element[1], factor),
							   Mix(q1.element[2], q2.element[2], factor),
							   Mix(q1.element[3], q2.element[3], factor));
		}
		else
		{
			float angle = Acos(cosTheta);

			return (Sin((1.0f - factor) * angle) * q1 + Sin(factor * angle) * q2) / Sin(angle);
		}
	}

	CQuaternion Lerp(const CQuaternion& q1, const CQuaternion& q2, const float& factor)
	{
		assert(factor >= 0.0f);
		assert(factor <= 1.0f);

		return q1 * (1.0f - factor) + (q2 * factor);
	}

	CQuaternion Slerp(const CQuaternion& q1, const CQuaternion& q2, const float& factor)
	{
		CQuaternion q3(q2);

		float  cosTheta = Dot(q1, q2);

		if (cosTheta < 0.0f)
		{
			q3 = -q2;
			cosTheta = -cosTheta;
		}

		if (cosTheta > 1.0f - std::numeric_limits<float>::epsilon())
		{
			return CQuaternion(Mix(q1.element[0], q3.element[0], factor),
							   Mix(q1.element[1], q3.element[1], factor),
							   Mix(q1.element[2], q3.element[2], factor),
							   Mix(q1.element[3], q3.element[3], factor));
		}
		else
		{
			float angle = Acos(cosTheta);

			return (Sin((1.0f - factor) * angle) * q1 + Sin(factor * angle) * q3) / Sin(angle);
		}
	}

	float Roll(const CQuaternion& q)
	{
		return Atan(2.0f * (q.element[1] * q.element[2] + q.element[0] * q.element[3]),
							q.element[0] * q.element[0] +
							q.element[1] * q.element[1] -
							q.element[2] * q.element[2] -
							q.element[3] * q.element[3]);
	}

	float Pitch(const CQuaternion& q)
	{
		return Atan(2.0f * (q.element[2] * q.element[3] + q.element[0] * q.element[1]),
							q.element[0] * q.element[0] -
							q.element[1] * q.element[1] -
							q.element[2] * q.element[2] +
							q.element[3] * q.element[3]);
	}

	float Yaw(const CQuaternion& q)
	{
		return Asin(Clamp(-2.0f * (q.element[1] * q.element[3] - q.element[0] * q.element[2]), -1.0f, 1.0f));
	}

	CVector3 EulerAngles(const CQuaternion& q)
	{
		return CVector3(Pitch(q), Yaw(q), Roll(q));
	}

	CQuaternion Rotate(const CQuaternion& q, const float& angle, const CVector3& v)
	{
		CVector3 tmp(v);

		float length = GetLength(tmp);

		if (Abs(length - 1.0f) > 0.001f)
		{
			float oneOverLen = 1.0f / length;

			tmp.element[0] *= oneOverLen;
			tmp.element[1] *= oneOverLen;
			tmp.element[2] *= oneOverLen;
		}

		const float angleRad(ToRadians(angle));
		const float s = Sin(angleRad * 0.5f);

		return q * CQuaternion(Cos(angleRad * 0.5f), tmp.element[0] * s, tmp.element[1] * s, tmp.element[2] * s);
	}

	CMatrix3x3 ToMat3(const CQuaternion& q)
	{
		CMatrix3x3 result(1.0f);

		float qxx(q.element[1] * q.element[1]);
		float qyy(q.element[2] * q.element[2]);
		float qzz(q.element[3] * q.element[3]);
		float qxz(q.element[1] * q.element[3]);
		float qxy(q.element[1] * q.element[2]);
		float qyz(q.element[2] * q.element[3]);
		float qwx(q.element[0] * q.element[1]);
		float qwy(q.element[0] * q.element[2]);
		float qwz(q.element[0] * q.element[3]);

		result._1D[0] = 1.0f - 2.0f * (qyy + qzz);
		result._1D[1] = 2.0f * (qxy + qwz);
		result._1D[2] = 2.0f * (qxz - qwy);
		
		result._1D[3] = 2.0f * (qxy - qwz);
		result._1D[4] = 1.0f - 2.0f * (qxx + qzz);
		result._1D[5] = 2.0f * (qyz + qwx);
		
		result._1D[6] = 2.0f * (qxz + qwy);
		result._1D[7] = 2.0f * (qyz - qwx);
		result._1D[8] = 1.0f - 2.0f * (qxx + qyy);

		return result;
	}

	CMatrix4x4 ToMat4(const CQuaternion& q)
	{
		return CMatrix4x4(ToMat3(q));
	}

	CQuaternion ToQuat(const CMatrix3x3& m)
	{
		float fourXSquaredMinus1 = m._1D[0] - m._1D[4] - m._1D[8];
		float fourYSquaredMinus1 = m._1D[4] - m._1D[0] - m._1D[8];
		float fourZSquaredMinus1 = m._1D[8] - m._1D[0] - m._1D[4];
		float fourWSquaredMinus1 = m._1D[0] + m._1D[4] + m._1D[8];

		int biggestIndex = 0;
		float fourBiggestSquaredMinus1 = fourWSquaredMinus1;

		if (fourXSquaredMinus1 > fourBiggestSquaredMinus1)
		{
			fourBiggestSquaredMinus1 = fourXSquaredMinus1;
			biggestIndex = 1;
		}

		if (fourYSquaredMinus1 > fourBiggestSquaredMinus1)
		{
			fourBiggestSquaredMinus1 = fourYSquaredMinus1;
			biggestIndex = 2;
		}

		if (fourZSquaredMinus1 > fourBiggestSquaredMinus1)
		{
			fourBiggestSquaredMinus1 = fourZSquaredMinus1;
			biggestIndex = 3;
		}

		float biggestVal = Sqrt(fourBiggestSquaredMinus1 + 1.0f) * 0.5f;
		float mult = 0.25f / biggestVal;

		CQuaternion result;

		switch (biggestIndex)
		{
		case 0:
			result.element[0] = biggestVal;
			result.element[1] = (m._1D[5] - m._1D[7]) * mult;
			result.element[2] = (m._1D[6] - m._1D[2]) * mult;
			result.element[3] = (m._1D[1] - m._1D[3]) * mult;
			break;
		case 1:
			result.element[0] = (m._1D[5] - m._1D[7]) * mult;
			result.element[1] = biggestVal;
			result.element[2] = (m._1D[1] + m._1D[3]) * mult;
			result.element[3] = (m._1D[6] + m._1D[2]) * mult;
			break;
		case 2:
			result.element[0] = (m._1D[6] - m._1D[2]) * mult;
			result.element[1] = (m._1D[1] + m._1D[3]) * mult;
			result.element[2] = biggestVal;
			result.element[3] = (m._1D[5] + m._1D[7]) * mult;
			break;
		case 3:
			result.element[0] = (m._1D[1] - m._1D[3]) * mult;
			result.element[1] = (m._1D[6] + m._1D[2]) * mult;
			result.element[2] = (m._1D[5] + m._1D[7]) * mult;
			result.element[3] = biggestVal;
			break;
		default:
			assert(false);
			break;
		}

		return result;
	}

	CQuaternion ToQuat(const CMatrix4x4& m)
	{
		return ToQuat(CMatrix3x3(m));
	}

	float Angle(const CQuaternion& q)
	{
		return Acos(q.element[0]) * 2.0f;
	}

	CVector3 Axis(const CQuaternion& q)
	{
		float tmp1 = 1.0f - q.element[0] * q.element[0];

		if (tmp1 <= 0.0f)
			return CVector3(0, 0, 1);

		float tmp2 = 1.0f / Sqrt(tmp1);

		return CVector3(q.element[1] * tmp2, q.element[2] * tmp2, q.element[3] * tmp2);
	}

	CQuaternion AngleAxis(const float& angle, const CVector3& axis)
	{
		const float angleRad = ToRadians(angle);
		const float s = Sin(angleRad * 0.5f);

		return CQuaternion(Cos(angleRad * 0.5f), axis.element[0] * s, axis.element[1] * s, axis.element[2] * s);
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
		return Sqrt(GetLengthSqrd(v));
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
		return Sqrt(GetLengthSqrd(v));
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
		return Sqrt(GetLengthSqrd(v));
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

	CMatrix2x2 Rotate(const CMatrix2x2& m, const float& angle)
	{
		const float a = ToRadians(angle);
		const float c = Cos(a);
		const float s = Sin(a);

		CMatrix2x2 result(1.0f);

		result.column[0] = m.column[0] *  c + m.column[1] * s;
		result.column[1] = m.column[0] * -s + m.column[1] * c;

		return result;
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

	CMatrix3x3 Rotate(const CMatrix3x3& m, const float& angle, const CVector3& v)
	{
		const float a = ToRadians(angle);
		const float c = Cos(a);
		const float s = Sin(a);

		CVector3 axis(Normalize(v));
		CVector3 temp((1.0f - c) * axis);

		CMatrix3x3 rotate(1.0f);

		rotate._1D[0] = c + temp[0] * axis[0];
		rotate._1D[1] = 0 + temp[0] * axis[1] + s * axis[2];
		rotate._1D[2] = 0 + temp[0] * axis[2] - s * axis[1];

		rotate._1D[3] = 0 + temp[1] * axis[0] - s * axis[2];
		rotate._1D[4] = c + temp[1] * axis[1];
		rotate._1D[5] = 0 + temp[1] * axis[2] + s * axis[0];

		rotate._1D[6] = 0 + temp[2] * axis[0] + s * axis[1];
		rotate._1D[7] = 0 + temp[2] * axis[1] - s * axis[0];
		rotate._1D[8] = c + temp[2] * axis[2];

		CMatrix3x3 result(1.0f);

		result.column[0] = m.column[0] * rotate._1D[0] + m.column[1] * rotate._1D[1] + m.column[2] * rotate._1D[2];
		result.column[1] = m.column[0] * rotate._1D[3] + m.column[1] * rotate._1D[4] + m.column[2] * rotate._1D[5];
		result.column[2] = m.column[0] * rotate._1D[6] + m.column[1] * rotate._1D[7] + m.column[2] * rotate._1D[8];

		return result;
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

	CMatrix4x4 Rotate(const CMatrix4x4& m, const float& angle, const CVector3& v)
	{
		const float a = ToRadians(angle);
		const float c = Cos(a);
		const float s = Sin(a);

		CVector3 axis(Normalize(v));
		CVector3 temp((1.0f - c) * axis);

		CMatrix4x4 rotate(1.0f);

		rotate._1D[0]  = c + temp[0] * axis[0];
		rotate._1D[1]  = 0 + temp[0] * axis[1] + s * axis[2];
		rotate._1D[2]  = 0 + temp[0] * axis[2] - s * axis[1];

		rotate._1D[4]  = 0 + temp[1] * axis[0] - s * axis[2];
		rotate._1D[5]  = c + temp[1] * axis[1];
		rotate._1D[6]  = 0 + temp[1] * axis[2] + s * axis[0];

		rotate._1D[8]  = 0 + temp[2] * axis[0] + s * axis[1];
		rotate._1D[9]  = 0 + temp[2] * axis[1] - s * axis[0];
		rotate._1D[10] = c + temp[2] * axis[2];

		CMatrix4x4 result(1.0f);

		result.column[0] = m.column[0] * rotate._1D[0] + m.column[1] * rotate._1D[1] + m.column[2] * rotate._1D[2];
		result.column[1] = m.column[0] * rotate._1D[4] + m.column[1] * rotate._1D[5] + m.column[2] * rotate._1D[6];
		result.column[2] = m.column[0] * rotate._1D[8] + m.column[1] * rotate._1D[9] + m.column[2] * rotate._1D[10];
		result.column[3] = m.column[3];												  

		return result;
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

		result._1D[0]  =  1.0f / (aspect * tanHalfFov);
		result._1D[5]  =  1.0f / (tanHalfFov);
		result._1D[10] = -(near + far) / (far - near);
		result._1D[11] = -1.0f;
		result._1D[14] =  (2.0f * near * far) / (near - far);

		return result;
	}

	CMatrix4x4 LookAt(const CVector3& eye, const CVector3& center, const CVector3& up)
	{
		const CVector3 direction(Normalize(center - eye));
		const CVector3 camRight(Normalize(Cross(direction, up)));
		const CVector3 camUp(Cross(camRight, direction));

		CMatrix4x4 result;

		result._1D[0]  = camRight.x;
		result._1D[4]  = camRight.y;
		result._1D[8]  = camRight.z;

		result._1D[1]  = camUp.x;
		result._1D[5]  = camUp.y;
		result._1D[9]  = camUp.z;

		result._1D[2]  = -direction.x;
		result._1D[6]  = -direction.y;
		result._1D[10] = -direction.z;

		result._1D[12] = -Dot(camRight, eye);
		result._1D[13] = -Dot(camUp, eye);
		result._1D[14] =  Dot(direction, eye);

		return result;
	}
	/*---------------------------------------Functions---------------------------------------*/
	/*------------------------------------------End------------------------------------------*/

	/*--------------------------------------Quaternions--------------------------------------*/
	/*-----------------------------------------Start-----------------------------------------*/
	CQuaternion::CQuaternion():
		w(1.0f), x(0.0f), y(0.0f), z(0.0f)
	{ }

	CQuaternion::CQuaternion(const float& s, const CVector3& v) :
		w(s), x(v.x), y(v.y), z(v.z)
	{ }

	CQuaternion::CQuaternion(const float& w, const float& x, const float& y, const float& z):
		w(w), x(x), y(y), z(z)
	{ }

	CQuaternion::CQuaternion(const CVector3& v1, const CVector3& v2)
	{
		const CVector3 localCross(Cross(v1, v2));
		float dot = Dot(v1, v2);
		CQuaternion q(1.0f + dot, localCross.element[1], localCross.element[2], localCross.element[3]);

		(*this) = Normalize(q);
	}

	CQuaternion::CQuaternion(const CVector3& eulerAngle)
	{
		CVector3 c = Cos(ToRadians(eulerAngle * 0.5f));
		CVector3 s = Sin(ToRadians(eulerAngle * 0.5f));

		this->element[0] = c.element[0] * c.element[1] * c.element[2] + s.element[0] * s.element[1] * s.element[2];
		this->element[1] = s.element[0] * c.element[1] * c.element[2] - c.element[0] * s.element[1] * s.element[2];
		this->element[2] = c.element[0] * s.element[1] * c.element[2] + s.element[0] * c.element[1] * s.element[2];
		this->element[3] = c.element[0] * c.element[1] * s.element[2] - s.element[0] * s.element[1] * c.element[2];
	}

	CQuaternion::CQuaternion(const CMatrix3x3& m)
	{
		*this = ToQuat(m);
	}

	CQuaternion::CQuaternion(const CMatrix4x4& m)
	{
		*this = ToQuat(m);
	}

	CQuaternion::CQuaternion(const CQuaternion& q):
		w(q.w), x(q.x), y(q.y), z(q.z)
	{ }

	CQuaternion& CQuaternion::Add(const CQuaternion& q)
	{
		this->element[0] += q.element[0];
		this->element[1] += q.element[1];
		this->element[2] += q.element[2];
		this->element[3] += q.element[3];

		return (*this);
	}

	CQuaternion& CQuaternion::Multiply(const CQuaternion& q)
	{
		const CQuaternion p(*this);
		const CQuaternion q1(q);

		this->element[0] = p.element[0] * q1.element[0] - p.element[1] * q1.element[1] - p.element[2] * q1.element[2] - p.element[3] * q1.element[3];
		this->element[1] = p.element[0] * q1.element[1] + p.element[1] * q1.element[0] + p.element[2] * q1.element[3] - p.element[3] * q1.element[2];
		this->element[2] = p.element[0] * q1.element[2] + p.element[2] * q1.element[0] + p.element[3] * q1.element[1] - p.element[1] * q1.element[3];
		this->element[3] = p.element[0] * q1.element[3] + p.element[3] * q1.element[0] + p.element[1] * q1.element[2] - p.element[2] * q1.element[1];

		return (*this);
	}

	CQuaternion& CQuaternion::Multiply(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;
		this->element[2] *= s;
		this->element[3] *= s;

		return (*this);
	}

	CQuaternion& CQuaternion::Divide(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;
		this->element[2] /= s;
		this->element[3] /= s;

		return (*this);
	}

	CQuaternion& CQuaternion::operator=(const CQuaternion& q)
	{
		this->element[0] = q.element[0];
		this->element[1] = q.element[1];
		this->element[2] = q.element[2];
		this->element[3] = q.element[3];

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
		this->element[0] += q.element[0];
		this->element[1] += q.element[1];
		this->element[2] += q.element[2];
		this->element[3] += q.element[3];

		return (*this);
	}

	CQuaternion& CQuaternion::operator*=(const CQuaternion& q)
	{
		CQuaternion const p(*this);
		CQuaternion const q1(q);

		this->element[0] = p.element[0] * q1.element[0] - p.element[1] * q1.element[1] - p.element[2] * q1.element[2] - p.element[3] * q1.element[3];
		this->element[1] = p.element[0] * q1.element[1] + p.element[1] * q1.element[0] + p.element[2] * q1.element[3] - p.element[3] * q1.element[2];
		this->element[2] = p.element[0] * q1.element[2] + p.element[2] * q1.element[0] + p.element[3] * q1.element[1] - p.element[1] * q1.element[3];
		this->element[3] = p.element[0] * q1.element[3] + p.element[3] * q1.element[0] + p.element[1] * q1.element[2] - p.element[2] * q1.element[1];

		return (*this);
	}

	CQuaternion& CQuaternion::operator*=(const float& s)
	{
		this->element[0] *= s;
		this->element[1] *= s;
		this->element[2] *= s;
		this->element[3] *= s;

		return (*this);
	}

	CQuaternion& CQuaternion::operator/=(const float& s)
	{
		assert(s != 0.0f);

		this->element[0] /= s;
		this->element[1] /= s;
		this->element[2] /= s;
		this->element[3] /= s;

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

	CVector2 LessThan(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] < rightVector.element[0],
						leftVector.element[1] < rightVector.element[1]);
	}

	CVector2 LessThan(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar < vector.element[0],
						scalar < vector.element[1]);
	}

	CVector2 LessThan(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] < scalar,
						vector.element[1] < scalar);
	}

	CVector2 LessThanEqual(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] <= rightVector.element[0],
						leftVector.element[1] <= rightVector.element[1]);
	}

	CVector2 LessThanEqual(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar <= vector.element[0],
						scalar <= vector.element[1]);
	}

	CVector2 LessThanEqual(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] <= scalar,
						vector.element[1] <= scalar);
	}

	CVector2 GreaterThan(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] > rightVector.element[0],
						leftVector.element[1] > rightVector.element[1]);
	}

	CVector2 GreaterThan(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar > vector.element[0],
						scalar > vector.element[1]);
	}

	CVector2 GreaterThan(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] > scalar,
						vector.element[1] > scalar);
	}

	CVector2 GreaterThanEqual(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] >= rightVector.element[0],
						leftVector.element[1] >= rightVector.element[1]);
	}

	CVector2 GreaterThanEqual(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar >= vector.element[0],
						scalar >= vector.element[1]);
	}

	CVector2 GreaterThanEqual(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] >= scalar,
						vector.element[1] >= scalar);
	}

	bool Equal(const CVector2& leftVector, const CVector2& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]);
	}

	bool Equal(const float& scalar, const CVector2& vector)
	{
		return (scalar == vector.element[0]) &&
			   (scalar == vector.element[1]);
	}

	bool Equal(const CVector2& vector, const float& scalar)
	{
		return (vector.element[0] == scalar) &&
			   (vector.element[1] == scalar);
	}

	bool NotEqual(const CVector2& leftVector, const CVector2& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]);
	}

	bool NotEqual(const float& scalar, const CVector2& vector)
	{
		return (scalar != vector.element[0]) ||
			   (scalar != vector.element[1]);
	}

	bool NotEqual(const CVector2& vector, const float& scalar)
	{
		return (vector.element[0] != scalar) ||
			   (vector.element[1] != scalar);
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

	CVector2 operator*(const CVector2& v, const CMatrix2x2& m)
	{
		return CVector2(v.element[0] * m._1D[0] + v.element[1] * m._1D[1],
						v.element[0] * m._1D[2] + v.element[1] * m._1D[3]);
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

	CVector2 operator<(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] < rightVector.element[0],
						leftVector.element[1] < rightVector.element[1]);
	}

	CVector2 operator<(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar < vector.element[0],
						scalar < vector.element[1]);
	}

	CVector2 operator<(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] < scalar,
						vector.element[1] < scalar);
	}

	CVector2 operator<=(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] <= rightVector.element[0],
						leftVector.element[1] <= rightVector.element[1]);
	}

	CVector2 operator<=(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar <= vector.element[0],
						scalar <= vector.element[1]);
	}

	CVector2 operator<=(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] <= scalar,
						vector.element[1] <= scalar);
	}

	CVector2 operator>(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] > rightVector.element[0],
						leftVector.element[1] > rightVector.element[1]);
	}

	CVector2 operator>(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar > vector.element[0],
						scalar > vector.element[1]);
	}

	CVector2 operator>(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] > scalar,
						vector.element[1] > scalar);
	}

	CVector2 operator>=(const CVector2& leftVector, const CVector2& rightVector)
	{
		return CVector2(leftVector.element[0] >= rightVector.element[0],
						leftVector.element[1] >= rightVector.element[1]);
	}

	CVector2 operator>=(const float& scalar, const CVector2& vector)
	{
		return CVector2(scalar >= vector.element[0],
						scalar >= vector.element[1]);
	}

	CVector2 operator>=(const CVector2& vector, const float& scalar)
	{
		return CVector2(vector.element[0] >= scalar,
						vector.element[1] >= scalar);
	}

	bool operator==(const CVector2& leftVector, const CVector2& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]);
	}

	bool operator==(const float& scalar, const CVector2& vector)
	{
		return (scalar == vector.element[0]) &&
			   (scalar == vector.element[1]);
	}

	bool operator==(const CVector2& vector, const float& scalar)
	{
		return (vector.element[0] == scalar) &&
			   (vector.element[1] == scalar);
	}

	bool operator!=(const CVector2& leftVector, const CVector2& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]);
	}

	bool operator!=(const float& scalar, const CVector2& vector)
	{
		return (scalar != vector.element[0]) ||
			   (scalar != vector.element[1]);
	}

	bool operator!=(const CVector2& vector, const float& scalar)
	{
		return (vector.element[0] != scalar) ||
			   (vector.element[1] != scalar);
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

	CVector3 LessThan(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] < rightVector.element[0],
						leftVector.element[1] < rightVector.element[1],
						leftVector.element[2] < rightVector.element[2]);
	}

	CVector3 LessThan(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar < vector.element[0],
						scalar < vector.element[1],
						scalar < vector.element[2]);
	}

	CVector3 LessThan(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] < scalar,
						vector.element[1] < scalar,
						vector.element[2] < scalar);
	}

	CVector3 LessThanEqual(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] <= rightVector.element[0],
						leftVector.element[1] <= rightVector.element[1],
						leftVector.element[2] <= rightVector.element[2]);
	}

	CVector3 LessThanEqual(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar <= vector.element[0],
						scalar <= vector.element[1],
						scalar <= vector.element[2]);
	}

	CVector3 LessThanEqual(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] <= scalar,
						vector.element[1] <= scalar,
						vector.element[2] <= scalar);
	}

	CVector3 GreaterThan(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] > rightVector.element[0],
						leftVector.element[1] > rightVector.element[1],
						leftVector.element[2] > rightVector.element[2]);
	}

	CVector3 GreaterThan(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar > vector.element[0],
						scalar > vector.element[1],
						scalar > vector.element[2]);
	}

	CVector3 GreaterThan(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] > scalar,
						vector.element[1] > scalar,
						vector.element[2] > scalar);
	}

	CVector3 GreaterThanEqual(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] >= rightVector.element[0],
						leftVector.element[1] >= rightVector.element[1],
						leftVector.element[2] >= rightVector.element[2]);
	}

	CVector3 GreaterThanEqual(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar >= vector.element[0],
						scalar >= vector.element[1],
						scalar >= vector.element[2]);
	}

	CVector3 GreaterThanEqual(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] >= scalar,
						vector.element[1] >= scalar,
						vector.element[2] >= scalar);
	}

	bool Equal(const CVector3& leftVector, const CVector3& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]) &&
			   (leftVector.element[2] == rightVector.element[2]);
	}

	bool Equal(const float& scalar, const CVector3& vector)
	{
		return (scalar == vector.element[0]) &&
			   (scalar == vector.element[1]) &&
			   (scalar == vector.element[2]);
	}

	bool Equal(const CVector3& vector, const float& scalar)
	{
		return (vector.element[0] == scalar) &&
			   (vector.element[1] == scalar) &&
			   (vector.element[1] == scalar);
	}

	bool NotEqual(const CVector3& leftVector, const CVector3& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]) ||
			   (leftVector.element[2] != rightVector.element[2]);
	}

	bool NotEqual(const float& scalar, const CVector3& vector)
	{
		return (scalar != vector.element[0]) ||
			   (scalar != vector.element[1]) ||
			   (scalar != vector.element[2]);
	}

	bool NotEqual(const CVector3& vector, const float& scalar)
	{
		return (vector.element[0] != scalar) ||
			   (vector.element[1] != scalar) ||
			   (vector.element[1] != scalar);
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

	CVector3 operator*(const CVector3& v, const CMatrix3x3& m)
	{
		return CVector3(v.element[0] * m._1D[0] + v.element[1] * m._1D[3] + v.element[2] * m._1D[6],
						v.element[0] * m._1D[1] + v.element[1] * m._1D[4] + v.element[2] * m._1D[7],
						v.element[0] * m._1D[2] + v.element[1] * m._1D[5] + v.element[2] * m._1D[8]);
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

	CVector3 operator<(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] < rightVector.element[0],
						leftVector.element[1] < rightVector.element[1],
						leftVector.element[2] < rightVector.element[2]);
	}

	CVector3 operator<(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar < vector.element[0],
						scalar < vector.element[1],
						scalar < vector.element[2]);
	}

	CVector3 operator<(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] < scalar,
						vector.element[1] < scalar,
						vector.element[2] < scalar);
	}

	CVector3 operator<=(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] <= rightVector.element[0],
						leftVector.element[1] <= rightVector.element[1],
						leftVector.element[2] <= rightVector.element[2]);
	}

	CVector3 operator<=(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar <= vector.element[0],
						scalar <= vector.element[1],
						scalar <= vector.element[2]);
	}

	CVector3 operator<=(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] <= scalar,
						vector.element[1] <= scalar,
						vector.element[2] <= scalar);
	}

	CVector3 operator>(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] > rightVector.element[0],
						leftVector.element[1] > rightVector.element[1],
						leftVector.element[2] > rightVector.element[2]);
	}

	CVector3 operator>(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar > vector.element[0],
						scalar > vector.element[1],
						scalar > vector.element[2]);
	}

	CVector3 operator>(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] > scalar,
						vector.element[1] > scalar,
						vector.element[2] > scalar);
	}

	CVector3 operator>=(const CVector3& leftVector, const CVector3& rightVector)
	{
		return CVector3(leftVector.element[0] >= rightVector.element[0],
						leftVector.element[1] >= rightVector.element[1],
						leftVector.element[2] >= rightVector.element[2]);
	}

	CVector3 operator>=(const float& scalar, const CVector3& vector)
	{
		return CVector3(scalar >= vector.element[0],
						scalar >= vector.element[1],
						scalar >= vector.element[2]);
	}

	CVector3 operator>=(const CVector3& vector, const float& scalar)
	{
		return CVector3(vector.element[0] >= scalar,
						vector.element[1] >= scalar,
						vector.element[2] >= scalar);
	}

	bool operator==(const CVector3& leftVector, const CVector3& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]) &&
			   (leftVector.element[2] == rightVector.element[2]);
	}

	bool operator==(const float& scalar, const CVector3& vector)
	{
		return (scalar == vector.element[0]) &&
			   (scalar == vector.element[1]) &&
			   (scalar == vector.element[2]);
	}

	bool operator==(const CVector3& vector, const float& scalar)
	{
		return (vector.element[0] == scalar) &&
			   (vector.element[1] == scalar) &&
			   (vector.element[2] == scalar);
	}

	bool operator!=(const CVector3& leftVector, const CVector3& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]) ||
			   (leftVector.element[2] != rightVector.element[2]);
	}

	bool operator!=(const float& scalar, const CVector3& vector)
	{
		return (scalar != vector.element[0]) ||
			   (scalar != vector.element[1]) ||
			   (scalar != vector.element[2]);
	}

	bool operator!=(const CVector3& vector, const float& scalar)
	{
		return (vector.element[0] != scalar) ||
			   (vector.element[1] != scalar) ||
			   (vector.element[2] != scalar);
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

	CVector4 LessThan(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] < rightVector.element[0],
						leftVector.element[1] < rightVector.element[1],
						leftVector.element[2] < rightVector.element[2],
						leftVector.element[3] < rightVector.element[3]);
	}

	CVector4 LessThan(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar < vector.element[0],
						scalar < vector.element[1],
						scalar < vector.element[2],
						scalar < vector.element[3]);
	}

	CVector4 LessThan(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] < scalar,
						vector.element[1] < scalar,
						vector.element[2] < scalar,
						vector.element[3] < scalar);
	}

	CVector4 LessThanEqual(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] <= rightVector.element[0],
						leftVector.element[1] <= rightVector.element[1],
						leftVector.element[2] <= rightVector.element[2],
						leftVector.element[3] <= rightVector.element[3]);
	}

	CVector4 LessThanEqual(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar <= vector.element[0],
						scalar <= vector.element[1],
						scalar <= vector.element[2],
						scalar <= vector.element[3]);
	}

	CVector4 LessThanEqual(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] <= scalar,
						vector.element[1] <= scalar,
						vector.element[2] <= scalar,
						vector.element[3] <= scalar);
	}

	CVector4 GreaterThan(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] > rightVector.element[0],
						leftVector.element[1] > rightVector.element[1],
						leftVector.element[2] > rightVector.element[2],
						leftVector.element[3] > rightVector.element[3]);
	}

	CVector4 GreaterThan(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar > vector.element[0],
						scalar > vector.element[1],
						scalar > vector.element[2],
						scalar > vector.element[3]);
	}

	CVector4 GreaterThan(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] > scalar,
						vector.element[1] > scalar,
						vector.element[2] > scalar,
						vector.element[3] > scalar);
	}

	CVector4 GreaterThanEqual(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] >= rightVector.element[0],
						leftVector.element[1] >= rightVector.element[1],
						leftVector.element[2] >= rightVector.element[2],
						leftVector.element[3] >= rightVector.element[3]);
	}

	CVector4 GreaterThanEqual(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar >= vector.element[0],
						scalar >= vector.element[1],
						scalar >= vector.element[2],
						scalar >= vector.element[3]);
	}

	CVector4 GreaterThanEqual(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] >= scalar,
						vector.element[1] >= scalar,
						vector.element[2] >= scalar,
						vector.element[3] >= scalar);
	}

	bool Equal(const CVector4& leftVector, const CVector4& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]) &&
			   (leftVector.element[2] == rightVector.element[2]) &&
			   (leftVector.element[3] == rightVector.element[3]);
	}

	bool Equal(const float& scalar, const CVector4& vector)
	{
		return (scalar == vector.element[0]) &&
			   (scalar == vector.element[1]) &&
			   (scalar == vector.element[2]) &&
			   (scalar == vector.element[3]);
	}

	bool Equal(const CVector4& vector, const float& scalar)
	{
		return (vector.element[0] == scalar) &&
			   (vector.element[1] == scalar) &&
			   (vector.element[2] == scalar) &&
			   (vector.element[3] == scalar);
	}

	bool NotEqual(const CVector4& leftVector, const CVector4& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]) ||
			   (leftVector.element[2] != rightVector.element[2]) ||
			   (leftVector.element[3] != rightVector.element[3]);
	}

	bool NotEqual(const float& scalar, const CVector4& vector)
	{
		return (scalar != vector.element[0]) ||
			   (scalar != vector.element[1]) ||
			   (scalar != vector.element[2]) ||
			   (scalar != vector.element[3]);
	}

	bool NotEqual(const CVector4& vector, const float& scalar)
	{
		return (vector.element[0] != scalar) ||
			   (vector.element[1] != scalar) ||
			   (vector.element[2] != scalar) ||
			   (vector.element[3] != scalar);
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

	CVector4 operator*(const CVector4& v, const CMatrix4x4& m)
	{
		return CVector4(v.element[0] * m._1D[0] + v.element[1] * m._1D[4] + v.element[2] * m._1D[8]  + v.element[3] * m._1D[12],
						v.element[0] * m._1D[1] + v.element[1] * m._1D[5] + v.element[2] * m._1D[9]  + v.element[3] * m._1D[13],
						v.element[0] * m._1D[2] + v.element[1] * m._1D[6] + v.element[2] * m._1D[10] + v.element[3] * m._1D[14],
						v.element[0] * m._1D[3] + v.element[1] * m._1D[7] + v.element[2] * m._1D[11] + v.element[3] * m._1D[15]);
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

	CVector4 operator<(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] < rightVector.element[0],
						leftVector.element[1] < rightVector.element[1],
						leftVector.element[2] < rightVector.element[2],
						leftVector.element[3] < rightVector.element[3]);
	}

	CVector4 operator<(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar < vector.element[0],
						scalar < vector.element[1],
						scalar < vector.element[2],
						scalar < vector.element[3]);
	}

	CVector4 operator<(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] < scalar,
						vector.element[1] < scalar,
						vector.element[2] < scalar,
						vector.element[3] < scalar);
	}

	CVector4 operator<=(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] <= rightVector.element[0],
						leftVector.element[1] <= rightVector.element[1],
						leftVector.element[2] <= rightVector.element[2],
						leftVector.element[3] <= rightVector.element[3]);
	}

	CVector4 operator<=(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar <= vector.element[0],
						scalar <= vector.element[1],
						scalar <= vector.element[2],
						scalar <= vector.element[3]);
	}

	CVector4 operator<=(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] <= scalar,
						vector.element[1] <= scalar,
						vector.element[2] <= scalar,
						vector.element[3] <= scalar);
	}

	CVector4 operator>(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] > rightVector.element[0],
						leftVector.element[1] > rightVector.element[1],
						leftVector.element[2] > rightVector.element[2],
						leftVector.element[3] > rightVector.element[3]);
	}

	CVector4 operator>(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar > vector.element[0],
						scalar > vector.element[1],
						scalar > vector.element[2],
						scalar > vector.element[3]);
	}

	CVector4 operator>(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] > scalar,
						vector.element[1] > scalar,
						vector.element[2] > scalar,
						vector.element[3] > scalar);
	}

	CVector4 operator>=(const CVector4& leftVector, const CVector4& rightVector)
	{
		return CVector4(leftVector.element[0] >= rightVector.element[0],
						leftVector.element[1] >= rightVector.element[1],
						leftVector.element[2] >= rightVector.element[2],
						leftVector.element[3] >= rightVector.element[3]);
	}

	CVector4 operator>=(const float& scalar, const CVector4& vector)
	{
		return CVector4(scalar >= vector.element[0],
						scalar >= vector.element[1],
						scalar >= vector.element[2],
						scalar >= vector.element[3]);
	}

	CVector4 operator>=(const CVector4& vector, const float& scalar)
	{
		return CVector4(vector.element[0] >= scalar,
						vector.element[1] >= scalar,
						vector.element[2] >= scalar,
						vector.element[3] >= scalar);
	}

	bool operator==(const CVector4& leftVector, const CVector4& rightVector)
	{
		return (leftVector.element[0] == rightVector.element[0]) &&
			   (leftVector.element[1] == rightVector.element[1]) &&
			   (leftVector.element[2] == rightVector.element[2]) &&
			   (leftVector.element[3] == rightVector.element[3]);
	}

	bool operator==(const float& scalar, const CVector4& vector)
	{
		return (scalar == vector.element[0]) &&
			   (scalar == vector.element[1]) &&
			   (scalar == vector.element[2]) &&
			   (scalar == vector.element[3]);
	}

	bool operator==(const CVector4& vector, const float& scalar)
	{
		return (vector.element[0] == scalar) &&
			   (vector.element[1] == scalar) &&
			   (vector.element[2] == scalar) &&
			   (vector.element[3] == scalar);
	}

	bool operator!=(const CVector4& leftVector, const CVector4& rightVector)
	{
		return (leftVector.element[0] != rightVector.element[0]) ||
			   (leftVector.element[1] != rightVector.element[1]) ||
			   (leftVector.element[2] != rightVector.element[2]) ||
			   (leftVector.element[3] != rightVector.element[3]);
	}

	bool operator!=(const float& scalar, const CVector4& vector)
	{
		return (scalar != vector.element[0]) ||
			   (scalar != vector.element[1]) ||
			   (scalar != vector.element[2]) ||
			   (scalar != vector.element[3]);
	}

	bool operator!=(const CVector4& vector, const float& scalar)
	{
		return (vector.element[0] != scalar) ||
			   (vector.element[1] != scalar) ||
			   (vector.element[2] != scalar) ||
			   (vector.element[3] != scalar);
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

	CVector2 operator*(const CMatrix2x2& m, const CVector2& v)
	{
		return CVector2(m._1D[0] * v.element[0] + m._1D[2] * v.element[1],
						m._1D[1] * v.element[0] + m._1D[3] * v.element[1]);
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

	CVector3 operator*(const CMatrix3x3& m, const CVector3& v)
	{
		return CVector3(m._1D[0] * v.element[0] + m._1D[3] * v.element[1] + m._1D[6] * v.element[2],
						m._1D[1] * v.element[0] + m._1D[4] * v.element[1] + m._1D[7] * v.element[2],
						m._1D[2] * v.element[0] + m._1D[5] * v.element[1] + m._1D[8] * v.element[2]);
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

	CVector4 operator*(const CMatrix4x4& m, const CVector4& v)
	{
		return CVector4(m._1D[0] * v.element[0] + m._1D[4] * v.element[1] + m._1D[8]  * v.element[2] + m._1D[12] * v.element[3],
						m._1D[1] * v.element[0] + m._1D[5] * v.element[1] + m._1D[9]  * v.element[2] + m._1D[13] * v.element[3],
						m._1D[2] * v.element[0] + m._1D[6] * v.element[1] + m._1D[10] * v.element[2] + m._1D[14] * v.element[3],
						m._1D[3] * v.element[0] + m._1D[7] * v.element[1] + m._1D[11] * v.element[2] + m._1D[15] * v.element[3]);
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
} // namespace idaem