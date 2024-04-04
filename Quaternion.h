#pragma once
#ifndef QUATERNION_H
#define QUATERNION_H

#include "vec3d.h"
#include "transform4d.h"


class Quatrernion
{
public:
	Quatrernion();
	Quatrernion(double w, double x, double y, double z);
	Quatrernion(RigidTransform4d M);
	void normalize();
	Quatrernion QuatrernionConjugate(Quatrernion q);
	double norm();
	double norm2();
	double innerProduct(Quatrernion q);
	Mat3d getRoatation();

	friend inline Quatrernion operator* (double scalar, const Quatrernion& q1)
	{
		Quatrernion result;
		for (int i = 0; i < 4; i++)
			result.value[i] = scalar * q1.value[i];
		return result;
	}
	friend inline Quatrernion operator* (const Quatrernion& a, const Quatrernion& b)
	{
		Quatrernion result;

		result.value[0] = a.value[0] * b.value[0] - a.value[1] * b.value[1] - a.value[2] * b.value[2] - a.value[3] * b.value[3];
		result.value[1] = a.value[0] * b.value[1] + a.value[1] * b.value[0] + a.value[2] * b.value[3] - a.value[3] * b.value[2];
		result.value[2] = a.value[0] * b.value[2] - a.value[1] * b.value[3] + a.value[2] * b.value[0] + a.value[3] * b.value[1];
		result.value[3] = a.value[0] * b.value[3] + a.value[1] * b.value[2] - a.value[2] * b.value[1] + a.value[3] * b.value[0];

		return result;
	}
	friend inline Quatrernion operator+ (const Quatrernion& q1, const Quatrernion& q2)
	{
		Quatrernion result;
		for (int i = 0; i < 4; i++)
			result.value[i] = q1.value[i] + q1.value[i];
		return result;
	}
	double value[4];
};



#endif // !DUAL_QUATERNION_H
