#pragma once
#ifndef DUAL_QUATERNION_H
#define DUAL_QUATERNION_H

#include "vec3d.h"
#include "transform4d.h"


class dualQuatrernion
{
public:
	dualQuatrernion();
	dualQuatrernion(RigidTransform4d M);
	void normalize();
	dualQuatrernion DualConjugate(dualQuatrernion q);
	dualQuatrernion QuatrernionConjugate(dualQuatrernion q);
	double norm();
	double innerProduct();
	Vec3d getTranslation();
	Mat3d getRoatation();

	friend inline dualQuatrernion operator* (double scalar, const dualQuatrernion& q1)
	{
		dualQuatrernion result;
		for (int i = 0; i < 8; i++)
			result.value[i] = scalar * q1.value[i];
		return result;
	}
	friend inline dualQuatrernion operator* (const dualQuatrernion& q1, const dualQuatrernion& q2)
	{

	}
	friend inline dualQuatrernion operator+ (const dualQuatrernion& q1, const dualQuatrernion& q2)
	{
		dualQuatrernion result;
		for (int i = 0; i < 8; i++)
			result.value[i] = q1.value[i] + q1.value[i];
		return result;
	}
	double value[8];
};



#endif // !DUAL_QUATERNION_H
