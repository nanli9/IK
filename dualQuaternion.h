#pragma once
#ifndef DUAL_QUATERNION_H
#define DUAL_QUATERNION_H

#include "vec3d.h"
#include "transform4d.h"
#include "Quaternion.h"

class dualQuaternion
{
public:
	dualQuaternion();
	dualQuaternion(RigidTransform4d M);
	void normalize();
	dualQuaternion DualConjugate();
	dualQuaternion QuaternionConjugate();
	Vec3d getTranslation();
	Mat3d getRoatation();

	friend inline dualQuaternion operator* (double scalar, const dualQuaternion& q1)
	{
		dualQuaternion result;
		result.q0 = scalar * q1.q0;
		result.q_epsilon = scalar * q1.q_epsilon;
		return result;
	}
	friend inline dualQuaternion operator* (const dualQuaternion& q1, const dualQuaternion& q2)
	{
		dualQuaternion result;
		result.q0 = q1.q0 * q2.q0;
		result.q_epsilon = q1.q0 * q2.q_epsilon + q1.q_epsilon * q2.q0;
		return result;
	}
	friend inline dualQuaternion operator+ (const dualQuaternion& q1, const dualQuaternion& q2)
	{
		dualQuaternion result;
		result.q0 = q1.q0 + q2.q0;
		result.q_epsilon = q1.q_epsilon + q2.q_epsilon;
		return result;
	}

	Quaternion q0, q_epsilon;
};



#endif // !DUAL_QUATERNION_H
