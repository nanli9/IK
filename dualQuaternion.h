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
	double innerProduct(dualQuatrernion q1);
	Vec3d getTranslation();
	Mat3d getRoatation();

	friend inline dualQuatrernion operator* (double scalar, const dualQuatrernion& q1);
	friend inline dualQuatrernion operator* (const dualQuatrernion& q1, const dualQuatrernion& q2);
	friend inline dualQuatrernion operator+ (const dualQuatrernion& q1, const dualQuatrernion& q2);


	double value[8];
};



#endif // !DUAL_QUATERNION_H
