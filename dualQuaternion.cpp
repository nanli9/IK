#include "dualQuaternion.h"


dualQuatrernion::dualQuatrernion()
{
	for (int i = 0; i < 8; i++)
		value[i] = 0;
}

dualQuatrernion::dualQuatrernion(RigidTransform4d M)
{


}

void dualQuatrernion::normalize()
{
}
dualQuatrernion dualQuatrernion::QuatrernionConjugate(dualQuatrernion q)
{
	dualQuatrernion result;
	result.value[0] = q.value[0];
	result.value[1] = -q.value[1];
	result.value[2] = -q.value[2];
	result.value[3] = -q.value[3];

	result.value[4] = q.value[4];
	result.value[5] = -q.value[5];
	result.value[6] = -q.value[6];
	result.value[7] = -q.value[7];

	return result;
}

dualQuatrernion dualQuatrernion::DualConjugate(dualQuatrernion q)
{
	dualQuatrernion result;

	result.value[0] = q.value[0];
	result.value[1] = q.value[1];
	result.value[2] = q.value[2];
	result.value[3] = q.value[3];

	result.value[4] = -q.value[4];
	result.value[5] = -q.value[5];
	result.value[6] = -q.value[6];
	result.value[7] = -q.value[7];

	return result;
}

double dualQuatrernion::norm()
{
	double result = 0;


	return result;
}

double dualQuatrernion::innerProduct()
{
	double result;
	result = value[0] * value[4] + value[1] * value[5] + value[2] * value[6] + value[3] * value[7];


	return result;
}

Vec3d dualQuatrernion::getTranslation()
{

	Vec3d result;

	return result;
}


Mat3d dualQuatrernion::getRoatation()
{
	Mat3d result;

	return result;
}