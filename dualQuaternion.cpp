#include "dualQuaternion.h"


dualQuaternion::dualQuaternion()
{
	q0 = Quaternion();
	q_epsilon = Quaternion();
}

dualQuaternion::dualQuaternion(RigidTransform4d m)
{
	q0 = Quaternion(m);
	Vec3d t = m.getTranslation();
	q_epsilon = 0.5 * (Quaternion(0,t[0], t[1], t[2])) * q0;
}

void dualQuaternion::normalize()
{
	double norm_q0_inverse = 1.0/q0.norm();
	q0 = norm_q0_inverse * q0;
	q_epsilon = norm_q0_inverse * q_epsilon - norm_q0_inverse * norm_q0_inverse * norm_q0_inverse * (q0.innerProduct(q_epsilon) * q0);

}
dualQuaternion dualQuaternion::QuaternionConjugate()
{
	dualQuaternion result;
	result.q0 = q0.QuaternionConjugate();
	result.q_epsilon = q_epsilon.QuaternionConjugate();
	return result;
}

dualQuaternion dualQuaternion::DualConjugate()
{
	dualQuaternion result;
	result.q0 = q0;
	result.q_epsilon = -1 * q_epsilon;
	return result;
}

Vec3d dualQuaternion::getTranslation()
{
	Vec3d result;
	double norm_q0_inverse = 1.0 / q0.norm();
	Quaternion q = 2 * (norm_q0_inverse * norm_q0_inverse * q_epsilon * q0.QuaternionConjugate());
	result = Vec3d(q.value[1], q.value[2], q.value[3]);
	return result;
}


Mat3d dualQuaternion::getRoatation()
{
	Mat3d result;
	result = q0.getRoatation();
	return result;
}