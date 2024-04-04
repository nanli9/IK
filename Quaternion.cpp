#include "Quaternion.h"


Quatrernion::Quatrernion()
{
	for (int i = 0; i < 4; i++)
		value[i] = 0;
}
Quatrernion::Quatrernion(double w,double x,double y,double z)
{
	value[0] = w;
	value[1] = x;
	value[2] = y;
	value[3] = z;
}

Quatrernion::Quatrernion(RigidTransform4d m)
{
	Mat3d R = m.getRotation();
	double r32 = R.data()[7];
	double r33 = R.data()[8];
	double alpha = atan2(r32, r33);
	double theta = atan2(-R.data()[6], sqrt(r32* r32 + r33 * r33));
	double beta = atan2(R.data()[3], R.data()[0]);
	Quatrernion x = Quatrernion(cos(alpha / 2), sin(alpha / 2), 0, 0);
	Quatrernion y = Quatrernion(cos(theta / 2), 0, sin(theta / 2), 0);
	Quatrernion z = Quatrernion(cos(beta / 2), 0 , 0, sin(beta / 2));

	Quatrernion result = z * y * x;
	for (int i = 0; i < 4; i++)
		value[i] = result.value[i];
}

void Quatrernion::normalize()
{
	for (int i = 0; i < 4; i++)
		value[i] /= norm();
}

Quatrernion Quatrernion::QuatrernionConjugate(Quatrernion q)
{
	Quatrernion result;

	result.value[0] = q.value[0];
	result.value[1] = -q.value[1];
	result.value[2] = -q.value[2];
	result.value[3] = -q.value[3];

	return result;
}

double Quatrernion::norm2()
{
	return value[0] * value[0] + value[1] * value[1] + value[2] * value[2] + value[3] * value[3];
}
double Quatrernion::norm()
{
	return sqrt(norm2());
}

double Quatrernion::innerProduct(Quatrernion q)
{
	double result;
	result = value[0] * q.value[0] + value[1] * q.value[1] + value[2] * q.value[2] + value[3] * q.value[3];
	return result;
}

Mat3d Quatrernion::getRoatation()
{
	Mat3d result;
	double t0, t1, t2, t3, t4;
	t0 = 2 * (value[0]* value[1] + value[2]* value[3]);
	t1 = 1 - 2 * (value[1]* value[1]+ value[2]* value[2]);
	double alpha = atan2(t0,t1);
	t2 = 2 * (value[0]* value[2]- value[3]* value[1]);
	if (t2 > 1)
		t2 = 1;
	else if (t2 < -1)
		t2 = -1;
	double theta = asin(t2);
	t3 = 2 * (value[0]* value[3] + value[1]* value[2]);
	t4 = 1 - 2 * (value[2]* value[2] + value[3]* value[3]);
	double beta = atan2(t3, t4);

	Mat3d rx,ry,rz;
	rx.set(
		1, 0, 0,
		0, cos(alpha), -sin(alpha),
		0, sin(alpha), cos(alpha));
	ry.set(
		cos(theta), 0, sin(theta),
		0, 1, 0,
		-sin(theta), 0, cos(theta));
	rz.set(
		cos(beta), -sin(beta), 0,
		sin(beta), cos(beta), 0,
		0, 0, 1);

	result = rz * ry * rx;
	return result;
}