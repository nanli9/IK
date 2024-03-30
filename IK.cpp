#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.
    for (int i = 0; i < numIKJoints; i++)
    {
        int index = IKJointIDs[i];
        int updateOrder = fk.getJointUpdateOrder(index);
        Vec3<double> translation = Vec3<double>(fk.getJointRestTranslation(index).data());
        RotateOrder rO = fk.getJointRotateOrder(index);
        Vec3<double> local_angles = Vec3<double>(eulerAngles[index]);
        Vec3<double> local_joint_orientation_angles = Vec3<double>(fk.getJointOrient(index).data());
        Mat3<double> R = Euler2Rotation(local_angles.data(), rO);
        Mat3<double> R_JO = Euler2Rotation(local_joint_orientation_angles.data(), rO);

        //handlePositions[i] = 0.0;

    }


}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;
  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .
    int numJoints = fk->getNumJoints();
    std::vector<double> eulerAngles;
    for (int i = 0; i < numJoints; i++)
    {
        eulerAngles.push_back(fk->getJointEulerAngles()[i].data()[0]);
        eulerAngles.push_back(fk->getJointEulerAngles()[i].data()[1]);
        eulerAngles.push_back(fk->getJointEulerAngles()[i].data()[2]);
    }
    trace_on(adolc_tagID);
    vector<adouble> x(FKInputDim);
    for (int i = 0; i < FKInputDim; i++)
        x[i] <<= 0;

    std::vector<adouble> y(FKOutputDim);

    forwardKinematicsFunction(numIKJoints, IKJointIDs,*fk, x, y);


    vector<double> output(FKOutputDim);
    for (int i = 0; i < FKOutputDim; i++)
        y[i] >>= output[i];

    trace_off();
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
  // You may find the following helpful:
  int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.

}

