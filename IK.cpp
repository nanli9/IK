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

//namespace
//{
//
//// Converts degrees to radians.
//template<typename real>
//inline real deg2rad(real deg) { return deg * M_PI / 180.0; }
//
//template<typename real>
//Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
//{
//  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
//  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
//  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));
//
//  switch(order)
//  {
//    case RotateOrder::XYZ:
//      return RZ * RY * RX;
//    case RotateOrder::YZX:
//      return RX * RZ * RY;
//    case RotateOrder::ZXY:
//      return RY * RX * RZ;
//    case RotateOrder::XZY:
//      return RY * RZ * RX;
//    case RotateOrder::YXZ:
//      return RZ * RX * RY;
//    case RotateOrder::ZYX:
//      return RX * RY * RZ;
//  }
//  assert(0);
//}
//
//// Performs forward kinematics, using the provided "fk" class.
//// This is the function whose Jacobian matrix will be computed using adolc.
//// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
////   IKJointIDs is an array of integers of length "numIKJoints"
//// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
//// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
//template<typename real>
//void forwardKinematicsFunction(
//    int numIKJoints, const int * IKJointIDs, const FK & fk,
//    const std::vector<real> & eulerAngles, std::vector<real> & globalPositions)
//{
//  // Students should implement this.
//  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
//  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
//  // Then, implement the same algorithm into this function. To do so,
//  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
//  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
//  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
//  // so that code is only written once. We considered this; but it is actually not easily doable.
//  // If you find a good approach, feel free to document it in the README file, for extra credit.
//    vector<Mat3<adouble>> R_global_list;
//    vector<Vec3<adouble>> T_global_list;
//    R_global_list.resize(fk.getNumJoints());
//    T_global_list.resize(fk.getNumJoints());
//    for (int i = 0; i < fk.getNumJoints(); i++)
//    {
//        int index = fk.getJointUpdateOrder(i);
//        Vec3<adouble> T_local = Vec3<adouble>(fk.getJointRestTranslation(index).data());
//        RotateOrder rO = fk.getJointRotateOrder(index);
//        Vec3<adouble> local_angles = Vec3<adouble>(eulerAngles[3*index], eulerAngles[3 * index+1], eulerAngles[3 * index+2]);
//        Vec3<adouble> local_joint_orientation_angles = Vec3<adouble>(fk.getJointOrient(index).data());
//        Mat3<adouble> R_local = Euler2Rotation(local_angles.data(), rO);
//        Mat3<adouble> R_JO = Euler2Rotation(local_joint_orientation_angles.data(), rO);
//        Mat3<adouble> R_final = R_JO * R_local;
//        //multiplyAffineTransform4ds;
//        Mat3<adouble> R_global = R_final;
//        Vec3<adouble> T_global = T_local;
//        int tmp = fk.getJointParent(index);
//        if(tmp!=-1)
//            multiplyAffineTransform4ds(R_global_list[tmp], T_global_list[tmp], R_final, T_local, R_global, T_global);
//
//        //tmp = fk.getJointParent(tmp);;
//        R_global_list[index]=R_global;
//        T_global_list[index]=T_global;
//    }
//   /* for (int i = 0; i < numIKJoints; i++)
//    {
//        int index = IKJointIDs[i];
//        Vec3<adouble> result = R_global_list[index] * Vec3<adouble>(0, 0, 0) + T_global_list[index];
//
//        globalPositions[3 * i + 0] = result.data()[0];
//        globalPositions[3 * i + 1] = result.data()[1];
//        globalPositions[3 * i + 2] = result.data()[2];
//
//    }*/
//    for (int i = 0; i < fk.getNumJoints(); i++)
//    {
//        globalPositions[12 * i + 0] = R_global_list[i].data()[0];
//        globalPositions[12 * i + 1] = R_global_list[i].data()[1];
//        globalPositions[12 * i + 2] = R_global_list[i].data()[2];
//
//        globalPositions[12 * i + 3] = R_global_list[i].data()[3];
//        globalPositions[12 * i + 4] = R_global_list[i].data()[4];
//        globalPositions[12 * i + 5] = R_global_list[i].data()[5];
//
//        globalPositions[12 * i + 6] = R_global_list[i].data()[6];
//        globalPositions[12 * i + 7] = R_global_list[i].data()[7];
//        globalPositions[12 * i + 8] = R_global_list[i].data()[8];
//
//        globalPositions[12 * i + 9] = T_global_list[i].data()[0];
//        globalPositions[12 * i + 10] = T_global_list[i].data()[1];
//        globalPositions[12 * i + 11] = T_global_list[i].data()[2];
//    }
//}
//
//} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;
  //train_adolc();

}

//void IK::train_adolc()
//{
//  // Students should implement this.
//  // Here, you should setup adol_c:
//  //   Define adol_c inputs and outputs. 
//  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
//  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
//  //   (in other words, compute the "Jacobian matrix" J).
//  // See ADOLCExample.cpp .
//    int numJoints = fk->getNumJoints();
//    trace_on(adolc_tagID);
//    vector<adouble> x(FKInputDim);
//    for (int i = 0; i < FKInputDim; i++)
//        x[i] <<= 0;
//
//    std::vector<adouble> y(12 * numJoints);
//
//    forwardKinematicsFunction(numIKJoints, IKJointIDs,*fk, x, y);
//
//    vector<double> output(12 * numJoints);
//    for (int i = 0; i < 12 * numJoints; i++)
//        y[i] >>= output[i];
//
//    trace_off();
//
//}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
  /*for (int i = 0; i < FKOutputDim; i++)
  {
      for (int j = 0; j < FKInputDim; j++)
      {
          printf("%.2f  ", jacobian_matrix[i][j]);
      }
      printf("\n");
  }*/
  // Students should implement this.
  // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
  // Specifically, use ::function, and ::jacobian .
  // See ADOLCExample.cpp .
  //
  // Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
  // Note that at entry, "jointEulerAngles" contains the input Euler angles. 
  // Upon exit, jointEulerAngles should contain the new Euler angles.
    // You may find the following helpful:
    int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!
    double* input_angle = new double[FKInputDim];
    double* output_global = new double[12 * numJoints];
    double* origin_pos = new double[FKOutputDim];
    Eigen::MatrixXd J_T(FKInputDim, FKOutputDim);
    Eigen::MatrixXd J_DAGGER(FKInputDim, FKOutputDim);
    Eigen::MatrixXd J(FKOutputDim, FKInputDim);
    Eigen::MatrixXd I(FKInputDim, FKInputDim);
    Eigen::VectorXd delta_b(FKOutputDim);

    for (int i = 0; i < FKOutputDim; i++)
        origin_pos[i] = 0;
    for (int i = 0; i < numJoints; i++)
    {
        double* tmp = jointEulerAngles[i].data();
        input_angle[3 * i] = tmp[0];
        input_angle[3 * i + 1] = tmp[1];
        input_angle[3 * i + 2] = tmp[2];
    }
    double** jacobian_matrix = new double* [12 * numJoints];
    for (int i = 0; i < 12 * numJoints; i++)
        jacobian_matrix[i] = new double[FKInputDim];
    ::jacobian(fk->adolc_tagID, 12 * numJoints, FKInputDim, input_angle, jacobian_matrix);
  ::function(fk->adolc_tagID, 12 * numJoints, FKInputDim, input_angle, output_global);

  double** jacobian = new double* [FKOutputDim];
  for (int i = 0; i < FKOutputDim; i++)
      jacobian[i] = new double[FKInputDim];

  for (int k = 0; k < numIKJoints; k++)
  {
    int index = IKJointIDs[k];
    for (int j = 0; j < FKInputDim; j++)
    {
        jacobian[3*k + 0][j] = jacobian_matrix[12 * index + 9][j];
        jacobian[3*k + 1][j] = jacobian_matrix[12 * index + 10][j];
        jacobian[3*k + 2][j] = jacobian_matrix[12 * index + 11][j];
    }
    
    origin_pos[3 * k + 0] = output_global[12 * index + 9];
    origin_pos[3 * k + 1] = output_global[12 * index + 10];
    origin_pos[3 * k + 2] = output_global[12 * index + 11];

  }

 /* Eigen::Matrix3d R();
  Eigen::Vector3d T(origin_pos[9], origin_pos[10], origin_pos[11]);
  Eigen::Vector3d Result = Eigen::Vector3d(0, 0, 0) + T;*/

  for (int i = 0; i < numIKJoints; i++)
  {
      const double* tmp = targetHandlePositions[i].data();
      delta_b(3 * i) = tmp[0] - origin_pos[3 * i];
      delta_b(3 * i + 1) = tmp[1] - origin_pos[3 * i + 1];
      delta_b(3 * i + 2) = tmp[2] - origin_pos[3 * i + 2];
  }
  //J = Eigen::MatrixXd(jacobian_matrix);
  for (int i = 0; i < FKOutputDim; i++)
  {
      for (int j = 0; j < FKInputDim; j++)
          J(i, j) = jacobian[i][j];
      /*for (int j = 0; j < FKOutputDim; j++)
      {
          if (i == j)
              I(i, j) = 1;
          else
              I(i, j) = 0;
      }*/
  }
  J_T = J.transpose();
  I.setIdentity();
  double alpha = 0.01;
  //Eigen::VectorXd delta_theta = (J_T * J + alpha * I).ldlt().solve(J_T * delta_b);
  
  J_DAGGER = J_T * (J*J_T).inverse();
  Eigen::VectorXd delta_theta = J_DAGGER * delta_b;



  for (int i = 0; i < numJoints; i++)
  {
    jointEulerAngles[i].data()[0] = fk->getJointEulerAngles()[i].data()[0] + delta_theta(3*i);
    jointEulerAngles[i].data()[1] = fk->getJointEulerAngles()[i].data()[1] + delta_theta(3*i+1);
    jointEulerAngles[i].data()[2] = fk->getJointEulerAngles()[i].data()[2] + delta_theta(3*i+2);
  }

  

}

