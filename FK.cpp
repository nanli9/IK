#include "FK.h"
#include "transform4d.h"
#include "graphSearchAlgorithms.h"
#include "basicAlgorithms.h"
#include "containerHelper.h"
#include "listIO.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <functional>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

namespace
{

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

// Convert Euler angles (in degrees) in the given RotateOrder, to the row-major 3x3 rotation.
void euler2Rotation(const double angle[3], double R[9], RotateOrder order)
{
  Mat3d RX = getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3d RY = getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3d RZ = getElementRotationMatrix(2, deg2rad(angle[2]));

  Mat3d & RMat = asMat3d(R);
  switch(order)
  {
  case XYZ:
    RMat = RZ * RY * RX;
    return;
  case YZX:
    RMat = RX * RZ * RY;
    return;
  case ZXY:
    RMat = RY * RX * RZ;
    return;
  case XZY:
    RMat = RY * RZ * RX;
    return;
  case YXZ:
    RMat = RZ * RX * RY;
    return;
  case ZYX:
    RMat = RX * RY * RZ;
    return;
  }
  assert(0);
}

} // anonymous namespace

FK::FK(const std::string & jointParentsFilename, const std::string & skeletonConfigFilename, int numIKJoints, const int* IKJointIDs)
{
  const int listOffset = 0;
  const bool sortListAfterLoad = false;
  int exitCode = ListIO::load(jointParentsFilename.c_str(), jointParents, listOffset, sortListAfterLoad);
  assert(exitCode == 0);
  numJoints = jointParents.size();
  cout << "Load # joints " << numJoints << " : " << streamRange(jointParents) << endl;

  ifstream fin(skeletonConfigFilename.c_str());
  assert(fin);
  // jointRestTransformConfig file format:
  //
  // <joint 0 translation vector> <joint 1 translation vector> ... 
  // <joint 0 rotation vector> <joint 1 rotation vector> ... 
  // <joint 0 jointOrientation vector> <joint 1 jointOrientation vector> ... 
  // (optional) <joint 0 rotateOrder ("xyz" or other orders) > <joint 0 rotateOrder> ... 
  
  // first, load translation, rotation and jointOrientation data
  jointRestTranslations.resize(numJoints);
  jointRestEulerAngles.resize(numJoints);
  jointOrientations.resize(numJoints);
  jointInvRestGlobalTransforms.resize(numJoints);

  for(int i = 0; i < numJoints; i++)
    fin >> jointRestTranslations[i][0] >> jointRestTranslations[i][1] >> jointRestTranslations[i][2];
  for(int i = 0; i < numJoints; i++)
    fin >> jointRestEulerAngles[i][0] >> jointRestEulerAngles[i][1] >> jointRestEulerAngles[i][2];
  for(int i = 0; i < numJoints; i++)
    fin >> jointOrientations[i][0] >> jointOrientations[i][1] >> jointOrientations[i][2];

  assert(fin.fail() == false);
  fin >> ws;

  jointEulerAngles = jointRestEulerAngles;

  jointRotateOrders.resize(numJoints, getDefaultRotateOrder());
  if (fin.eof() == false) // load rotateOrder
  {
    // {0: "XYZ", 1: "YZX", 2: "ZXY", 3: "XZY", 4: "YXZ", 5: "ZYX"}
    const map<string, RotateOrder> orderStrMap =
    {
      {"xyz", RotateOrder::XYZ},
      {"yzx", RotateOrder::YZX},
      {"zxy", RotateOrder::ZXY},
      {"xzy", RotateOrder::XZY},
      {"yxz", RotateOrder::YXZ},
      {"zyx", RotateOrder::ZYX}
    };
    string orderStr(3, '\0');
    for(int i = 0; i < numJoints; i++)
    {
      memset(&orderStr[0], 0, sizeof(char) * 3);
      for(int d = 0; d < 3; d++)
        fin.get(orderStr[d]);
      for(int d = 0; d < 3; d++)
        orderStr[d] = tolower(orderStr[d]);
//      cout << "joint " << i << " " << orderStr << endl;
      auto it = orderStrMap.find(orderStr);
      assert(it != orderStrMap.end());
      jointRotateOrders[i] = it->second;
      fin >> ws;
    }
    assert(fin.fail() == false);
  }
  fin.close();

  vector<bool> jointVisited(numJoints, false);
  // Use recursion to create an order to update joint transforms, 
  // so that one joint's parent always gets updated before the joint.
  std::function<void(int)> goToParent = [&](int jointID) -> void
  {
    if (jointVisited[jointID])
      return;
    if (jointParents[jointID] >= 0) // this joint is not the root
    {
      goToParent(jointParents[jointID]);
    }

    jointUpdateOrder.push_back(jointID);
    jointVisited[jointID] = true;
  };

  jointUpdateOrder.clear();
  for(int i = 0; i < numJoints; i++)
    goToParent(i);
  assert(jointUpdateOrder.size() == (size_t)numJoints);
  for(int i = 0; i < numJoints; i++)
    assert(jointParents[i] < 0 || jointUpdateOrder[jointParents[i]] < jointUpdateOrder[i]);

  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->FKInputDim = numJoints * 3;
  this->FKOutputDim = numIKJoints * 3;
  train_adolc();

  // Call computeLocalAndGlobalTransforms to compute jointRestGlobalTransforms given restTranslations/EulerAngles/JointOrientations.
  // jointInvRestGlobalTransforms here is just a place-holder.
  vector<RigidTransform4d> jointRestGlobalTransforms(numJoints);
  computeLocalAndGlobalTransforms(jointRestTranslations, jointRestEulerAngles, jointOrientations, jointRotateOrders, 
      jointParents, jointUpdateOrder,
      jointInvRestGlobalTransforms/*not used*/, jointRestGlobalTransforms);

  for (int i = 0; i < numJoints; i++)
  {
    jointInvRestGlobalTransforms[i] = inv(jointRestGlobalTransforms[i]);
  }

  buildJointChildren();

  jointLocalTransforms.resize(numJoints);
  jointGlobalTransforms.resize(numJoints);
  jointSkinTransforms.resize(numJoints);
  computeJointTransforms();
  
}

void FK::buildJointChildren()
{
  jointChildren.assign(numJoints, {});
  for(int jointID = 0; jointID < numJoints; jointID++)
  {
    int parentID = jointParents[jointID];
    if (parentID < 0)
      continue;
    jointChildren[parentID].push_back(jointID);
  }
}

vector<int> FK::getJointDescendents(int jointID) const
{
  vector<int> ret = jointChildren[jointID];
  doBFSOnDirectedTree(convertArrayToFunction(jointChildren), ret);
  sort(ret.begin(), ret.end());
  return ret;
}

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

        switch (order)
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
        int numIKJoints, const int* IKJointIDs,int numJoints, std::vector<Vec3d> jointRestTranslations, std::vector<Vec3d>jointOrientations, std::vector<int> jointParents,
        std::vector<int> jointUpdateOrder, std::vector<RotateOrder> jointRotateOrders, const std::vector<real>& eulerAngles, std::vector<real>& globalPositions)
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
        vector<Mat3<adouble>> R_global_list;
        vector<Vec3<adouble>> T_global_list;
        R_global_list.resize(numJoints);
        T_global_list.resize(numJoints);
        for (int i = 0; i < numJoints; i++)
        {
            
            int index = jointUpdateOrder[i];
            Vec3<adouble> T_local = Vec3<adouble>(jointRestTranslations[index].data());
            RotateOrder rO = jointRotateOrders[index];
            Vec3<adouble> local_angles = Vec3<adouble>(eulerAngles[3 * index], eulerAngles[3 * index + 1], eulerAngles[3 * index + 2]);
            Vec3<adouble> local_joint_orientation_angles = Vec3<adouble>(jointOrientations[index].data());
            Mat3<adouble> R_local = Euler2Rotation(local_angles.data(), rO);
            Mat3<adouble> R_JO = Euler2Rotation(local_joint_orientation_angles.data(), rO);
            Mat3<adouble> R_final = R_JO * R_local;
            //multiplyAffineTransform4ds;
            Mat3<adouble> R_global = R_final;
            Vec3<adouble> T_global = T_local;
            int tmp = jointParents[index];
            if (tmp != -1)
                multiplyAffineTransform4ds(R_global_list[tmp], T_global_list[tmp], R_final, T_local, R_global, T_global);

            //tmp = fk.getJointParent(tmp);;
            R_global_list[index] = R_global;
            T_global_list[index] = T_global;
        }
        /* for (int i = 0; i < numIKJoints; i++)
         {
             int index = IKJointIDs[i];
             Vec3<adouble> result = R_global_list[index] * Vec3<adouble>(0, 0, 0) + T_global_list[index];

             globalPositions[3 * i + 0] = result.data()[0];
             globalPositions[3 * i + 1] = result.data()[1];
             globalPositions[3 * i + 2] = result.data()[2];

         }*/
        for (int i = 0; i < numJoints; i++)
        {
            globalPositions[12 * i + 0] = R_global_list[i].data()[0];
            globalPositions[12 * i + 1] = R_global_list[i].data()[1];
            globalPositions[12 * i + 2] = R_global_list[i].data()[2];

            globalPositions[12 * i + 3] = R_global_list[i].data()[3];
            globalPositions[12 * i + 4] = R_global_list[i].data()[4];
            globalPositions[12 * i + 5] = R_global_list[i].data()[5];

            globalPositions[12 * i + 6] = R_global_list[i].data()[6];
            globalPositions[12 * i + 7] = R_global_list[i].data()[7];
            globalPositions[12 * i + 8] = R_global_list[i].data()[8];

            globalPositions[12 * i + 9] = T_global_list[i].data()[0];
            globalPositions[12 * i + 10] = T_global_list[i].data()[1];
            globalPositions[12 * i + 11] = T_global_list[i].data()[2];
        }
    }

} // end anonymous namespaces

void FK::train_adolc()
{
    // Students should implement this.
    // Here, you should setup adol_c:
    //   Define adol_c inputs and outputs. 
    //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
    //   This will later make it possible for you to compute the gradient of this function in IK::doIK
    //   (in other words, compute the "Jacobian matrix" J).
    // See ADOLCExample.cpp .
    trace_on(adolc_tagID);
    vector<adouble> x(FKInputDim);
    for (int i = 0; i < FKInputDim; i++)
        x[i] <<= 0;

    std::vector<adouble> y(12 * numJoints);

    forwardKinematicsFunction(numIKJoints, IKJointIDs,numJoints,jointRestTranslations,jointOrientations, jointParents,
        jointUpdateOrder, jointRotateOrders,x, y);

    vector<double> output(12 * numJoints);
    for (int i = 0; i < 12 * numJoints; i++)
        y[i] >>= output[i];

    trace_off();

    
}


// This is the main function that performs forward kinematics.
// Each joint has its local transformation relative to the parent joint. 
// globalTransform of a joint is the transformation that converts a point expressed in the joint's local frame of reference, to the world coordinate frame.
// localTransform is the transformation that converts a point expressed in the joint's local frame of reference, to the coordinate frame of its parent joint. Specifically, if xLocal is the homogeneous coordinate of a point expressed in the joint's local frame, and xParent is the homogeneous coordinate of the same point expressed in the frame of the joint's parent, we have:
// xParent = localTransform * xLocal , and
// globalTransform = parentGlobalTransform * localTransform .
// Note that the globalTransform of the root joint equals its localTransform.
//
// Input: translations of each joint relative to its parent (in parent's coordinate system), 
// current Euler angles, joint orientations, joint rotation order, joint parents, joint update order.
// All arrays are assumed to have the same length (= #joints).
// Joint orientations are always given in XYZ order (Maya convention).
// Output: localTransforms and globalTransforms.
void FK::computeLocalAndGlobalTransforms(
    const vector<Vec3d> & translations, const vector<Vec3d> & eulerAngles, 
    const vector<Vec3d> & jointOrientationEulerAngles, const vector<RotateOrder> & rotateOrders,
    const std::vector<int> jointParents, const vector<int> & jointUpdateOrder,
    vector<RigidTransform4d> & localTransforms, vector<RigidTransform4d> & globalTransforms)
{
  // Students should implement this.
  // First, compute the localTransform for each joint, using eulerAngles and jointOrientationEulerAngles,
  // and the "euler2Rotation" function.
  // Then, recursively compute the globalTransforms, from the root to the leaves of the hierarchy.
  // Use the jointParents and jointUpdateOrder arrays to do so.
  // Also useful are the Mat3d and RigidTransform4d classes defined in the Vega folder.
    double* input_angle = new double[FKInputDim];
    double* output_global = new double[12 * numJoints];
    for (int i = 0; i < numJoints; i++)
    {
        input_angle[3 * i + 0] = eulerAngles[i].data()[0];
        input_angle[3 * i + 1] = eulerAngles[i].data()[1];
        input_angle[3 * i + 2] = eulerAngles[i].data()[2];
    }
    
    ::function(adolc_tagID, 12 * numJoints, FKInputDim, input_angle, output_global);
    for (int i = 0; i < jointUpdateOrder.size(); i++)
    {
        int index = jointUpdateOrder[i];
        Vec3d translate(output_global[12 * index+9], output_global[12 * index + 10], output_global[12 * index + 11]);
        double R[9];
        for (int j = 0; j < 9; j++)
        {
            R[j] = output_global[12 * index + j];
        }
        Mat3d rotation(R);
        RigidTransform4d global_matrix(rotation, translate);

        globalTransforms[index] = global_matrix;
    }

  //for (int i = 0; i < jointUpdateOrder.size(); i++)
  //{
  //    int index = jointUpdateOrder[i];
  //    RotateOrder rO = rotateOrders[index];
  //    double R[9],R_JO[9];
  //    Vec3d local_angles = eulerAngles[index];
  //    Vec3d local_joint_orientation_angles = jointOrientationEulerAngles[index];
  //    euler2Rotation(local_angles.data(), R, rO);
  //    euler2Rotation(local_joint_orientation_angles.data(), R_JO, rO);
  //    //compute the local matrix for each joint
  //    RigidTransform4d local_matrix = RigidTransform4d(Mat3d(R_JO) * Mat3d(R), translations[index]);
  //    localTransforms[index] = local_matrix;
  //    RigidTransform4d global_matrix = local_matrix;
  //    int tmp = jointParents[index];
  //    while (tmp!=-1)
  //    {
  //        global_matrix = localTransforms[tmp] * global_matrix;

  //        tmp = jointParents[tmp];
  //    }
  //    globalTransforms[index] = global_matrix;
  //}


  



}

// Compute skinning transformations for all the joints, using the formula:
// skinTransform = globalTransform * invRestTransform
void FK::computeSkinningTransforms(
    const vector<RigidTransform4d> & globalTransforms, 
    const vector<RigidTransform4d> & invRestGlobalTransforms,
    vector<RigidTransform4d> & skinTransforms)
{
  // Students should implement this.
  for(int i=0; i<skinTransforms.size(); i++)
  {
    skinTransforms[i] = globalTransforms[i] * invRestGlobalTransforms[i];
  }

}

void FK::computeJointTransforms()
{
  computeLocalAndGlobalTransforms(jointRestTranslations, jointEulerAngles, jointOrientations, jointRotateOrders, 
      jointParents, jointUpdateOrder,
      jointLocalTransforms, jointGlobalTransforms);
  computeSkinningTransforms(jointGlobalTransforms, jointInvRestGlobalTransforms, jointSkinTransforms);
}

void FK::resetToRestPose()
{
  jointEulerAngles = jointRestEulerAngles;
  computeJointTransforms();
}

