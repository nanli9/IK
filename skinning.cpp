#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

struct DualQuaternion {
    Eigen::Quaterniond  q1;
    Eigen::Quaterniond  q2;
};


Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
  // Students should implement this
  // The following below is just a dummy implementation.
  //meshSkinningJoints
    //for (int i = 0; i < 22; i++)
    //{
    //    //jointSkinTransforms[i].print();
    //    for (int j = 0; j < 4; j++)
    //    {
    //        for (int k = 0; k < 4; k++)
    //        {
    //            if (j != k)
    //            {
    //                if (fabs(jointSkinTransforms[i][j][k]) > 0.001)
    //                    printf("error\n");
    //            }
    //        }

    //    }
    //}
  for(int i=0; i<numMeshVertices; i++)
  {
      double x = restMeshVertexPositions[3 * i + 0];
      double y = restMeshVertexPositions[3 * i + 1];
      double z = restMeshVertexPositions[3 * i + 2];
      Vec4d rest_vertex_pos = Vec4d(x, y, z, 1);
      Vec4d result_pos = Vec4d(0, 0, 0, 1);
      double sum = 0;
      for (int j = 0; j < numJointsInfluencingEachVertex; j++)
      {
          double weight = meshSkinningWeights[numJointsInfluencingEachVertex * i+j];
          sum += weight;
          RigidTransform4d skin_matrix = jointSkinTransforms[meshSkinningJoints[numJointsInfluencingEachVertex * i+j]];
          result_pos += weight * skin_matrix * rest_vertex_pos;
      }
      if (fabs(sum - 1) > 0.001)
          printf("error\n");
      /*if (fabs(result_pos[0] - x) > 0.001)
          printf("error\n");*/
      newMeshVertexPositions[3 * i + 0] = result_pos[0];
      newMeshVertexPositions[3 * i + 1] = result_pos[1];
      newMeshVertexPositions[3 * i + 2] = result_pos[2];
  }


}

