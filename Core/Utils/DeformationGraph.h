/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at
 * <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef DEFORMATIONGRAPH_H_
#define DEFORMATIONGRAPH_H_

#include <sophus/se3.hpp>
#include <vector>

#include "GraphNode.h"
#include "Jacobian.h"
#include "Stopwatch.h"

/**
 * This is basically and object-oriented type approach. Using an array based approach would be
 * faster...
 */

class CholeskyDecomp;

class DeformationGraph {
 public:
  DeformationGraph(int k, std::vector<Eigen::Vector3d>* sourceVertices);
  virtual ~DeformationGraph();

  void initialiseGraph(
      std::vector<Eigen::Vector3d>* customGraph,
      std::vector<uint64_t>* graphTimeMap);

  void appendVertices(std::vector<uint64_t>* vertexTimeMap, uint32_t originalPointEnd);

  // This clears the pose map...
  void setPosesSeq(std::vector<uint64_t>* poseTimeMap, const std::vector<Sophus::SE3d>& T_wcs);

  // Stores a weight and node pointer for a vertex
  class VertexWeightMap {
   public:
    VertexWeightMap(double weight, int node) : weight(weight), node(node), relative(false) {}

    double weight;
    int node;
    bool relative;

    /**
     * BubblesortLOL
     * @param list
     * @param graph
     */
    static void sort(std::vector<VertexWeightMap>& list, std::vector<GraphNode*>& graph) {
      bool done = false;

      int size = list.size();

      while (!done) {
        done = true;
        for (int i = 0; i < size - 1; i++) {
          if (graph.at(list[i].node)->id > graph.at(list[i + 1].node)->id) {
            done = false;
            std::swap(list[i], list[i + 1]);
          }
        }
        size--;
      }
    }
  };

  std::vector<GraphNode*>& getGraph();
  std::vector<uint64_t>& getGraphTimes();

  void addConstraint(int vertexId, Eigen::Vector3d& target);
  void addRelativeConstraint(int vertexId, int targetId);

  void clearConstraints();

  void applyGraphToVertices();
  void applyGraphToPoses(std::vector<Sophus::SE3d*> T_wc_ptrs);

  bool optimiseGraphSparse(
      float& error,
      float& meanConsErr,
      const bool fernMatch,
      const uint64_t lastDeformTime);
  void resetGraph();

  bool isInit() {
    return initialised;
  }

  // Number of neighbours
  const int k;

 private:
  bool initialised;

  // From paper
  const double wRot;
  const double wReg;
  const double wCon;

  static const int numVariables = 12;
  static const int eRotRows = 6;
  static const int eRegRows = 3;
  static const int eConRows = 3;

  // Graph itself
  std::vector<GraphNode> graphNodes;
  std::vector<GraphNode*> graph;

  // Maps vertex indices to neighbours and weights
  std::vector<std::vector<VertexWeightMap>> vertexMap;
  std::vector<Eigen::Vector3d>* sourceVertices;

  // Maps pose indices to neighbours and weights
  std::vector<std::vector<VertexWeightMap>> poseMap;

  // Stores a vertex constraint
  class Constraint {
   public:
    Constraint(int vertexId, Eigen::Vector3d& targetPosition)
        : vertexId(vertexId), targetPosition(targetPosition), relative(false), targetId(-1) {}

    Constraint(int vertexId, int targetId)
        : vertexId(vertexId),
          targetPosition(Eigen::Vector3d::Zero()),
          relative(true),
          targetId(targetId) {}

    int vertexId;
    Eigen::Vector3d targetPosition;
    bool relative;
    int targetId;
  };

  std::vector<Constraint> constraints;

  std::vector<Eigen::Vector3d>* graphCloud;
  std::vector<uint64_t> sampledGraphTimes;
  uint32_t lastPointCount;

  void connectGraphSeq();

  void weightVerticesSeq(std::vector<uint64_t>* vertexTimeMap);

  void computeVertexPosition(int vertexId, Eigen::Vector3d& position);

  void sparseJacobian(Jacobian& jacobian, const int numRows, const int numCols, const int backSet);

  Eigen::VectorXd sparseResidual(const int maxRows);

  void applyDeltaSparse(Eigen::VectorXd& delta);

  CholeskyDecomp* cholesky;

  float nonRelativeConstraintError();
};

#endif /* DEFORMATIONGRAPH_H_ */
