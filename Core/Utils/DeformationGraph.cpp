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

#include "DeformationGraph.h"
#include "CholeskyDecomp.h"

DeformationGraph::DeformationGraph(int k, std::vector<Eigen::Vector3d>* sourceVertices)
    : k(k),
      initialised(false),
      wRot(1),
      wReg(10),
      wCon(100),
      sourceVertices(sourceVertices),
      graphCloud(new std::vector<Eigen::Vector3d>),
      lastPointCount(0),
      cholesky(new CholeskyDecomp) {}

DeformationGraph::~DeformationGraph() {
  if (initialised) {
    graphNodes.clear();
  }

  delete graphCloud;

  delete cholesky;
}

std::vector<GraphNode*>& DeformationGraph::getGraph() {
  return graph;
}

std::vector<uint64_t>& DeformationGraph::getGraphTimes() {
  return sampledGraphTimes;
}

void DeformationGraph::initialiseGraph(
    std::vector<Eigen::Vector3d>* customGraph,
    std::vector<uint64_t>* graphTimeMap) {
  graphCloud->clear();

  sampledGraphTimes.clear();

  sampledGraphTimes.insert(sampledGraphTimes.end(), graphTimeMap->begin(), graphTimeMap->end());

  graphCloud->insert(graphCloud->end(), customGraph->begin(), customGraph->end());

  graphNodes.clear();

  graph.clear();

  graphNodes.resize(graphCloud->size());

  for (uint32_t i = 0; i < graphCloud->size(); i++) {
    graphNodes[i].id = i;

    graphNodes[i].enabled = true;

    graphNodes[i].position = graphCloud->at(i);

    graphNodes[i].translation = Eigen::Vector3d::Zero();

    graphNodes[i].rotation.setIdentity();

    graph.push_back(&graphNodes[i]);
  }

  connectGraphSeq();

  initialised = true;
}

void DeformationGraph::appendVertices(
    std::vector<uint64_t>* vertexTimeMap,
    uint32_t originalPointEnd) {
  vertexMap.resize(lastPointCount);

  weightVerticesSeq(vertexTimeMap);

  lastPointCount = originalPointEnd;
}

void DeformationGraph::applyGraphToPoses(std::vector<Sophus::SE3d*> T_wc_ptrs) {
  assert(T_wc_ptrs.size() == poseMap.size() && initialised);

  Eigen::Vector3d newPosition;
  Eigen::Matrix3d rotation;

  for (size_t i = 0; i < T_wc_ptrs.size(); i++) {
    std::vector<VertexWeightMap>& weightMap = poseMap.at(i);

    newPosition = Eigen::Vector3d::Zero();
    rotation = Eigen::Matrix3d::Zero();

    for (size_t j = 0; j < weightMap.size(); j++) {
      newPosition += weightMap.at(j).weight *
          (graph.at(weightMap.at(j).node)->rotation *
               (T_wc_ptrs.at(i)->translation() - graph.at(weightMap.at(j).node)->position) +
           graph.at(weightMap.at(j).node)->position + graph.at(weightMap.at(j).node)->translation);

      rotation += weightMap.at(j).weight * graph.at(weightMap.at(j).node)->rotation;
    }

    Eigen::Matrix3d newRotation = rotation * T_wc_ptrs.at(i)->rotationMatrix();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(newRotation, Eigen::ComputeFullU | Eigen::ComputeFullV);

    T_wc_ptrs.at(i)->translation() = newPosition;
    T_wc_ptrs.at(i)->rotationMatrix() = svd.matrixU() * svd.matrixV().transpose();
  }
}

void DeformationGraph::setPosesSeq(
    std::vector<uint64_t>* poseTimeMap,
    const std::vector<Sophus::SE3d>& T_wcs) {
  poseMap.clear();

  const uint32_t lookBack = 20;

  for (uint32_t i = 0; i < T_wcs.size(); i++) {
    uint64_t poseTime = poseTimeMap->at(i);

    uint32_t foundIndex = 0;

    int imin = 0;
    int imax = sampledGraphTimes.size() - 1;
    int imid = (imin + imax) / 2;

    while (imax >= imin) {
      imid = (imin + imax) / 2;

      if (sampledGraphTimes[imid] < poseTime) {
        imin = imid + 1;
      } else if (sampledGraphTimes[imid] > poseTime) {
        imax = imid - 1;
      } else {
        break;
      }
    }

    imin = std::min(imin, (int)sampledGraphTimes.size() - 1);

    if (abs(int64_t(sampledGraphTimes[imin]) - int64_t(poseTime)) <=
            abs(int64_t(sampledGraphTimes[imid]) - int64_t(poseTime)) &&
        abs(int64_t(sampledGraphTimes[imin]) - int64_t(poseTime)) <=
            abs(int64_t(sampledGraphTimes[imax]) - int64_t(poseTime))) {
      foundIndex = imin;
    } else if (
        abs(int64_t(sampledGraphTimes[imid]) - int64_t(poseTime)) <=
            abs(int64_t(sampledGraphTimes[imin]) - int64_t(poseTime)) &&
        abs(int64_t(sampledGraphTimes[imid]) - int64_t(poseTime)) <=
            abs(int64_t(sampledGraphTimes[imax]) - int64_t(poseTime))) {
      foundIndex = imid;
    } else {
      foundIndex = imax;
    }

    std::vector<std::pair<float, int>> nearNodes;

    if (foundIndex == graphCloud->size()) {
      foundIndex = graphCloud->size() - 1;
    }

    uint32_t distanceBack = 0;
    for (int j = (int)foundIndex; j >= 0; j--) {
      std::pair<float, int> newNode;
      newNode.first = (graphCloud->at(j) - T_wcs.at(i).translation()).norm();
      newNode.second = j;

      nearNodes.push_back(newNode);

      if (++distanceBack == lookBack) {
        break;
      }
    }

    if (distanceBack != lookBack) {
      for (uint32_t j = foundIndex + 1; j < sampledGraphTimes.size(); j++) {
        std::pair<float, int> newNode;
        newNode.first = (graphCloud->at(j) - T_wcs.at(i).translation()).norm();
        newNode.second = j;

        nearNodes.push_back(newNode);

        if (++distanceBack == lookBack) {
          break;
        }
      }
    }

    std::sort(
        nearNodes.begin(),
        nearNodes.end(),
        [](const std::pair<float, int>& left, const std::pair<float, int>& right) {
          return left.first < right.first;
        });

    Eigen::Vector3d vertexPosition = T_wcs.at(i).translation();
    double dMax = nearNodes.at(k).first;

    std::vector<VertexWeightMap> newMap;

    double weightSum = 0;

    for (uint32_t j = 0; j < (uint32_t)k; j++) {
      newMap.push_back(VertexWeightMap(
          std::pow(
              1.0 - (vertexPosition - graphNodes[nearNodes.at(j).second].position).norm() / dMax,
              2.0),
          nearNodes.at(j).second));
      weightSum += newMap.back().weight;
    }

    for (uint32_t j = 0; j < newMap.size(); j++) {
      newMap.at(j).weight /= weightSum;
    }

    VertexWeightMap::sort(newMap, graph);

    poseMap.push_back(newMap);
  }
}

void DeformationGraph::connectGraphSeq() {
  for (int i = 0; i < k / 2; i++) {
    for (int n = 0; n < k + 1; n++) {
      if (i == n) {
        continue;
      }

      graphNodes[i].neighbours.push_back(n);
    }
  }

  for (uint32_t i = k / 2; i < graphCloud->size() - (k / 2); i++) {
    for (int n = 0; n < k / 2; n++) {
      graphNodes[i].neighbours.push_back(i - (n + 1));
      graphNodes[i].neighbours.push_back(i + (n + 1));
    }
  }

  for (uint32_t i = graphCloud->size() - (k / 2); i < graphCloud->size(); i++) {
    for (uint32_t n = graphCloud->size() - (k + 1); n < graphCloud->size(); n++) {
      if (i == n) {
        continue;
      }

      graphNodes[i].neighbours.push_back(n);
    }
  }
}

void DeformationGraph::weightVerticesSeq(std::vector<uint64_t>* vertexTimeMap) {
  const uint32_t lookBack = 20;

  for (uint32_t i = lastPointCount; i < sourceVertices->size(); i++) {
    uint64_t vertexTime = vertexTimeMap->at(i);

    uint32_t foundIndex = 0;

    int imin = 0;
    int imax = sampledGraphTimes.size() - 1;
    int imid = (imin + imax) / 2;

    while (imax >= imin) {
      imid = (imin + imax) / 2;

      if (sampledGraphTimes[imid] < vertexTime) {
        imin = imid + 1;
      } else if (sampledGraphTimes[imid] > vertexTime) {
        imax = imid - 1;
      } else {
        break;
      }
    }

    imin = std::min(imin, (int)sampledGraphTimes.size() - 1);

    if (abs(int64_t(sampledGraphTimes[imin]) - int64_t(vertexTime)) <=
            abs(int64_t(sampledGraphTimes[imid]) - int64_t(vertexTime)) &&
        abs(int64_t(sampledGraphTimes[imin]) - int64_t(vertexTime)) <=
            abs(int64_t(sampledGraphTimes[imax]) - int64_t(vertexTime))) {
      foundIndex = imin;
    } else if (
        abs(int64_t(sampledGraphTimes[imid]) - int64_t(vertexTime)) <=
            abs(int64_t(sampledGraphTimes[imin]) - int64_t(vertexTime)) &&
        abs(int64_t(sampledGraphTimes[imid]) - int64_t(vertexTime)) <=
            abs(int64_t(sampledGraphTimes[imax]) - int64_t(vertexTime))) {
      foundIndex = imid;
    } else {
      foundIndex = imax;
    }

    std::vector<std::pair<float, int>> nearNodes;

    if (foundIndex == graphCloud->size()) {
      foundIndex = graphCloud->size() - 1;
    }

    uint32_t distanceBack = 0;
    for (int j = (int)foundIndex; j >= 0; j--) {
      std::pair<float, int> newNode;
      newNode.first = (graphCloud->at(j) - sourceVertices->at(i)).norm();
      newNode.second = j;

      nearNodes.push_back(newNode);

      if (++distanceBack == lookBack) {
        break;
      }
    }

    if (distanceBack != lookBack) {
      for (uint32_t j = foundIndex + 1; j < sampledGraphTimes.size(); j++) {
        std::pair<float, int> newNode;
        newNode.first = (graphCloud->at(j) - sourceVertices->at(i)).norm();
        newNode.second = j;

        nearNodes.push_back(newNode);

        if (++distanceBack == lookBack) {
          break;
        }
      }
    }

    std::sort(
        nearNodes.begin(),
        nearNodes.end(),
        [](const std::pair<float, int>& left, const std::pair<float, int>& right) {
          return left.first < right.first;
        });

    Eigen::Vector3d vertexPosition = sourceVertices->at(i);
    double dMax = nearNodes.at(k).first;

    std::vector<VertexWeightMap> newMap;

    double weightSum = 0;

    for (uint32_t j = 0; j < (uint32_t)k; j++) {
      newMap.push_back(VertexWeightMap(
          std::pow(
              1.0 - (vertexPosition - graphNodes[nearNodes.at(j).second].position).norm() / dMax,
              2.0),
          nearNodes.at(j).second));
      weightSum += newMap.back().weight;
    }

    for (uint32_t j = 0; j < newMap.size(); j++) {
      newMap.at(j).weight /= weightSum;
    }

    VertexWeightMap::sort(newMap, graph);

    vertexMap.push_back(newMap);
  }
}

void DeformationGraph::applyGraphToVertices() {
  Eigen::Vector3d position;

  for (uint32_t i = 0; i < sourceVertices->size(); i++) {
    computeVertexPosition(i, position);
    sourceVertices->at(i) = position;
  }
}

void DeformationGraph::addConstraint(int vertexId, Eigen::Vector3d& target) {
  assert(initialised);

  // Overwrites old constraint
  for (uint32_t i = 0; i < constraints.size(); i++) {
    if (constraints.at(i).vertexId == vertexId) {
      constraints.at(i) = Constraint(vertexId, target);
      return;
    }
  }

  constraints.push_back(Constraint(vertexId, target));
}

void DeformationGraph::addRelativeConstraint(int vertexId, int targetId) {
  assert(initialised);

  // Overwrites old constraint
  for (uint32_t i = 0; i < constraints.size(); i++) {
    if (constraints.at(i).vertexId == vertexId) {
      constraints.at(i) = Constraint(vertexId, targetId);
      return;
    }
  }

  constraints.push_back(Constraint(vertexId, targetId));
}

void DeformationGraph::clearConstraints() {
  constraints.clear();
}

bool DeformationGraph::optimiseGraphSparse(
    float& error,
    float& meanConsErr,
    const bool fernMatch,
    const uint64_t lastDeformTime) {
  assert(initialised);

  TICK("opt");

  meanConsErr = nonRelativeConstraintError();

  if (fernMatch && meanConsErr < 0.06) {
    TOCK("opt");
    return false;
  }

  int maxRows = (eRotRows + eRegRows * k) * graph.size() + eConRows * constraints.size();
  int numCols = 0;
  int backSet = graph.size() * numVariables;

  for (size_t i = 0; i < graph.size(); i++) {
    graph.at(i)->enabled = sampledGraphTimes.at(i) > lastDeformTime;

    if (graph.at(i)->enabled) {
      numCols += numVariables;
      backSet -= numVariables;
    }
  }

  Eigen::VectorXd residual = sparseResidual(maxRows);

  Jacobian jacobian;

  sparseJacobian(jacobian, residual.rows(), numCols, backSet);

  error = residual.squaredNorm();

  double lastError = error;
  double errorDiff = 0;

  //    std::cout << "Initial error: " << error << ", " << meanConsErr << std::endl;

  int iter = 0;

  while (iter++ < 3) {
    Eigen::VectorXd delta = cholesky->solve(jacobian, -residual, iter == 1);

    applyDeltaSparse(delta);

    residual = sparseResidual(maxRows);

    error = residual.squaredNorm();

    errorDiff = error - lastError;

    //        std::cout << "Iteration " << iter << ": " << error << std::endl;

    if (error > lastError || delta.norm() < 1e-2 || error < 1e-3 ||
        fabs(errorDiff) < 1e-5 * error || (iter == 1 && fernMatch && error > 10.0f)) {
      break;
    }

    lastError = error;

    sparseJacobian(jacobian, residual.rows(), numCols, backSet);
  }

  cholesky->freeFactor();

  TOCK("opt");

  meanConsErr = nonRelativeConstraintError();

  //    std::cout << "Final error: " << error << ", " << meanConsErr << std::endl;

  return true;
}

void DeformationGraph::sparseJacobian(
    Jacobian& jacobian,
    const int numRows,
    const int numCols,
    const int backSet) {
  std::vector<OrderedJacobianRow*> rows(numRows);

  // We know exact counts per row...
  int lastRow = 0;

  for (uint32_t j = 0; j < graph.size(); j++) {
    if (graph.at(j)->enabled) {
      int colOffset = graph.at(j)->id * numVariables;

      // No weights for rotation as rotation weight = 1
      const Eigen::Matrix3d& rotation = graph.at(j)->rotation;

      rows[lastRow] = new OrderedJacobianRow(6);
      rows[lastRow + 1] = new OrderedJacobianRow(6);
      rows[lastRow + 2] = new OrderedJacobianRow(6);
      rows[lastRow + 3] = new OrderedJacobianRow(3);
      rows[lastRow + 4] = new OrderedJacobianRow(3);
      rows[lastRow + 5] = new OrderedJacobianRow(3);

      rows[lastRow]->append(colOffset - backSet, rotation(0, 1));
      rows[lastRow]->append(colOffset + 1 - backSet, rotation(1, 1));
      rows[lastRow]->append(colOffset + 2 - backSet, rotation(2, 1));
      rows[lastRow]->append(colOffset + 3 - backSet, rotation(0, 0));
      rows[lastRow]->append(colOffset + 4 - backSet, rotation(1, 0));
      rows[lastRow]->append(colOffset + 5 - backSet, rotation(2, 0));

      rows[lastRow + 1]->append(colOffset - backSet, rotation(0, 2));
      rows[lastRow + 1]->append(colOffset + 1 - backSet, rotation(1, 2));
      rows[lastRow + 1]->append(colOffset + 2 - backSet, rotation(2, 2));
      rows[lastRow + 1]->append(colOffset + 6 - backSet, rotation(0, 0));
      rows[lastRow + 1]->append(colOffset + 7 - backSet, rotation(1, 0));
      rows[lastRow + 1]->append(colOffset + 8 - backSet, rotation(2, 0));

      rows[lastRow + 2]->append(colOffset + 3 - backSet, rotation(0, 2));
      rows[lastRow + 2]->append(colOffset + 4 - backSet, rotation(1, 2));
      rows[lastRow + 2]->append(colOffset + 5 - backSet, rotation(2, 2));
      rows[lastRow + 2]->append(colOffset + 6 - backSet, rotation(0, 1));
      rows[lastRow + 2]->append(colOffset + 7 - backSet, rotation(1, 1));
      rows[lastRow + 2]->append(colOffset + 8 - backSet, rotation(2, 1));

      rows[lastRow + 3]->append(colOffset - backSet, 2 * rotation(0, 0));
      rows[lastRow + 3]->append(colOffset + 1 - backSet, 2 * rotation(1, 0));
      rows[lastRow + 3]->append(colOffset + 2 - backSet, 2 * rotation(2, 0));

      rows[lastRow + 4]->append(colOffset + 3 - backSet, 2 * rotation(0, 1));
      rows[lastRow + 4]->append(colOffset + 4 - backSet, 2 * rotation(1, 1));
      rows[lastRow + 4]->append(colOffset + 5 - backSet, 2 * rotation(2, 1));

      rows[lastRow + 5]->append(colOffset + 6 - backSet, 2 * rotation(0, 2));
      rows[lastRow + 5]->append(colOffset + 7 - backSet, 2 * rotation(1, 2));
      rows[lastRow + 5]->append(colOffset + 8 - backSet, 2 * rotation(2, 2));

      lastRow += eRotRows;
    }
  }

  for (uint32_t j = 0; j < graph.size(); j++) {
    int colOffset = graph.at(j)->id * numVariables;

    // For each neighbour
    for (uint32_t n = 0; n < graph.at(j)->neighbours.size(); n++) {
      if (graph.at(graph.at(j)->neighbours.at(n))->enabled || graph.at(j)->enabled) {
        rows[lastRow] = new OrderedJacobianRow(5);
        rows[lastRow + 1] = new OrderedJacobianRow(5);
        rows[lastRow + 2] = new OrderedJacobianRow(5);

        Eigen::Vector3d delta =
            graph.at(graph.at(j)->neighbours.at(n))->position - graph.at(j)->position;

        int colOffsetN = graph.at(graph.at(j)->neighbours.at(n))->id * numVariables;

        assert(colOffset != colOffsetN);

        if (colOffsetN < colOffset && graph.at(graph.at(j)->neighbours.at(n))->enabled) {
          rows[lastRow]->append(colOffsetN + 9 - backSet, -1.0 * std::sqrt(wReg));
          rows[lastRow + 1]->append(colOffsetN + 10 - backSet, -1.0 * std::sqrt(wReg));
          rows[lastRow + 2]->append(colOffsetN + 11 - backSet, -1.0 * std::sqrt(wReg));
        }

        if (graph.at(j)->enabled) {
          rows[lastRow]->append(colOffset - backSet, delta(0) * std::sqrt(wReg));
          rows[lastRow]->append(colOffset + 3 - backSet, delta(1) * std::sqrt(wReg));
          rows[lastRow]->append(colOffset + 6 - backSet, delta(2) * std::sqrt(wReg));
          rows[lastRow]->append(colOffset + 9 - backSet, 1.0 * std::sqrt(wReg));

          rows[lastRow + 1]->append(colOffset + 1 - backSet, delta(0) * std::sqrt(wReg));
          rows[lastRow + 1]->append(colOffset + 4 - backSet, delta(1) * std::sqrt(wReg));
          rows[lastRow + 1]->append(colOffset + 7 - backSet, delta(2) * std::sqrt(wReg));
          rows[lastRow + 1]->append(colOffset + 10 - backSet, 1.0 * std::sqrt(wReg));

          rows[lastRow + 2]->append(colOffset + 2 - backSet, delta(0) * std::sqrt(wReg));
          rows[lastRow + 2]->append(colOffset + 5 - backSet, delta(1) * std::sqrt(wReg));
          rows[lastRow + 2]->append(colOffset + 8 - backSet, delta(2) * std::sqrt(wReg));
          rows[lastRow + 2]->append(colOffset + 11 - backSet, 1.0 * std::sqrt(wReg));
        }

        if (colOffsetN > colOffset && graph.at(graph.at(j)->neighbours.at(n))->enabled) {
          rows[lastRow]->append(colOffsetN + 9 - backSet, -1.0 * std::sqrt(wReg));
          rows[lastRow + 1]->append(colOffsetN + 10 - backSet, -1.0 * std::sqrt(wReg));
          rows[lastRow + 2]->append(colOffsetN + 11 - backSet, -1.0 * std::sqrt(wReg));
        }

        lastRow += eRegRows;
      }
    }
  }

  for (uint32_t l = 0; l < constraints.size(); l++) {
    const std::vector<VertexWeightMap>& weightMap = vertexMap.at(constraints.at(l).vertexId);

    bool nodeInfluences = false;

    for (size_t i = 0; i < weightMap.size(); i++) {
      if (graph.at(weightMap.at(i).node)->enabled) {
        nodeInfluences = true;
        break;
      }
    }

    if (constraints.at(l).relative && !nodeInfluences) {
      const std::vector<VertexWeightMap>& relWeightMap = vertexMap.at(constraints.at(l).targetId);

      for (size_t i = 0; i < relWeightMap.size(); i++) {
        if (graph.at(relWeightMap.at(i).node)->enabled) {
          nodeInfluences = true;
          break;
        }
      }
    }

    if (nodeInfluences) {
      Eigen::Vector3d sourcePosition = sourceVertices->at(constraints.at(l).vertexId);

      rows[lastRow] = new OrderedJacobianRow(4 * k * 2);
      rows[lastRow + 1] = new OrderedJacobianRow(4 * k * 2);
      rows[lastRow + 2] = new OrderedJacobianRow(4 * k * 2);

      assert(graph.at(weightMap.at(0).node)->id < graph.at(weightMap.at(1).node)->id);

      if (constraints.at(l).relative) {
        Eigen::Vector3d targetPosition = sourceVertices->at(constraints.at(l).targetId);

        std::vector<VertexWeightMap>& relWeightMap = vertexMap.at(constraints.at(l).targetId);

        for (uint32_t i = 0; i < relWeightMap.size(); i++) {
          relWeightMap.at(i).relative = true;
        }

        std::vector<VertexWeightMap> weightMapMixed;

        weightMapMixed.insert(weightMapMixed.end(), weightMap.begin(), weightMap.end());
        weightMapMixed.insert(weightMapMixed.end(), relWeightMap.begin(), relWeightMap.end());

        VertexWeightMap::sort(weightMapMixed, graph);

        std::map<int, bool> checkList;

        for (uint32_t i = 0; i < weightMapMixed.size(); i++) {
          if (graph.at(weightMapMixed.at(i).node)->enabled) {
            int colOffset = graph.at(weightMapMixed.at(i).node)->id * numVariables;

            if (weightMapMixed.at(i).relative) {
              Eigen::Vector3d delta =
                  (graph.at(weightMapMixed.at(i).node)->position - targetPosition) *
                  weightMapMixed.at(i).weight;

              // We have to sum the Jacobian entries in this case
              if (checkList[graph.at(weightMapMixed.at(i).node)->id]) {
                rows[lastRow]->addTo(colOffset - backSet, delta(0), std::sqrt(wCon));
                rows[lastRow]->addTo(colOffset + 3 - backSet, delta(1), std::sqrt(wCon));
                rows[lastRow]->addTo(colOffset + 6 - backSet, delta(2), std::sqrt(wCon));
                rows[lastRow]->addTo(
                    colOffset + 9 - backSet, -weightMapMixed.at(i).weight, std::sqrt(wCon));

                rows[lastRow + 1]->addTo(colOffset + 1 - backSet, delta(0), std::sqrt(wCon));
                rows[lastRow + 1]->addTo(colOffset + 4 - backSet, delta(1), std::sqrt(wCon));
                rows[lastRow + 1]->addTo(colOffset + 7 - backSet, delta(2), std::sqrt(wCon));
                rows[lastRow + 1]->addTo(
                    colOffset + 10 - backSet, -weightMapMixed.at(i).weight, std::sqrt(wCon));

                rows[lastRow + 2]->addTo(colOffset + 2 - backSet, delta(0), std::sqrt(wCon));
                rows[lastRow + 2]->addTo(colOffset + 5 - backSet, delta(1), std::sqrt(wCon));
                rows[lastRow + 2]->addTo(colOffset + 8 - backSet, delta(2), std::sqrt(wCon));
                rows[lastRow + 2]->addTo(
                    colOffset + 11 - backSet, -weightMapMixed.at(i).weight, std::sqrt(wCon));
              } else {
                rows[lastRow]->append(colOffset - backSet, delta(0) * std::sqrt(wCon));
                rows[lastRow]->append(colOffset + 3 - backSet, delta(1) * std::sqrt(wCon));
                rows[lastRow]->append(colOffset + 6 - backSet, delta(2) * std::sqrt(wCon));
                rows[lastRow]->append(
                    colOffset + 9 - backSet, -weightMapMixed.at(i).weight * std::sqrt(wCon));

                rows[lastRow + 1]->append(colOffset + 1 - backSet, delta(0) * std::sqrt(wCon));
                rows[lastRow + 1]->append(colOffset + 4 - backSet, delta(1) * std::sqrt(wCon));
                rows[lastRow + 1]->append(colOffset + 7 - backSet, delta(2) * std::sqrt(wCon));
                rows[lastRow + 1]->append(
                    colOffset + 10 - backSet, -weightMapMixed.at(i).weight * std::sqrt(wCon));

                rows[lastRow + 2]->append(colOffset + 2 - backSet, delta(0) * std::sqrt(wCon));
                rows[lastRow + 2]->append(colOffset + 5 - backSet, delta(1) * std::sqrt(wCon));
                rows[lastRow + 2]->append(colOffset + 8 - backSet, delta(2) * std::sqrt(wCon));
                rows[lastRow + 2]->append(
                    colOffset + 11 - backSet, -weightMapMixed.at(i).weight * std::sqrt(wCon));
              }
            } else {
              Eigen::Vector3d delta =
                  (sourcePosition - graph.at(weightMapMixed.at(i).node)->position) *
                  weightMapMixed.at(i).weight;

              // We have to sum the Jacobian entries in this case
              if (checkList[graph.at(weightMapMixed.at(i).node)->id]) {
                rows[lastRow]->addTo(colOffset - backSet, delta(0), std::sqrt(wCon));
                rows[lastRow]->addTo(colOffset + 3 - backSet, delta(1), std::sqrt(wCon));
                rows[lastRow]->addTo(colOffset + 6 - backSet, delta(2), std::sqrt(wCon));
                rows[lastRow]->addTo(
                    colOffset + 9 - backSet, weightMapMixed.at(i).weight, std::sqrt(wCon));

                rows[lastRow + 1]->addTo(colOffset + 1 - backSet, delta(0), std::sqrt(wCon));
                rows[lastRow + 1]->addTo(colOffset + 4 - backSet, delta(1), std::sqrt(wCon));
                rows[lastRow + 1]->addTo(colOffset + 7 - backSet, delta(2), std::sqrt(wCon));
                rows[lastRow + 1]->addTo(
                    colOffset + 10 - backSet, weightMapMixed.at(i).weight, std::sqrt(wCon));

                rows[lastRow + 2]->addTo(colOffset + 2 - backSet, delta(0), std::sqrt(wCon));
                rows[lastRow + 2]->addTo(colOffset + 5 - backSet, delta(1), std::sqrt(wCon));
                rows[lastRow + 2]->addTo(colOffset + 8 - backSet, delta(2), std::sqrt(wCon));
                rows[lastRow + 2]->addTo(
                    colOffset + 11 - backSet, weightMapMixed.at(i).weight, std::sqrt(wCon));
              } else {
                rows[lastRow]->append(colOffset - backSet, delta(0) * std::sqrt(wCon));
                rows[lastRow]->append(colOffset + 3 - backSet, delta(1) * std::sqrt(wCon));
                rows[lastRow]->append(colOffset + 6 - backSet, delta(2) * std::sqrt(wCon));
                rows[lastRow]->append(
                    colOffset + 9 - backSet, weightMapMixed.at(i).weight * std::sqrt(wCon));

                rows[lastRow + 1]->append(colOffset + 1 - backSet, delta(0) * std::sqrt(wCon));
                rows[lastRow + 1]->append(colOffset + 4 - backSet, delta(1) * std::sqrt(wCon));
                rows[lastRow + 1]->append(colOffset + 7 - backSet, delta(2) * std::sqrt(wCon));
                rows[lastRow + 1]->append(
                    colOffset + 10 - backSet, weightMapMixed.at(i).weight * std::sqrt(wCon));

                rows[lastRow + 2]->append(colOffset + 2 - backSet, delta(0) * std::sqrt(wCon));
                rows[lastRow + 2]->append(colOffset + 5 - backSet, delta(1) * std::sqrt(wCon));
                rows[lastRow + 2]->append(colOffset + 8 - backSet, delta(2) * std::sqrt(wCon));
                rows[lastRow + 2]->append(
                    colOffset + 11 - backSet, weightMapMixed.at(i).weight * std::sqrt(wCon));
              }
            }

            checkList[graph.at(weightMapMixed.at(i).node)->id] = true;
          }
        }
      } else {
        // Populate each column on the current Jacobian block rows
        // WARNING: Assumes weightMap is sorted by id!
        for (uint32_t i = 0; i < weightMap.size(); i++) {
          if (graph.at(weightMap.at(i).node)->enabled) {
            int colOffset = graph.at(weightMap.at(i).node)->id * numVariables;

            Eigen::Vector3d delta = (sourcePosition - graph.at(weightMap.at(i).node)->position) *
                weightMap.at(i).weight;

            rows[lastRow]->append(colOffset - backSet, delta(0) * std::sqrt(wCon));
            rows[lastRow]->append(colOffset + 3 - backSet, delta(1) * std::sqrt(wCon));
            rows[lastRow]->append(colOffset + 6 - backSet, delta(2) * std::sqrt(wCon));
            rows[lastRow]->append(
                colOffset + 9 - backSet, weightMap.at(i).weight * std::sqrt(wCon));

            rows[lastRow + 1]->append(colOffset + 1 - backSet, delta(0) * std::sqrt(wCon));
            rows[lastRow + 1]->append(colOffset + 4 - backSet, delta(1) * std::sqrt(wCon));
            rows[lastRow + 1]->append(colOffset + 7 - backSet, delta(2) * std::sqrt(wCon));
            rows[lastRow + 1]->append(
                colOffset + 10 - backSet, weightMap.at(i).weight * std::sqrt(wCon));

            rows[lastRow + 2]->append(colOffset + 2 - backSet, delta(0) * std::sqrt(wCon));
            rows[lastRow + 2]->append(colOffset + 5 - backSet, delta(1) * std::sqrt(wCon));
            rows[lastRow + 2]->append(colOffset + 8 - backSet, delta(2) * std::sqrt(wCon));
            rows[lastRow + 2]->append(
                colOffset + 11 - backSet, weightMap.at(i).weight * std::sqrt(wCon));
          }
        }
      }

      lastRow += eConRows;
    }
  }

  assert(lastRow == numRows);

  jacobian.assign(rows, numCols);
}

Eigen::VectorXd DeformationGraph::sparseResidual(const int maxRows) {
  // Now the residual
  Eigen::VectorXd residual(maxRows);

  int numRows = 0;

  for (uint32_t j = 0; j < graph.size(); j++) {
    if (graph.at(j)->enabled) {
      // No weights for rotation as rotation weight = 1
      const Eigen::Matrix3d& rotation = graph.at(j)->rotation;

      // ab + de + gh
      residual(numRows) = rotation.col(0).dot(rotation.col(1));

      // ac + df + gi
      residual(numRows + 1) = rotation.col(0).dot(rotation.col(2));

      // bc + ef + hi
      residual(numRows + 2) = rotation.col(1).dot(rotation.col(2));

      // a^2 + d^2 + g^2 - 1
      residual(numRows + 3) = (rotation.col(0).dot(rotation.col(0)) - 1.0);

      // b^2 + e^2 + h^2 - 1
      residual(numRows + 4) = (rotation.col(1).dot(rotation.col(1)) - 1.0);

      // c^2 + f^2 + i^2 - 1
      residual(numRows + 5) = (rotation.col(2).dot(rotation.col(2)) - 1.0);

      numRows += eRotRows;
    }
  }

  for (uint32_t j = 0; j < graph.size(); j++) {
    for (uint32_t n = 0; n < graph.at(j)->neighbours.size(); n++) {
      if (graph.at(graph.at(j)->neighbours.at(n))->enabled || graph.at(j)->enabled) {
        residual.segment(numRows, 3) =
            (graph.at(j)->rotation *
                 (graph.at(graph.at(j)->neighbours.at(n))->position - graph.at(j)->position) +
             graph.at(j)->position + graph.at(j)->translation -
             (graph.at(graph.at(j)->neighbours.at(n))->position +
              graph.at(graph.at(j)->neighbours.at(n))->translation)) *
            std::sqrt(wReg);
        numRows += eRegRows;
      }
    }
  }

  for (uint32_t l = 0; l < constraints.size(); l++) {
    const std::vector<VertexWeightMap>& weightMap = vertexMap.at(constraints.at(l).vertexId);

    bool nodeInfluences = false;

    for (size_t i = 0; i < weightMap.size(); i++) {
      if (graph.at(weightMap.at(i).node)->enabled) {
        nodeInfluences = true;
        break;
      }
    }

    if (constraints.at(l).relative && !nodeInfluences) {
      const std::vector<VertexWeightMap>& relWeightMap = vertexMap.at(constraints.at(l).targetId);

      for (size_t i = 0; i < relWeightMap.size(); i++) {
        if (graph.at(relWeightMap.at(i).node)->enabled) {
          nodeInfluences = true;
          break;
        }
      }
    }

    if (nodeInfluences) {
      if (constraints.at(l).relative) {
        Eigen::Vector3d srcPos, tarPos;

        computeVertexPosition(constraints.at(l).vertexId, srcPos);

        computeVertexPosition(constraints.at(l).targetId, tarPos);

        residual.segment(numRows, 3) = (srcPos - tarPos) * std::sqrt(wCon);
      } else {
        Eigen::Vector3d position;

        computeVertexPosition(constraints.at(l).vertexId, position);

        residual.segment(numRows, 3) =
            (position - constraints.at(l).targetPosition) * std::sqrt(wCon);
      }

      numRows += eConRows;
    }
  }

  residual.conservativeResize(numRows);

  return residual;
}

void DeformationGraph::resetGraph() {
  for (uint32_t j = 0; j < graph.size(); j++) {
    graph.at(j)->rotation.setIdentity();
    graph.at(j)->translation.setIdentity();
  }
}

void DeformationGraph::applyDeltaSparse(Eigen::VectorXd& delta) {
  assert(initialised);

  // Current row
  int z = 0;

  for (uint32_t j = 0; j < graph.size(); j++) {
    if (graph.at(j)->enabled) {
      const_cast<double*>(graph.at(j)->rotation.data())[0] += delta(z + 0);
      const_cast<double*>(graph.at(j)->rotation.data())[1] += delta(z + 1);
      const_cast<double*>(graph.at(j)->rotation.data())[2] += delta(z + 2);

      const_cast<double*>(graph.at(j)->rotation.data())[3] += delta(z + 3);
      const_cast<double*>(graph.at(j)->rotation.data())[4] += delta(z + 4);
      const_cast<double*>(graph.at(j)->rotation.data())[5] += delta(z + 5);

      const_cast<double*>(graph.at(j)->rotation.data())[6] += delta(z + 6);
      const_cast<double*>(graph.at(j)->rotation.data())[7] += delta(z + 7);
      const_cast<double*>(graph.at(j)->rotation.data())[8] += delta(z + 8);

      const_cast<double*>(graph.at(j)->translation.data())[0] += delta(z + 9);
      const_cast<double*>(graph.at(j)->translation.data())[1] += delta(z + 10);
      const_cast<double*>(graph.at(j)->translation.data())[2] += delta(z + 11);

      z += numVariables;
    }
  }
}

void DeformationGraph::computeVertexPosition(int vertexId, Eigen::Vector3d& position) {
  assert(initialised);

  std::vector<VertexWeightMap>& weightMap = vertexMap.at(vertexId);

  position(0) = 0;
  position(1) = 0;
  position(2) = 0;

  Eigen::Vector3d sourcePosition = sourceVertices->at(vertexId);

  for (uint32_t i = 0; i < weightMap.size(); i++) {
    position += weightMap.at(i).weight *
        (graph.at(weightMap.at(i).node)->rotation *
             (sourcePosition - graph.at(weightMap.at(i).node)->position) +
         graph.at(weightMap.at(i).node)->position + graph.at(weightMap.at(i).node)->translation);
  }
}

float DeformationGraph::nonRelativeConstraintError() {
  float result = 0;

  for (uint32_t l = 0; l < constraints.size(); l++) {
    if (!constraints.at(l).relative) {
      Eigen::Vector3d position;
      computeVertexPosition(constraints.at(l).vertexId, position);
      result += (position - constraints.at(l).targetPosition).norm();
    }
  }

  return result / constraints.size();
}
