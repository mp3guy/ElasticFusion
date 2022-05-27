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

#include "Deformation.h"

Deformation::Deformation()
    : def(4, &pointPool),
      originalPointPool(0),
      firstGraphNode(0),
      sampleProgram(loadProgramGeomFromFile("sample.vert", "sample.geom")),
      bufferSize(1024), // max nodes basically
      count(0),
      rawSampledNodes_w(new Eigen::Vector4f[bufferSize]),
      graphPosePoints(new std::vector<Eigen::Vector3d>),
      lastDeformTime(0) {
  // x, y, z and init time
  for (int i = 0; i < bufferSize; i++) {
    rawSampledNodes_w[i] = Eigen::Vector4f::Zero();
  }

  glGenTransformFeedbacks(1, &fid);
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(
      GL_ARRAY_BUFFER, bufferSize * sizeof(Eigen::Vector4f), &rawSampledNodes_w[0], GL_STREAM_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  sampleProgram->Bind();

  int loc[1] = {
      glGetVaryingLocationNV(sampleProgram->programId(), "vData"),
  };

  glTransformFeedbackVaryingsNV(sampleProgram->programId(), 1, loc, GL_INTERLEAVED_ATTRIBS);

  sampleProgram->Unbind();

  glGenQueries(1, &countQuery);
}

Deformation::~Deformation() {
  delete[] rawSampledNodes_w;
  glDeleteTransformFeedbacks(1, &fid);
  glDeleteBuffers(1, &vbo);
  glDeleteQueries(1, &countQuery);
  delete graphPosePoints;
}

std::vector<GraphNode*>& Deformation::getGraph() {
  return def.getGraph();
}

void Deformation::addConstraint(const Constraint& constraint) {
  constraints.push_back(constraint);
}

void Deformation::addConstraint(
    const Eigen::Vector4d& src,
    const Eigen::Vector4d& target,
    const uint64_t& srcTime,
    const uint64_t& targetTime,
    const bool pinConstraints) {
  // Add the new constraint
  constraints.push_back(Constraint(src.head(3), target.head(3), srcTime, targetTime, false));

  if (pinConstraints) {
    constraints.push_back(
        Constraint(target.head(3), target.head(3), targetTime, targetTime, false, true));
  }
}

bool Deformation::constrain(
    std::vector<Ferns::Frame*>& ferns,
    std::vector<float>& rawGraph,
    int time,
    const bool fernMatch,
    std::vector<std::pair<uint64_t, Sophus::SE3d>>& t_T_wc,
    const bool relaxGraph,
    std::vector<Constraint>* newRelativeCons) {
  if (def.isInit()) {
    std::vector<uint64_t> times;
    std::vector<Sophus::SE3d> T_wcs;
    std::vector<Sophus::SE3d*> T_wc_ptrs;

    // Deform the set of ferns
    for (size_t i = 0; i < ferns.size(); i++) {
      times.push_back(ferns.at(i)->srcTime);
      T_wcs.push_back(ferns.at(i)->T_wc);
      T_wc_ptrs.push_back(&ferns.at(i)->T_wc);
    }

    if (fernMatch) {
      // Also apply to the current full pose graph (this might be silly :D)
      for (size_t i = 0; i < t_T_wc.size(); i++) {
        times.push_back(t_T_wc.at(i).first);
        T_wcs.push_back(t_T_wc.at(i).second);
        T_wc_ptrs.push_back(&t_T_wc.at(i).second);
      }
    }

    def.setPosesSeq(&times, T_wcs);

    int originalPointPool = pointPool.size();

    // Target to source
    for (size_t i = 0; i < constraints.size(); i++) {
      pointPool.push_back(constraints.at(i).src);
      vertexTimes.push_back(constraints.at(i).srcTime);
      constraints.at(i).srcPointPoolId = pointPool.size() - 1;

      if (constraints.at(i).relative) {
        pointPool.push_back(constraints.at(i).target);
        vertexTimes.push_back(constraints.at(i).targetTime);
        constraints.at(i).tarPointPoolId = pointPool.size() - 1;
      }
    }

    def.appendVertices(&vertexTimes, originalPointPool);
    def.clearConstraints();

    for (size_t i = 0; i < constraints.size(); i++) {
      if (constraints.at(i).relative) {
        def.addRelativeConstraint(
            constraints.at(i).srcPointPoolId, constraints.at(i).tarPointPoolId);
      } else {
        Eigen::Vector3d targetPoint = constraints.at(i).target;
        def.addConstraint(constraints.at(i).srcPointPoolId, targetPoint);
      }
    }

    float error = 0;
    float meanConsError = 0;
    bool optimised = def.optimiseGraphSparse(
        error, meanConsError, fernMatch, (fernMatch || relaxGraph) ? 0 : lastDeformTime);

    bool poseUpdated = false;

    if (!fernMatch || (fernMatch && optimised && meanConsError < 0.0003 && error < 0.12)) {
      def.applyGraphToPoses(T_wc_ptrs);

      def.applyGraphToVertices();

      if (!fernMatch && newRelativeCons) {
        newRelativeCons->clear();

        // Target to source
        for (size_t i = 0; i < constraints.size(); i++) {
          if (!constraints.at(i).relative && !constraints.at(i).pin) {
            newRelativeCons->push_back(Constraint(
                pointPool.at(constraints.at(i).srcPointPoolId),
                constraints.at(i).target,
                constraints.at(i).srcTime,
                constraints.at(i).targetTime,
                true));
          }
        }
      }

      std::vector<GraphNode*>& graphNodes = def.getGraph();
      std::vector<uint64_t> graphTimes = def.getGraphTimes();

      // 16 floats per node...
      rawGraph.resize(graphNodes.size() * 16);

      for (size_t i = 0; i < graphNodes.size(); i++) {
        const Eigen::Vector3f position = graphNodes.at(i)->position.cast<float>();
        const Eigen::Matrix3f rotation = graphNodes.at(i)->rotation.cast<float>();
        const Eigen::Vector3f translation = graphNodes.at(i)->translation.cast<float>();
        memcpy(&rawGraph.at(i * 16), position.data(), sizeof(float) * 3);
        memcpy(&rawGraph.at(i * 16 + 3), rotation.data(), sizeof(float) * 9);
        memcpy(&rawGraph.at(i * 16 + 12), translation.data(), sizeof(float) * 3);
        rawGraph.at(i * 16 + 15) = (float)graphTimes.at(i);
      }

      if (!fernMatch && !relaxGraph) {
        lastDeformTime = time;
      }

      poseUpdated = true;
    }

    vertexTimes.resize(originalPointPool);
    pointPool.resize(originalPointPool);

    constraints.clear();

    return poseUpdated;
  }

  return false;
}

void Deformation::sampleGraphFrom(Deformation& other) {
  Eigen::Vector4f* otherRawSampledNodes_w = other.getRawSampledNodes_w();

  constexpr int sampleRate = 5;

  if (other.getCount() / sampleRate > def.k) {
    for (int i = 0; i < other.getCount(); i += sampleRate) {
      graphPosePoints->push_back(otherRawSampledNodes_w[i].head<3>().cast<double>());

      if (i > 0 && otherRawSampledNodes_w[i](3) < graphPoseTimes.back()) {
        assert(false && "Assumption failed");
      }

      graphPoseTimes.push_back(otherRawSampledNodes_w[i](3));
    }

    def.initialiseGraph(graphPosePoints, &graphPoseTimes);

    graphPoseTimes.clear();
    graphPosePoints->clear();
  }
}

void Deformation::sampleGraphModel(const std::pair<GLuint, GLuint>& model) {
  sampleProgram->Bind();

  glBindBuffer(GL_ARRAY_BUFFER, model.first);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, 0);

  glEnableVertexAttribArray(1);
  glVertexAttribPointer(
      1, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f)));

  glEnableVertexAttribArray(2);
  glVertexAttribPointer(
      2,
      4,
      GL_FLOAT,
      GL_FALSE,
      Vertex::SIZE,
      reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 2));

  glEnable(GL_RASTERIZER_DISCARD);

  glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, fid);

  glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbo);

  glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, countQuery);

  glBeginTransformFeedback(GL_POINTS);

  glDrawTransformFeedback(GL_POINTS, model.second);

  glEndTransformFeedback();

  glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);

  glGetQueryObjectuiv(countQuery, GL_QUERY_RESULT, &count);

  glDisable(GL_RASTERIZER_DISCARD);

  glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  sampleProgram->Unbind();

  glFinish();

  if ((int)count > def.k) {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Eigen::Vector4f) * count, rawSampledNodes_w);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    for (size_t i = 0; i < count; i++) {
      graphPosePoints->push_back(rawSampledNodes_w[i].head<3>().cast<double>());

      if (i > 0 && rawSampledNodes_w[i](3) < graphPoseTimes.back()) {
        assert(false && "Assumption failed");
      }

      graphPoseTimes.push_back(rawSampledNodes_w[i](3));
    }

    def.initialiseGraph(graphPosePoints, &graphPoseTimes);

    graphPoseTimes.clear();
    graphPosePoints->clear();
  }
}
