/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef DEFORMATION_H_
#define DEFORMATION_H_

#include "Utils/DeformationGraph.h"
#include "Shaders/Shaders.h"
#include "Shaders/Uniform.h"
#include "Shaders/Vertex.h"
#include "GPUTexture.h"
#include "Utils/Resolution.h"
#include "Utils/Intrinsics.h"
#include "Ferns.h"

#include <pangolin/gl/gl.h>

class Deformation
{
    public:
        Deformation();
        virtual ~Deformation();

        std::vector<GraphNode*> & getGraph();

        void getRawGraph(std::vector<float> & graph);

        void sampleGraphModel(const std::pair<GLuint, GLuint> & model);

        void sampleGraphFrom(Deformation & other);

        class Constraint
        {
            public:
                Constraint(const Eigen::Vector3f & src,
                           const Eigen::Vector3f & target,
                           const uint64_t & srcTime,
                           const uint64_t & targetTime,
                           const bool relative,
                           const bool pin = false)
                 : src(src),
                   target(target),
                   srcTime(srcTime),
                   targetTime(targetTime),
                   relative(relative),
                   pin(pin),
                   srcPointPoolId(-1),
                   tarPointPoolId(-1)
                {}

                Eigen::Vector3f src;
                Eigen::Vector3f target;
                uint64_t srcTime;
                uint64_t targetTime;
                bool relative;
                bool pin;
                int srcPointPoolId;
                int tarPointPoolId;
        };

        void addConstraint(const Eigen::Vector4f & src,
                           const Eigen::Vector4f & target,
                           const uint64_t & srcTime,
                           const uint64_t & targetTime,
                           const bool pinConstraints);

        void addConstraint(const Constraint & constraint);

        bool constrain(std::vector<Ferns::Frame*> & ferns,
                       std::vector<float> & rawGraph,
                       int time,
                       const bool fernMatch,
                       std::vector<std::pair<unsigned long long int, Eigen::Matrix4f> > & poseGraph,
                       const bool relaxGraph,
                       std::vector<Constraint> * newRelativeCons = 0);

        Eigen::Vector4f * getVertices()
        {
            return vertices;
        }

        int getCount()
        {
            return int(count);
        }

        int getLastDeformTime()
        {
            return lastDeformTime;
        }

    private:
        DeformationGraph def;

        std::vector<unsigned long long int> vertexTimes;
        std::vector<Eigen::Vector3f> pointPool;
        int originalPointPool;
        int firstGraphNode;

        std::shared_ptr<Shader> sampleProgram;
        GLuint vbo;
        GLuint fid;
        const int bufferSize;
        GLuint countQuery;
        unsigned int count;
        Eigen::Vector4f * vertices;

        std::vector<std::pair<uint64_t, Eigen::Vector3f> > poseGraphPoints;
        std::vector<unsigned long long int> graphPoseTimes;
        std::vector<Eigen::Vector3f> * graphPosePoints;

        std::vector<Constraint> constraints;
        int lastDeformTime;
};

#endif /* DEFORMATION_H_ */
