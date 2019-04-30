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

#ifndef DEFORMATIONGRAPH_H_
#define DEFORMATIONGRAPH_H_

#include <vector>

#include "Stopwatch.h"
#include "GraphNode.h"
#include "Jacobian.h"

/**
 * This is basically and object-oriented type approach. Using an array based approach would be faster...
 */

class CholeskyDecomp;

class DeformationGraph
{
    public:
        DeformationGraph(int k, std::vector<Eigen::Vector3f> * sourceVertices);
        virtual ~DeformationGraph();

        void initialiseGraph(std::vector<Eigen::Vector3f> * customGraph,
                             std::vector<unsigned long long int> * graphTimeMap);

        void appendVertices(std::vector<unsigned long long int> * vertexTimeMap, unsigned int originalPointEnd);

        //This clears the pose map...
        void setPosesSeq(std::vector<unsigned long long int> * poseTimeMap, const std::vector<Eigen::Matrix4f> & poses);

        //Stores a weight and node pointer for a vertex
        class VertexWeightMap
        {
            public:
                VertexWeightMap(double weight, int node)
                : weight(weight),
                  node(node),
                  relative(false)
                {}

                double weight;
                int node;
                bool relative;

                /**
                 * BubblesortLOL
                 * @param list
                 * @param graph
                 */
                static void sort(std::vector<VertexWeightMap> & list, std::vector<GraphNode *> & graph)
                {
                    bool done = false;

                    int size = list.size();

                    while(!done)
                    {
                        done = true;
                        for(int i = 0; i < size - 1; i++)
                        {
                            if(graph.at(list[i].node)->id > graph.at(list[i + 1].node)->id)
                            {
                                done = false;
                                std::swap(list[i], list[i + 1]);
                            }
                        }
                        size--;
                    }
                }
        };

        std::vector<GraphNode *> & getGraph();
        std::vector<unsigned long long int> & getGraphTimes();

        void addConstraint(int vertexId, Eigen::Vector3f & target);
        void addRelativeConstraint(int vertexId, int targetId);

        void clearConstraints();

        void applyGraphToVertices();
        void applyGraphToPoses(std::vector<Eigen::Matrix4f*> & poses);

        bool optimiseGraphSparse(float & error, float & meanConsErr, const bool fernMatch, const unsigned long long int lastDeformTime);
        void resetGraph();

        bool isInit()
        {
            return initialised;
        }

        //Number of neighbours
        const int k;

    private:
        bool initialised;

        //From paper
        const double wRot;
        const double wReg;
        const double wCon;

        static const int numVariables = 12;
        static const int eRotRows = 6;
        static const int eRegRows = 3;
        static const int eConRows = 3;

        //Graph itself
        std::vector<GraphNode> graphNodes;
        std::vector<GraphNode *> graph;

        //Maps vertex indices to neighbours and weights
        std::vector<std::vector<VertexWeightMap>> vertexMap;
        std::vector<Eigen::Vector3f> * sourceVertices;

        //Maps pose indices to neighbours and weights
        std::vector<std::vector<VertexWeightMap>> poseMap;

        //Stores a vertex constraint
        class Constraint
        {
            public:
                Constraint(int vertexId,
                           Eigen::Vector3f & targetPosition)
                 : vertexId(vertexId),
                   targetPosition(targetPosition),
                   relative(false),
                   targetId(-1)
                {}

                Constraint(int vertexId,
                           int targetId)
                 : vertexId(vertexId),
                   targetPosition(Eigen::Vector3f::Zero()),
                   relative(true),
                   targetId(targetId)
                {}

                int vertexId;
                Eigen::Vector3f targetPosition;
                bool relative;
                int targetId;
        };

        std::vector<Constraint> constraints;

        std::vector<Eigen::Vector3f> * graphCloud;
        std::vector<unsigned long long int> sampledGraphTimes;
        unsigned int lastPointCount;

        void connectGraphSeq();

        void weightVerticesSeq(std::vector<unsigned long long int> * vertexTimeMap);

        void computeVertexPosition(int vertexId, Eigen::Vector3f & position);

        void sparseJacobian(Jacobian & jacobian, const int numRows, const int numCols, const int backSet);

        Eigen::VectorXd sparseResidual(const int maxRows);

        void applyDeltaSparse(Eigen::VectorXd & delta);

        CholeskyDecomp * cholesky;

        float nonRelativeConstraintError();
};

#endif /* DEFORMATIONGRAPH_H_ */
