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

#ifndef GRAPHNODE_H_
#define GRAPHNODE_H_

#include <Eigen/Dense>

class GraphNode
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        GraphNode()
        {}

        int id;
        Eigen::Vector3f position;
        Eigen::Matrix3f rotation;
        Eigen::Vector3f translation;
        std::vector<int> neighbours;
        bool enabled;
};

#endif /* GRAPHNODE_H_ */
