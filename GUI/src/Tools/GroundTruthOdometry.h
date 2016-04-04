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

#ifndef GROUNDTRUTHODOMETRY_H_
#define GROUNDTRUTHODOMETRY_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <iostream>
#include <fstream>
#include <map>
#include <Utils/OdometryProvider.h>

class GroundTruthOdometry
{
    public:
        GroundTruthOdometry(const std::string & filename);

        virtual ~GroundTruthOdometry();

        Eigen::Matrix4f getTransformation(uint64_t timestamp);

        Eigen::MatrixXd getCovariance();

    private:
        void loadTrajectory(const std::string & filename);

        std::map<uint64_t, Eigen::Isometry3f, std::less<int>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Isometry3f> > > camera_trajectory;
        uint64_t last_utime;
};

#endif /* GROUNDTRUTHODOMETRY_H_ */
