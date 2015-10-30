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

#ifndef INTRINSICS_H_
#define INTRINSICS_H_

#include <cassert>

class Intrinsics
{
    public:
        static const Intrinsics & getInstance(float fx = 0, float fy = 0, float cx = 0, float cy = 0)
        {
            static const Intrinsics instance(fx, fy, cx, cy);
            return instance;
        }

        const float & fx() const
        {
            return fx_;
        }

        const float & fy() const
        {
            return fy_;
        }

        const float & cx() const
        {
            return cx_;
        }

        const float & cy() const
        {
            return cy_;
        }

    private:
        Intrinsics(float fx, float fy, float cx, float cy)
         : fx_(fx),
           fy_(fy),
           cx_(cx),
           cy_(cy)
        {
            assert(fx != 0 && fy != 0 && "You haven't initialised the Intrinsics class!");
        }

        const float fx_, fy_, cx_, cy_;
};

#endif /* INTRINSICS_H_ */
