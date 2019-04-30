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

#ifndef UTILS_JACOBIAN_H_
#define UTILS_JACOBIAN_H_

#include <vector>

#include "OrderedJacobianRow.h"

class Jacobian
{
    public:
        Jacobian()
         : columns(0)
        {}

        virtual ~Jacobian()
        {
            reset();
        }

        void assign(std::vector<OrderedJacobianRow*> & rows, const int columns)
        {
            reset();
            this->rows = rows;
            this->columns = columns;
        }

        int cols() const
        {
            return columns;
        }

        int nonZero() const
        {
            int count = 0;
            for(size_t i = 0; i < rows.size(); i++)
            {
                count += rows[i]->nonZeros();
            }
            return count;
        }

        std::vector<OrderedJacobianRow*> rows;

    private:
        int columns;

        void reset()
        {
            for(size_t i = 0; i < rows.size(); i++)
            {
                delete rows[i];
            }
            rows.clear();
        }
};



#endif /* UTILS_JACOBIAN_H_ */
