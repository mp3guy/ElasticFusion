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

#ifndef UTILS_ORDEREDJACOBIANROW_H_
#define UTILS_ORDEREDJACOBIANROW_H_

#include <cassert>
#include <unordered_map>

class OrderedJacobianRow
{
    public:
        OrderedJacobianRow(const int nonZeros)
         : indices(new int[nonZeros]),
           vals(new double[nonZeros]),
           lastSlot(0),
           lastIndex(-1),
           maxNonZero(nonZeros)
        {}

        virtual ~OrderedJacobianRow()
        {
            delete [] indices;
            delete [] vals;
        }

        //You have to use this in an ordered fashion for efficiency :)
        void append(const int index, const double value)
        {
            assert(index > lastIndex);
            indexSlotMap[index] = lastSlot;
            indices[lastSlot] = index;
            vals[lastSlot] = value;
            lastSlot++;
            lastIndex = index;
        }

        //To add to an existing and already weighted value
        void addTo(const int index, const double value, const double weight)
        {
            double & val = vals[indexSlotMap[index]];
            val = ((val / weight) + value) * weight;
        }

        int nonZeros()
        {
            return lastSlot;
        }

        int * indices;
        double * vals;

    private:
        int lastSlot;
        int lastIndex;
        const int maxNonZero;
        std::unordered_map<int, int> indexSlotMap;
};


#endif /* UTILS_ORDEREDJACOBIANROW_H_ */
