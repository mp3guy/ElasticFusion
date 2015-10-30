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

#ifndef UTILS_IMG_H_
#define UTILS_IMG_H_

#include <Eigen/Core>

template <class T>
class Img
{
    public:
        Img(const int rows, const int cols)
         : rows(rows),
           cols(cols),
           data(new unsigned char[rows * cols * sizeof(T)]),
           owned(true)
        {}

        Img(const int rows, const int cols, T * data)
         : rows(rows),
           cols(cols),
           data((unsigned char *)data),
           owned(false)
        {}

        virtual ~Img()
        {
            if(owned)
            {
                delete [] data;
            }
        }

        const int rows;
        const int cols;
        unsigned char * data;
        const bool owned;

        template<typename V> inline
        V & at(const int i)
        {
            return ((V*)data)[i];
        }

        template<typename V> inline
        V & at(const int row, const int col)
        {
            return ((V*)data)[cols * row + col];
        }

        template<typename V> inline const
        V & at(const int row, const int col) const
        {
            return ((const V*)data)[cols * row + col];
        }
};

#endif /* UTILS_IMG_H_ */
