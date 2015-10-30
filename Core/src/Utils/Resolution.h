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

#ifndef RESOLUTION_H_
#define RESOLUTION_H_

#include <cassert>

class Resolution
{
    public:
        static const Resolution & getInstance(int width = 0, int height = 0)
        {
            static const Resolution instance(width, height);
            return instance;
        }

        const int & width() const
        {
            return imgWidth;
        }

        const int & height() const
        {
            return imgHeight;
        }

        const int & cols() const
        {
            return imgWidth;
        }

        const int & rows() const
        {
            return imgHeight;
        }

        const int & numPixels() const
        {
            return imgNumPixels;
        }

    private:
        Resolution(int width, int height)
         : imgWidth(width),
           imgHeight(height),
           imgNumPixels(width * height)
        {
            assert(width > 0 && height > 0 && "You haven't initialised the Resolution class!");
        }

        const int imgWidth;
        const int imgHeight;
        const int imgNumPixels;
};

#endif /* RESOLUTION_H_ */
