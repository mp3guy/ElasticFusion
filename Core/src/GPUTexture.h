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

#ifndef GPUTEXTURE_H_
#define GPUTEXTURE_H_

#include <pangolin/pangolin.h>
#include <driver_types.h>
#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>

class GPUTexture
{
    public:
        GPUTexture(const int width,
                   const int height,
                   const GLenum internalFormat,
                   const GLenum format,
                   const GLenum dataType,
                   const bool draw = false,
                   const bool cuda = false);

        virtual ~GPUTexture();

        static const std::string RGB, DEPTH_RAW, DEPTH_FILTERED, DEPTH_METRIC, DEPTH_METRIC_FILTERED, DEPTH_NORM;

        pangolin::GlTexture * texture;

        cudaGraphicsResource * cudaRes;

        const bool draw;

    private:
        GPUTexture() : texture(0), cudaRes(0), draw(false), width(0), height(0), internalFormat(0), format(0), dataType(0) {}
        const int width;
        const int height;
        const GLenum internalFormat;
        const GLenum format;
        const GLenum dataType;
};

#endif /* GPUTEXTURE_H_ */
