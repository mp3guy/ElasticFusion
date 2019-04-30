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

#include "ComputePack.h"

const std::string ComputePack::NORM = "NORM";
const std::string ComputePack::FILTER = "FILTER";
const std::string ComputePack::METRIC = "METRIC";
const std::string ComputePack::METRIC_FILTERED = "METRIC_FILTERED";

ComputePack::ComputePack(std::shared_ptr<Shader> program,
                         pangolin::GlTexture * target)
 : program(program),
   renderBuffer(Resolution::getInstance().width(), Resolution::getInstance().height()),
   target(target)
{
    frameBuffer.AttachColour(*target);
    frameBuffer.AttachDepth(renderBuffer);
}

ComputePack::~ComputePack()
{

}

void ComputePack::compute(pangolin::GlTexture * input, const std::vector<Uniform> * const uniforms)
{
    input->Bind();

    frameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, renderBuffer.width, renderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program->Bind();

    if(uniforms)
    {
        for(size_t i = 0; i < uniforms->size(); i++)
        {
            program->setUniform(uniforms->at(i));
        }
    }

    glDrawArrays(GL_POINTS, 0, 1);

    frameBuffer.Unbind();

    program->Unbind();

    glPopAttrib();

    glFinish();
}
