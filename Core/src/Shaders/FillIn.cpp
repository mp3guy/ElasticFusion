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

#include "FillIn.h"

FillIn::FillIn()
 : imageTexture(Resolution::getInstance().width(),
                Resolution::getInstance().height(),
                GL_RGBA,
                GL_RGB,
                GL_UNSIGNED_BYTE,
                false,
                true),
   vertexTexture(Resolution::getInstance().width(),
                 Resolution::getInstance().height(),
                 GL_RGBA32F,
                 GL_LUMINANCE,
                 GL_FLOAT,
                 false,
                 true),
   normalTexture(Resolution::getInstance().width(),
                 Resolution::getInstance().height(),
                 GL_RGBA32F,
                 GL_LUMINANCE,
                 GL_FLOAT,
                 false,
                 true),
   imageProgram(loadProgramFromFile("empty.vert", "fill_rgb.frag", "quad.geom")),
   imageRenderBuffer(Resolution::getInstance().width(), Resolution::getInstance().height()),
   vertexProgram(loadProgramFromFile("empty.vert", "fill_vertex.frag", "quad.geom")),
   vertexRenderBuffer(Resolution::getInstance().width(), Resolution::getInstance().height()),
   normalProgram(loadProgramFromFile("empty.vert", "fill_normal.frag", "quad.geom")),
   normalRenderBuffer(Resolution::getInstance().width(), Resolution::getInstance().height())
{
    imageFrameBuffer.AttachColour(*imageTexture.texture);
    imageFrameBuffer.AttachDepth(imageRenderBuffer);

    vertexFrameBuffer.AttachColour(*vertexTexture.texture);
    vertexFrameBuffer.AttachDepth(vertexRenderBuffer);

    normalFrameBuffer.AttachColour(*normalTexture.texture);
    normalFrameBuffer.AttachDepth(normalRenderBuffer);
}

FillIn::~FillIn()
{

}

void FillIn::image(GPUTexture * existingRgb, GPUTexture * rawRgb, bool passthrough)
{
    imageFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, imageRenderBuffer.width, imageRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    imageProgram->Bind();

    imageProgram->setUniform(Uniform("eSampler", 0));
    imageProgram->setUniform(Uniform("rSampler", 1));
    imageProgram->setUniform(Uniform("passthrough", (int)passthrough));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, existingRgb->texture->tid);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, rawRgb->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    imageFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    imageProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void FillIn::vertex(GPUTexture * existingVertex, GPUTexture * rawDepth, bool passthrough)
{
    vertexFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, vertexRenderBuffer.width, vertexRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    vertexProgram->Bind();

    vertexProgram->setUniform(Uniform("eSampler", 0));
    vertexProgram->setUniform(Uniform("rSampler", 1));
    vertexProgram->setUniform(Uniform("passthrough", (int)passthrough));

    Eigen::Vector4f cam(Intrinsics::getInstance().cx(),
                  Intrinsics::getInstance().cy(),
                  1.0f / Intrinsics::getInstance().fx(),
                  1.0f / Intrinsics::getInstance().fy());

    vertexProgram->setUniform(Uniform("cam", cam));
    vertexProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    vertexProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, existingVertex->texture->tid);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, rawDepth->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    vertexFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    vertexProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void FillIn::normal(GPUTexture * existingNormal, GPUTexture * rawDepth, bool passthrough)
{
    normalFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, normalRenderBuffer.width, normalRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    normalProgram->Bind();

    normalProgram->setUniform(Uniform("eSampler", 0));
    normalProgram->setUniform(Uniform("rSampler", 1));
    normalProgram->setUniform(Uniform("passthrough", (int)passthrough));

    Eigen::Vector4f cam(Intrinsics::getInstance().cx(),
                  Intrinsics::getInstance().cy(),
                  1.0f / Intrinsics::getInstance().fx(),
                  1.0f / Intrinsics::getInstance().fy());

    normalProgram->setUniform(Uniform("cam", cam));
    normalProgram->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    normalProgram->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, existingNormal->texture->tid);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, rawDepth->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    normalFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    normalProgram->Unbind();

    glPopAttrib();

    glFinish();
}
