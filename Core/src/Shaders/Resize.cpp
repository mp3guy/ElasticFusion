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

#include "Resize.h"

Resize::Resize(int srcWidth,
               int srcHeight,
               int destWidth,
               int destHeight)
: imageTexture(destWidth,
               destHeight,
               GL_RGBA,
               GL_RGB,
               GL_UNSIGNED_BYTE,
               false,
               true),
  vertexTexture(destWidth,
                destHeight,
                GL_RGBA32F,
                GL_LUMINANCE,
                GL_FLOAT,
                false,
                true),
  timeTexture(destWidth,
              destHeight,
              GL_LUMINANCE16UI_EXT,
              GL_LUMINANCE_INTEGER_EXT,
              GL_UNSIGNED_SHORT,
              false,
              true),
  imageProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),
  imageRenderBuffer(destWidth, destHeight),
  vertexProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),
  vertexRenderBuffer(destWidth, destHeight),
  timeProgram(loadProgramFromFile("empty.vert", "resize.frag", "quad.geom")),
  timeRenderBuffer(destWidth, destHeight)
{
   imageFrameBuffer.AttachColour(*imageTexture.texture);
   imageFrameBuffer.AttachDepth(imageRenderBuffer);

   vertexFrameBuffer.AttachColour(*vertexTexture.texture);
   vertexFrameBuffer.AttachDepth(vertexRenderBuffer);

   timeFrameBuffer.AttachColour(*timeTexture.texture);
   timeFrameBuffer.AttachDepth(timeRenderBuffer);
}

Resize::~Resize()
{
}

void Resize::image(GPUTexture * source, Img<Eigen::Matrix<unsigned char, 3, 1>> & dest)
{
    imageFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, imageRenderBuffer.width, imageRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    imageProgram->Bind();

    imageProgram->setUniform(Uniform("eSampler", 0));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    glReadPixels(0, 0, imageRenderBuffer.width, imageRenderBuffer.height, GL_RGB, GL_UNSIGNED_BYTE, dest.data);

    imageFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    imageProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void Resize::vertex(GPUTexture * source, Img<Eigen::Vector4f> & dest)
{
    vertexFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, vertexRenderBuffer.width, vertexRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    vertexProgram->Bind();

    vertexProgram->setUniform(Uniform("eSampler", 0));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    glReadPixels(0, 0, vertexRenderBuffer.width, vertexRenderBuffer.height, GL_RGBA, GL_FLOAT, dest.data);

    vertexFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    vertexProgram->Unbind();

    glPopAttrib();

    glFinish();
}

void Resize::time(GPUTexture * source, Img<unsigned short> & dest)
{
    timeFrameBuffer.Bind();

    glPushAttrib(GL_VIEWPORT_BIT);

    glViewport(0, 0, timeRenderBuffer.width, timeRenderBuffer.height);

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    timeProgram->Bind();

    timeProgram->setUniform(Uniform("eSampler", 0));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source->texture->tid);

    glDrawArrays(GL_POINTS, 0, 1);

    glReadPixels(0, 0, timeRenderBuffer.width, timeRenderBuffer.height, GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_SHORT, dest.data);

    timeFrameBuffer.Unbind();

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    timeProgram->Unbind();

    glPopAttrib();

    glFinish();
}
