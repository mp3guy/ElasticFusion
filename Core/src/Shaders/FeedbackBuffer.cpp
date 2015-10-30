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

#include "FeedbackBuffer.h"

const std::string FeedbackBuffer::RAW = "RAW";
const std::string FeedbackBuffer::FILTERED = "FILTERED";

FeedbackBuffer::FeedbackBuffer(std::shared_ptr<Shader> program)
 : program(program),
   drawProgram(loadProgramFromFile("draw_feedback.vert", "draw_feedback.frag")),
   bufferSize(Resolution::getInstance().numPixels() * Vertex::SIZE),
   count(0)
{
    float * vertices = new float[bufferSize];

    memset(&vertices[0], 0, bufferSize);

    glGenTransformFeedbacks(1, &fid);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, bufferSize, &vertices[0], GL_STREAM_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    delete [] vertices;

    std::vector<Eigen::Vector2f> uv;

    for(int i = 0; i < Resolution::getInstance().width(); i++)
    {
        for(int j = 0; j < Resolution::getInstance().height(); j++)
        {
            uv.push_back(Eigen::Vector2f(((float)i / (float)Resolution::getInstance().width()) + 1.0 / (2 * (float)Resolution::getInstance().width()),
                                   ((float)j / (float)Resolution::getInstance().height()) + 1.0 / (2 * (float)Resolution::getInstance().height())));
        }
    }

    glGenBuffers(1, &uvo);
    glBindBuffer(GL_ARRAY_BUFFER, uvo);
    glBufferData(GL_ARRAY_BUFFER, uv.size() * sizeof(Eigen::Vector2f), &uv[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    program->Bind();

    int loc[3] =
    {
        glGetVaryingLocationNV(program->programId(), "vPosition0"),
        glGetVaryingLocationNV(program->programId(), "vColor0"),
        glGetVaryingLocationNV(program->programId(), "vNormRad0"),
    };

    glTransformFeedbackVaryingsNV(program->programId(), 3, loc, GL_INTERLEAVED_ATTRIBS);

    program->Unbind();

    glGenQueries(1, &countQuery);
}

FeedbackBuffer::~FeedbackBuffer()
{
    glDeleteTransformFeedbacks(1, &fid);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &uvo);
    glDeleteQueries(1, &countQuery);
}

void FeedbackBuffer::compute(pangolin::GlTexture * color,
                             pangolin::GlTexture * depth,
                             const int & time,
                             const float depthCutoff)
{
    program->Bind();

    Eigen::Vector4f cam(Intrinsics::getInstance().cx(),
                  Intrinsics::getInstance().cy(),
                  1.0f / Intrinsics::getInstance().fx(),
                  1.0f / Intrinsics::getInstance().fy());

    program->setUniform(Uniform("cam", cam));
    program->setUniform(Uniform("threshold", 0.0f));
    program->setUniform(Uniform("cols", (float)Resolution::getInstance().cols()));
    program->setUniform(Uniform("rows", (float)Resolution::getInstance().rows()));
    program->setUniform(Uniform("time", time));
    program->setUniform(Uniform("gSampler", 0));
    program->setUniform(Uniform("cSampler", 1));
    program->setUniform(Uniform("maxDepth", depthCutoff));

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, uvo);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glEnable(GL_RASTERIZER_DISCARD);

    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, fid);

    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, vbo);

    glBeginTransformFeedback(GL_POINTS);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depth->tid);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, color->tid);

    glDrawArrays(GL_POINTS, 0, Resolution::getInstance().numPixels());

    glBindTexture(GL_TEXTURE_2D, 0);

    glActiveTexture(GL_TEXTURE0);

    glEndTransformFeedback();

    glDisable(GL_RASTERIZER_DISCARD);

    glDisableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);

    program->Unbind();

    glFinish();
}

void FeedbackBuffer::render(pangolin::OpenGlMatrix mvp,
                            const Eigen::Matrix4f & pose,
                            const bool drawNormals,
                            const bool drawColors)
{
    drawProgram->Bind();

    drawProgram->setUniform(Uniform("MVP", mvp));
    drawProgram->setUniform(Uniform("pose", pose));
    drawProgram->setUniform(Uniform("colorType", (drawNormals ? 1 : drawColors ? 2 : 0)));

    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, 0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 1));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, Vertex::SIZE, reinterpret_cast<GLvoid*>(sizeof(Eigen::Vector4f) * 2));

    glDrawTransformFeedback(GL_POINTS, fid);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    drawProgram->Unbind();
}
