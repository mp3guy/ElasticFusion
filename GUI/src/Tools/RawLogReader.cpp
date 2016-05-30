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

#include "RawLogReader.h"

RawLogReader::RawLogReader(std::string file, bool flipColors)
 : LogReader(file, flipColors)
{
    assert(pangolin::FileExists(file.c_str()));

    fp = fopen(file.c_str(), "rb");

    currentFrame = 0;

    assert(fread(&numFrames, sizeof(int32_t), 1, fp));

    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
    decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
    decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];
}

RawLogReader::~RawLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;

    fclose(fp);
}

void RawLogReader::getBack()
{
    assert(filePointers.size() > 0);

    fseek(fp, filePointers.top(), SEEK_SET);

    filePointers.pop();

    getCore();
}

void RawLogReader::getNext()
{
    filePointers.push(ftell(fp));

    getCore();
}

void RawLogReader::getCore()
{
    assert(fread(&timestamp, sizeof(int64_t), 1, fp));

    assert(fread(&depthSize, sizeof(int32_t), 1, fp));
    assert(fread(&imageSize, sizeof(int32_t), 1, fp));

    assert(fread(depthReadBuffer, depthSize, 1, fp));

    if(imageSize > 0)
    {
        assert(fread(imageReadBuffer, imageSize, 1, fp));
    }

    if(depthSize == numPixels * 2)
    {
        memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
    }
    else
    {
        unsigned long decompLength = numPixels * 2;
        uncompress(&decompressionBufferDepth[0], (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);
    }

    if(imageSize == numPixels * 3)
    {
        memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);
    }
    else if(imageSize > 0)
    {
        jpeg.readData(imageReadBuffer, imageSize, (unsigned char *)&decompressionBufferImage[0]);
    }
    else
    {
        memset(&decompressionBufferImage[0], 0, numPixels * 3);
    }

    depth = (unsigned short *)decompressionBufferDepth;
    rgb = (unsigned char *)&decompressionBufferImage[0];

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
}

void RawLogReader::fastForward(int frame)
{
    while(currentFrame < frame && hasMore())
    {
        filePointers.push(ftell(fp));

        assert(fread(&timestamp, sizeof(int64_t), 1, fp));

        assert(fread(&depthSize, sizeof(int32_t), 1, fp));
        assert(fread(&imageSize, sizeof(int32_t), 1, fp));

        assert(fread(depthReadBuffer, depthSize, 1, fp));

        if(imageSize > 0)
        {
            assert(fread(imageReadBuffer, imageSize, 1, fp));
        }

        currentFrame++;
    }
}

int RawLogReader::getNumFrames()
{
    return numFrames;
}

bool RawLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}


void RawLogReader::rewind()
{
    if (filePointers.size() != 0)
    {
        std::stack<int> empty;
        std::swap(empty, filePointers);
    }

    fclose(fp);
    fp = fopen(file.c_str(), "rb");

    assert(fread(&numFrames, sizeof(int32_t), 1, fp));

    currentFrame = 0;
}

bool RawLogReader::rewound()
{
    return filePointers.size() == 0;
}

const std::string RawLogReader::getFile()
{
    return file;
}

void RawLogReader::setAuto(bool value)
{

}
