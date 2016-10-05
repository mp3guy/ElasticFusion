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

#include "Parse.h"

Parse::Parse()
{

}

const Parse & Parse::get()
{
  static const Parse instance;
  return instance;
}

int Parse::arg(int argc, char** argv, const char* str, std::string &val) const
{
    int index = findArg(argc, argv, str) + 1;

    if(index > 0 && index < argc)
    {
        val = argv[index];
    }

    return index - 1;
}

int Parse::arg(int argc, char** argv, const char* str, float &val) const
{
    int index = findArg(argc, argv, str) + 1;

    if(index > 0 && index < argc)
    {
        val = atof(argv[index]);
    }

    return index - 1;
}

int Parse::arg(int argc, char** argv, const char* str, int &val) const
{
    int index = findArg(argc, argv, str) + 1;

    if(index > 0 && index < argc)
    {
        val = atoi(argv[index]);
    }

    return index - 1;
}

std::string Parse::shaderDir() const
{
    std::string currentVal = STR(SHADER_DIR);

    assert(pangolin::FileExists(currentVal) && "Shader directory not found!");

    return currentVal;
}

std::string Parse::baseDir() const
{
    char buf[256];
#ifdef WIN32
    int length = GetModuleFileName(NULL,buf,sizeof(buf));
#else
    int length = readlink("/proc/self/exe",buf,sizeof(buf));
#endif
    std::string currentVal;
    currentVal.append((char *)&buf, length);

    currentVal = currentVal.substr(0, currentVal
#ifdef WIN32
      .rfind("\\build\\"));
#else
      .rfind("/build/"));
#endif
    return currentVal;
}

int Parse::findArg(int argc, char** argv, const char* argument_name) const
{
    for(int i = 1; i < argc; ++i)
    {
        // Search for the string
        if(strcmp(argv[i], argument_name) == 0)
        {
            return i;
        }
    }
    return -1;
}
