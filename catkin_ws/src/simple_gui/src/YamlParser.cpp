#include "YamlParser.h"

YamlParser::YamlParser()
{
}

YamlParser::~YamlParser()
{
}

void YamlParser::loadPredefinedPositionsQ(std::string file)
{
    nodePredefinedQ = NULL;
    if(file != "")
    {
        std::cout << "YamlParser.->Trying to parse file: " << file << std::endl;
        nodePredefinedQ = YAML::LoadFile(file);
    }
}
