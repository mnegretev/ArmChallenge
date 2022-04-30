#pragma once
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

class YamlParser
{
public:
    YamlParser();
    ~YamlParser();

    YAML::Node nodePredefinedQ;
    void loadPredefinedPositionsQ(std::string file);
};
