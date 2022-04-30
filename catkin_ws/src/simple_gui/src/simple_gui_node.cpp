#include <iostream>
#include <QApplication>
#include "MainWindow.h"
#include "QtRosNode.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SIMPLE GUI BY MARCOSOFT" << std::endl;
    ros::init(argc, argv, "simple_gui");
    ros::NodeHandle n;


    std::string q_predefined_file = "";
    if(ros::param::has("~q_predefined"))
        ros::param::get("~q_predefined", q_predefined_file);

    YamlParser yamlParser;
    if(q_predefined_file != "") yamlParser.loadPredefinedPositionsQ(q_predefined_file);
    
    QtRosNode qtRosNode;
    qtRosNode.setNodeHandle(&n);
    qtRosNode.start();

    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.setRosNode(&qtRosNode);
    mainWindow.setYamlParser(&yamlParser);
        
    mainWindow.show();
    return app.exec();
}
