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
    std::cout <<"SimpleGUI.->Waiting for current position topic..." << std::endl;
    sensor_msgs::JointState q = *ros::topic::waitForMessage<sensor_msgs::JointState>("/my_gen3/joint_states",ros::Duration(100.0));

    YamlParser yamlParser;
    if(q_predefined_file != "") yamlParser.loadPredefinedPositionsQ(q_predefined_file);
    
    QtRosNode qtRosNode;
    qtRosNode.setNodeHandle(&n);
    qtRosNode.start();

    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.setRosNode(&qtRosNode);
    mainWindow.setYamlParser(&yamlParser);
    mainWindow.initGuiElements(q);
        
    mainWindow.show();
    return app.exec();
}
