#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QGraphicsPixmapItem>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>
#include "QtRosNode.h"
#include "YamlParser.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QtRosNode* qtRosNode;
    YamlParser* yamlParser;

    void setRosNode(QtRosNode* qtRosNode);
    void setYamlParser(YamlParser* yamlParser);
    void closeEvent(QCloseEvent *event);
    void initGuiElements(sensor_msgs::JointState& q0);

public slots:
    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();
    void btnXpPressed();
    void btnXmPressed();
    void btnYpPressed();
    void btnYmPressed();
    void btnZpPressed();
    void btnZmPressed();
    void btnRollpPressed();
    void btnRollmPressed();
    void btnPitchpPressed();
    void btnPitchmPressed();
    void btnYawpPressed();
    void btnYawmPressed();  

    void get_IK_and_update_ui(std::vector<float> cartesian);
    void sbAnglesValueChanged(double d);
    void txtArticularGoalReturnPressed();
    void txtCartesianGoalReturnPressed();
    void sbGripperValueChanged(double d);
    
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
