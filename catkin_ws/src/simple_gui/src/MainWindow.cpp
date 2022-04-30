#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    QObject::connect(ui->btnXp    , SIGNAL(pressed()), this, SLOT(btnXpPressed()));
    QObject::connect(ui->btnXm    , SIGNAL(pressed()), this, SLOT(btnXmPressed()));
    QObject::connect(ui->btnYp    , SIGNAL(pressed()), this, SLOT(btnYpPressed()));
    QObject::connect(ui->btnYm    , SIGNAL(pressed()), this, SLOT(btnYmPressed()));
    QObject::connect(ui->btnZp    , SIGNAL(pressed()), this, SLOT(btnZpPressed()));
    QObject::connect(ui->btnZm    , SIGNAL(pressed()), this, SLOT(btnZmPressed()));
    QObject::connect(ui->btnRollp , SIGNAL(pressed()), this, SLOT(btnRollpPressed()));
    QObject::connect(ui->btnRollm , SIGNAL(pressed()), this, SLOT(btnRollmPressed()));
    QObject::connect(ui->btnPitchp, SIGNAL(pressed()), this, SLOT(btnPitchpPressed()));
    QObject::connect(ui->btnPitchm, SIGNAL(pressed()), this, SLOT(btnPitchmPressed()));
    QObject::connect(ui->btnYawp  , SIGNAL(pressed()), this, SLOT(btnYawpPressed()));
    QObject::connect(ui->btnYawm  , SIGNAL(pressed()), this, SLOT(btnYawmPressed()));

    QObject::connect(ui->txtAngles1, SIGNAL(valueChanged(double)), this, SLOT(sbAnglesValueChanged(double)));
    QObject::connect(ui->txtAngles2, SIGNAL(valueChanged(double)), this, SLOT(sbAnglesValueChanged(double)));
    QObject::connect(ui->txtAngles3, SIGNAL(valueChanged(double)), this, SLOT(sbAnglesValueChanged(double)));
    QObject::connect(ui->txtAngles4, SIGNAL(valueChanged(double)), this, SLOT(sbAnglesValueChanged(double)));
    QObject::connect(ui->txtAngles5, SIGNAL(valueChanged(double)), this, SLOT(sbAnglesValueChanged(double)));
    QObject::connect(ui->txtAngles6, SIGNAL(valueChanged(double)), this, SLOT(sbAnglesValueChanged(double)));
    QObject::connect(ui->txtAngles7, SIGNAL(valueChanged(double)), this, SLOT(sbAnglesValueChanged(double)));
    QObject::connect(ui->txtAnglesG, SIGNAL(valueChanged(double)), this, SLOT(sbGripperValueChanged(double)));

    QObject::connect(ui->txtArticularGoal, SIGNAL(returnPressed()), this, SLOT(txtArticularGoalReturnPressed()));
    QObject::connect(ui->txtCartesianGoal, SIGNAL(returnPressed()), this, SLOT(txtCartesianGoalReturnPressed()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;

    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));

}

void MainWindow::setYamlParser(YamlParser* yamlParser)
{
    this->yamlParser = yamlParser;
}

//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    ui->lblCurrentQ1->setText(QString::number(qtRosNode->current_q[0], 'f',3));
    ui->lblCurrentQ2->setText(QString::number(qtRosNode->current_q[1], 'f',3));
    ui->lblCurrentQ3->setText(QString::number(qtRosNode->current_q[2], 'f',3));
    ui->lblCurrentQ4->setText(QString::number(qtRosNode->current_q[3], 'f',3));
    ui->lblCurrentQ5->setText(QString::number(qtRosNode->current_q[4], 'f',3));
    ui->lblCurrentQ6->setText(QString::number(qtRosNode->current_q[5], 'f',3));
    ui->lblCurrentQ7->setText(QString::number(qtRosNode->current_q[6], 'f',3));
}

void MainWindow::btnXpPressed()
{
}

void MainWindow::btnXmPressed()
{
}

void MainWindow::btnYpPressed()
{
}

void MainWindow::btnYmPressed()
{
}

void MainWindow::btnZpPressed()
{
}

void MainWindow::btnZmPressed()
{
}

void MainWindow::btnRollpPressed()    
{
}

void MainWindow::btnRollmPressed()
{
}

void MainWindow::btnPitchpPressed()
{
}

void MainWindow::btnPitchmPressed()
{
}

void MainWindow::btnYawpPressed()
{
}

void MainWindow::btnYawmPressed()
{
}

void MainWindow::sbAnglesValueChanged(double d)
{
    qtRosNode->publish_q_goal_angles(ui->txtAngles1->value(), ui->txtAngles2->value(), ui->txtAngles3->value(),
                                     ui->txtAngles4->value(), ui->txtAngles5->value(), ui->txtAngles6->value(),
                                     ui->txtAngles7->value());
}

void MainWindow::txtArticularGoalReturnPressed()
{
}

void MainWindow::txtCartesianGoalReturnPressed()
{
}

void MainWindow::sbGripperValueChanged(double d)
{
}
