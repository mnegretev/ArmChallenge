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

void MainWindow::initGuiElements(sensor_msgs::JointState& q0)
{
    ui->gbArticular->setEnabled(false);
    ui->txtAngles1->setValue(q0.position[1]);
    ui->txtAngles2->setValue(q0.position[2]);
    ui->txtAngles3->setValue(q0.position[3]);
    ui->txtAngles4->setValue(q0.position[4]);
    ui->txtAngles5->setValue(q0.position[5]);
    ui->txtAngles6->setValue(q0.position[6]);
    ui->txtAngles7->setValue(q0.position[7]);
    ui->gbArticular->setEnabled(true);
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

    ui->lblCurrentX    ->setText(QString::number(qtRosNode->current_cartesian[0], 'f',3));
    ui->lblCurrentY    ->setText(QString::number(qtRosNode->current_cartesian[1], 'f',3));
    ui->lblCurrentZ    ->setText(QString::number(qtRosNode->current_cartesian[2], 'f',3));
    ui->lblCurrentRoll ->setText(QString::number(qtRosNode->current_cartesian[3], 'f',3));
    ui->lblCurrentPitch->setText(QString::number(qtRosNode->current_cartesian[4], 'f',3));
    ui->lblCurrentYaw  ->setText(QString::number(qtRosNode->current_cartesian[5], 'f',3));
}

void MainWindow::btnXpPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[0] += 0.05;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnXmPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[0] -= 0.05;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnYpPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[1] += 0.05;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnYmPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[1] -= 0.05;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnZpPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[2] += 0.05;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnZmPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[2] -= 0.05;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnRollpPressed()    
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[3] += 0.1;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnRollmPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[3] -= 0.1;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnPitchpPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[4] += 0.1;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnPitchmPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[4] -= 0.1;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnYawpPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[5] += 0.1;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::btnYawmPressed()
{
    std::vector<float> goal_cartesian = qtRosNode->current_cartesian;
    goal_cartesian[5] -= 0.1;
    get_IK_and_update_ui(goal_cartesian);
}

void MainWindow::get_IK_and_update_ui(std::vector<float> cartesian)
{
    std::vector<float> q = qtRosNode->current_q;
    if(!qtRosNode->call_inverse_kinematics(cartesian, q))
    {
        std::cout << "SimpleGUI.->Cannot calculate inverse kinematics." << std::endl;
        return;
    }
    ui->gbArticular->setEnabled(false);
    ui->txtAngles1->setValue(q[0]);
    ui->txtAngles2->setValue(q[1]);
    ui->txtAngles3->setValue(q[2]);
    ui->txtAngles4->setValue(q[3]);
    ui->txtAngles5->setValue(q[4]);
    ui->txtAngles6->setValue(q[5]);
    ui->txtAngles7->setValue(q[6]);
    ui->gbArticular->setEnabled(true);
    qtRosNode->publish_q_goal_angles(q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
}

void MainWindow::sbAnglesValueChanged(double d)
{
    qtRosNode->publish_q_goal_angles(ui->txtAngles1->value(), ui->txtAngles2->value(), ui->txtAngles3->value(),
                                     ui->txtAngles4->value(), ui->txtAngles5->value(), ui->txtAngles6->value(),
                                     ui->txtAngles7->value());
}

void MainWindow::txtArticularGoalReturnPressed()
{
    std::vector<std::string> parts;
    std::string str = this->ui->txtArticularGoal->text().toStdString();
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    std::vector<float> q = qtRosNode->current_q;
    for(size_t i=0; i < parts.size() && i < 7; i++)
    {
        std::stringstream ss(parts[i]);
        if(!(ss >> q[i]))
            q[i] = qtRosNode->current_q[i];
    }
    ui->gbArticular->setEnabled(false);
    ui->txtAngles1->setValue(q[0]);
    ui->txtAngles2->setValue(q[1]);
    ui->txtAngles3->setValue(q[2]);
    ui->txtAngles4->setValue(q[3]);
    ui->txtAngles5->setValue(q[4]);
    ui->txtAngles6->setValue(q[5]);
    ui->txtAngles7->setValue(q[6]);
    ui->gbArticular->setEnabled(true);
    qtRosNode->publish_q_goal_angles(q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
}

void MainWindow::txtCartesianGoalReturnPressed()
{
    std::vector<std::string> parts;
    std::string str = this->ui->txtCartesianGoal->text().toStdString();
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    std::vector<float> cartesian = qtRosNode->current_cartesian;
    for(size_t i=0; i < parts.size() && i < 6; i++)
    {
        std::stringstream ss(parts[i]);
        if(!(ss >> cartesian[i]))
            cartesian[i] = qtRosNode->current_cartesian[i];
    }
    get_IK_and_update_ui(cartesian);
}

void MainWindow::sbGripperValueChanged(double d)
{
    qtRosNode->publish_goal_gripper(d);
}
