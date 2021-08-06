#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initUi();

    //timeID_frameUpdate = startTimer(50);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::timerEvent(QTimerEvent *event)
{
    if(event->timerId() == timeID_frameUpdate){
        if (Check_Robot())
            frameUpdate();
    }
}


void MainWindow::initUi()
{
    // set home page
    ui->stackedWidget_workSpace->setCurrentIndex(HOMEPAGE);
    // hide error plot frame
    ui->widget_errorPlot->hide();
    this->statusBar()->showMessage("HI");
}

void MainWindow::frameUpdate()
{
    if (!Check_Robot()){ return; }

    QStringList currentPose = getCurrentPose();

    ui->label_measuresPose_X->setText(currentPose.at(POSE_X));
    ui->label_measuresPose_Y->setText(currentPose.at(POSE_Y));
    ui->label_measuresPose_Z->setText(currentPose.at(POSE_Z));
    ui->label_measuresPose_phi->setText(currentPose.at(POSE_PHI));
    ui->label_measuresPose_theta->setText(currentPose.at(POSE_THETA));
    ui->label_measuresPose_psi->setText(currentPose.at(POSE_PSI));

    ui->label_measuresLength_L1->setText(currentPose.at(LENGTH_1));
    ui->label_measuresLength_L2->setText(currentPose.at(LENGTH_2));
    ui->label_measuresLength_L3->setText(currentPose.at(LENGTH_3));
    ui->label_measuresLength_L4->setText(currentPose.at(LENGTH_4));
    ui->label_measuresLength_L5->setText(currentPose.at(LENGTH_5));
    ui->label_measuresLength_L6->setText(currentPose.at(LENGTH_6));
}

void MainWindow::Select_Robot()
{
    if (ROBOT != NULL){
        delete ROBOT;
        ROBOT = NULL;
    }
    ROBOT = new Item(RDK->ItemUserPick("Select a robot", RoboDK::ITEM_TYPE_ROBOT));
    //ROBOT = new Item(RDK->getItem("UR10", RoboDK::ITEM_TYPE_ROBOT));
    if (Check_Robot()){
        this->statusBar()->showMessage("Robot selected: " + ROBOT->Name());
    }
}

bool MainWindow::Check_RoboDK()
{
    if (RDK == NULL){
        this->statusBar()->showMessage("RoboDK API is not connected");
        return false;
    }
    if (!RDK->Connected()){
        this->statusBar()->showMessage("RoboDK is not running");
        return false;
    }
    return true;
}

bool MainWindow::Check_Robot()
{
    if (!Check_RoboDK()){ return false; }

    if (ROBOT == NULL){
        this->statusBar()->showMessage("Select a robot first");
        return false;
    }
    if (!ROBOT->Valid()){
        this->statusBar()->showMessage("Robot item is not valid");
        return false;
    }
    return true;
}

void MainWindow::IncrementalMove(int id, double sense)
{
    if (!Check_Robot()) { return; }

    // check the index
    if (id < 0 || id >= 6){
        qDebug()<< "Invalid id provided to for an incremental move";
        return;
    }

    // calculate the relative movement
    double step = sense * ui->doubleSpinBox_step->value();

    // apply to XYZWPR
    tXYZWPR xyzwpr;
    for (int i=0; i<6; i++){
        xyzwpr[i] = 0;
    }
    xyzwpr[id] = step;

    Mat pose_increment;
    pose_increment.FromXYZRPW(xyzwpr);

    Mat pose_robot = ROBOT->Pose();

    Mat pose_robot_new;

    // apply relative to the TCP:
    pose_robot_new = pose_robot * pose_increment;

    ROBOT->MoveJ(pose_robot_new);

}

QStringList MainWindow::getCurrentPose()
{
    QString separator = " , ";
    int decimals = 4;

    // Get robot joints
    tJoints joints(ROBOT->Joints());
    QString joints_str = joints.ToString(separator, decimals);

    QStringList jointsStringList = joints_str.replace(";",",").replace("\n",",").split(",", QString::SkipEmptyParts);
    // Get robot pose
    Mat robot_pose(ROBOT->Pose());
    QString pose_str = robot_pose.ToString(separator, decimals);
    QStringList poseStringList = pose_str.replace(";",",").replace("\n",",").split(",", QString::SkipEmptyParts);

    //qDebug() << poseStringList.mid(0, 6) + jointsStringList.mid(0, 6);
    return poseStringList.mid(0, 6) + jointsStringList.mid(0, 6);
}


void MainWindow::on_pushButton_connectRoboDK_clicked()
{
    robodk_window = NULL;
    //adjustSize();

    // Start RoboDK API here (RoboDK will start if it is not running)
    ROBOT = NULL;
    RDK = new RoboDK();
    if (!RDK->Connected()){
        qDebug() << "Failed to start RoboDK API!!";
        this->statusBar()->showMessage("RoboDK is not running");

    }
}




void MainWindow::on_stackedWidget_workSpace_currentChanged(int arg1)
{
    int currentIndex = arg1;

    switch(currentIndex)
    {
    case HOMEPAGE:
        ui->label_frameTitle->setText("Stewart Platform Calibration");
        ui->comboBox_orientationType->setVisible(false);
        ui->pushButton_nextPage->setEnabled(true);
        ui->pushButton_previousPage->setEnabled(false);
        break;

    case PARAMETERPAGE:
        ui->label_frameTitle->setText("Parameter");
        ui->comboBox_orientationType->setVisible(false);
        ui->pushButton_nextPage->setEnabled(true);
        ui->pushButton_previousPage->setEnabled(true);
        break;

    case OPERATIONPAGE:
        ui->label_frameTitle->setText("Operation");
        ui->comboBox_orientationType->setVisible(true);
        ui->pushButton_nextPage->setEnabled(true);
        ui->pushButton_previousPage->setEnabled(true);
        break;

    case TRAJECTORYPAGE:
        ui->label_frameTitle->setText("Trajectory Generation");
        ui->comboBox_orientationType->setVisible(true);
        ui->pushButton_nextPage->setEnabled(true);
        ui->pushButton_previousPage->setEnabled(true);
        break;

    case BATCH_CALIBRATIONPAGE:
        ui->label_frameTitle->setText("Operation");
        ui->comboBox_orientationType->setVisible(true);
        ui->pushButton_nextPage->setEnabled(false);
        ui->pushButton_previousPage->setEnabled(true);
        break;
    }

    //
    // Trajectory Generation
}



void MainWindow::on_pushButton_errorPlot_clicked()
{
    bool isPtnErrorPlotPressed = ui->pushButton_errorPlot->isChecked();
    if(isPtnErrorPlotPressed)
    {
        ui->widget_errorPlot->show();
        ui->widget_errorPlot->raise();
    }
    else
        ui->widget_errorPlot->hide();
}



void MainWindow::on_pushButton_loadModel_clicked()
{
    if (!Check_RoboDK()){ return; }

    QStringList files = QFileDialog::getOpenFileNames(this, ".", tr("Open one or more files with RoboDK"));
    foreach (QString file, files){
        qDebug() << "Loading: " << file;
        RDK->AddFile(file);
    }

    if (!Check_Robot()){
        Select_Robot();
    }
}

void MainWindow::on_pushButton_nextPage_clicked()
{
    int currentIndex = ui->stackedWidget_workSpace->currentIndex();
    ui->stackedWidget_workSpace->setCurrentIndex(currentIndex +1);
}

void MainWindow::on_pushButton_previousPage_clicked()
{
    int currentIndex = ui->stackedWidget_workSpace->currentIndex();
    ui->stackedWidget_workSpace->setCurrentIndex(currentIndex -1);
}
void MainWindow::on_btnTXn_clicked(){ IncrementalMove(0, -1); }
void MainWindow::on_btnTYn_clicked(){ IncrementalMove(1, -1); }
void MainWindow::on_btnTZn_clicked(){ IncrementalMove(2, -1); }
void MainWindow::on_btnRXn_clicked(){ IncrementalMove(3, -1); }
void MainWindow::on_btnRYn_clicked(){ IncrementalMove(4, -1); }
void MainWindow::on_btnRZn_clicked(){ IncrementalMove(5, -1); }

void MainWindow::on_btnTXp_clicked(){ IncrementalMove(0, +1); }
void MainWindow::on_btnTYp_clicked(){ IncrementalMove(1, +1); }
void MainWindow::on_btnTZp_clicked(){ IncrementalMove(2, +1); }
void MainWindow::on_btnRXp_clicked(){ IncrementalMove(3, +1); }
void MainWindow::on_btnRYp_clicked(){ IncrementalMove(4, +1); }
void MainWindow::on_btnRZp_clicked(){ IncrementalMove(5, +1); }

void MainWindow::on_pushButton_moveToHome_clicked()
{
    if (!Check_RoboDK()){ return; }
    tJoints jointsHome(ROBOT->JointsHome());
    ROBOT->MoveJ(jointsHome);
}

void MainWindow::on_pushButton_selectRobot_clicked()
{
    Select_Robot();
}

void MainWindow::on_pushButton_addToolFrame_clicked()
{
    if (!Check_RoboDK()){ return; }
    Mat currentRobotPose;
    ROBOT->AddTool(currentRobotPose, "Tool");

}

void MainWindow::on_radioButton_relativeBase_clicked()
{
    ui->stackedWidget_movement->setCurrentIndex(0);
}

void MainWindow::on_radioButton_relativeOwn_clicked()
{
    ui->stackedWidget_movement->setCurrentIndex(1);
}

void MainWindow::on_pushButton_clicked()
{
    frameUpdate();
}
