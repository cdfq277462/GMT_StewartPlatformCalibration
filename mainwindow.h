#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// TIP: use #define RDK_SKIP_NAMESPACE to avoid using namespaces
#include "robodk_api.h"
using namespace RoboDK_API;

enum WORKINGPAGE{
    HOMEPAGE = 0,
    PARAMETERPAGE = 1,
    OPERATIONPAGE = 2,
    TRAJECTORYPAGE = 3,
    BATCH_CALIBRATIONPAGE = 4

};

enum POSE{
    POSE_X = 0,
    POSE_Y = 1,
    POSE_Z = 2,
    POSE_PHI = 3,
    POSE_THETA = 4,
    POSE_PSI = 5,
};
enum LENGTH{
    LENGTH_1 = 6,
    LENGTH_2 = 7,
    LENGTH_3 = 8,
    LENGTH_4 = 9,
    LENGTH_5 = 10,
    LENGTH_6 = 11,
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void timerEvent(QTimerEvent *event);

    void initUi();

    void frameUpdate();
    /// Select a robot
    void Select_Robot();

    /// Validate if RoboDK is running (RDK is valid)
    bool Check_RoboDK();

    /// Validate if a Robot has been selected (ROBOT variable is valid)
    bool Check_Robot();

    /// Apply an incremental movement
    void IncrementalMove(int id, double sense);

    QStringList getCurrentPose();

    void on_pushButton_nextPage_clicked();

    void on_pushButton_previousPage_clicked();

    void on_pushButton_connectRoboDK_clicked();

    void on_stackedWidget_workSpace_currentChanged(int arg1);

    void on_pushButton_errorPlot_clicked();

    void on_pushButton_loadModel_clicked();

    void on_btnTXn_clicked();
    void on_btnTYn_clicked();
    void on_btnTZn_clicked();
    void on_btnRXn_clicked();
    void on_btnRYn_clicked();
    void on_btnRZn_clicked();
    void on_btnTXp_clicked();
    void on_btnTYp_clicked();
    void on_btnTZp_clicked();
    void on_btnRXp_clicked();
    void on_btnRYp_clicked();
    void on_btnRZp_clicked();

    void on_pushButton_moveToHome_clicked();

    void on_pushButton_selectRobot_clicked();

    void on_pushButton_addToolFrame_clicked();

    void on_radioButton_relativeBase_clicked();

    void on_radioButton_relativeOwn_clicked();

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;

    /// Pointer to RoboDK
    RoboDK *RDK;

    /// Pointer to the robot item
    Item *ROBOT;

    /// Pointer to the RoboDK window
    QWindow *robodk_window;

    int timeID_frameUpdate;
};
#endif // MAINWINDOW_H
