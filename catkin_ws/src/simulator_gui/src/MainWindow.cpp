#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
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
    QObject::connect(ui->hsA, SIGNAL(valueChanged(int)), this, SLOT(hsSemimajorAxisChanged    (int)));
    QObject::connect(ui->hsE, SIGNAL(valueChanged(int)), this, SLOT(hsEccentricityChanged     (int)));
    QObject::connect(ui->hsN, SIGNAL(valueChanged(int)), this, SLOT(hsAscendingNodeChanged    (int)));
    QObject::connect(ui->hsI, SIGNAL(valueChanged(int)), this, SLOT(hsInclinationChange       (int)));
    QObject::connect(ui->hsW, SIGNAL(valueChanged(int)), this, SLOT(hsArgumentPeriapsisChanged(int)));
    QObject::connect(ui->hsM, SIGNAL(valueChanged(int)), this, SLOT(hsMeanAnomalyChanged      (int)));
    QObject::connect(ui->txtA, SIGNAL(returnPressed()), this, SLOT(txtSemimajorAxisReturnPressed    ()));
    QObject::connect(ui->txtE, SIGNAL(returnPressed()), this, SLOT(txtEccentricityReturnPressed     ()));
    QObject::connect(ui->txtN, SIGNAL(returnPressed()), this, SLOT(txtAscendingNodeReturnPressed    ()));
    QObject::connect(ui->txtI, SIGNAL(returnPressed()), this, SLOT(txtInclinationReturnPressed      ()));
    QObject::connect(ui->txtW, SIGNAL(returnPressed()), this, SLOT(txtArgumentPeriapsisReturnPressed()));
    QObject::connect(ui->txtM, SIGNAL(returnPressed()), this, SLOT(txtMeanAnomalyReturnPressed      ()));
    QObject::connect(ui->btnContinuous, SIGNAL(clicked()), this, SLOT(btnContinuousClicked()));
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
}

//
//SLOTS FOR SIGNALS EMITTED BY CONTROLS
//
void MainWindow::hsSemimajorAxisChanged    (int value)
{
    qtRosNode->publishSemimajorAxis(value/10.0);
}

void MainWindow::hsEccentricityChanged     (int value)
{
    qtRosNode->publishEccentricity(value/100.0);
}

void MainWindow::hsAscendingNodeChanged    (int value)
{
    qtRosNode->setAscendingNode(value*2*M_PI/100);
}

void MainWindow::hsInclinationChange       (int value)
{
    qtRosNode->setInclination(value*2*M_PI/100);
}

void MainWindow::hsArgumentPeriapsisChanged(int value)
{
    qtRosNode->setArgumentOfPeriapsis(value*2*M_PI/100);
}

void MainWindow::hsMeanAnomalyChanged      (int value)
{
    qtRosNode->publishMeanAnomaly(value*2*M_PI/100);
}

void MainWindow::txtSemimajorAxisReturnPressed    ()
{
}

void MainWindow::txtEccentricityReturnPressed     ()
{
}

void MainWindow::txtAscendingNodeReturnPressed    ()
{
}

void MainWindow::txtInclinationReturnPressed      ()
{
}

void MainWindow::txtArgumentPeriapsisReturnPressed()
{
}

void MainWindow::txtMeanAnomalyReturnPressed      ()
{
}

void MainWindow::btnContinuousClicked()
{
    if(ui->hsM->isEnabled())
    {
        ui->hsM->setEnabled(false);
        qtRosNode->current_mean_anomaly = ui->hsM->value()*2*M_PI/99;
        qtRosNode->continuous_mean_anomaly = true;
    }
    else
    {
        qtRosNode->continuous_mean_anomaly = false;
        ui->hsM->setValue((int)(qtRosNode->current_mean_anomaly/(2*M_PI)*99));
        ui->hsM->setEnabled(true);
    }
}
