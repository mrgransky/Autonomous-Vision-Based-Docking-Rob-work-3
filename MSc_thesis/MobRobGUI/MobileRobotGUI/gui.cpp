#include "gui.h"
#include "ui_gui.h"
#include "visualization.h"

#include<iostream>
#include<stdio.h>

#include <QTime>
using namespace std;

GUI::GUI(QWidget *parent) :QMainWindow(parent),ui(new Ui::GUI)
{
    ui->setupUi(this);
    this->setWindowTitle("GUI for vison-based docking of ROB@WORK 3");

    GoToDocking();
    Dock();
    Point_fiducial_Detection();
    AR_Detection();
    ProgQuit();
    timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(updatePos()));
    timer->start(100);
}

GUI::~GUI()
{
    delete ui;
    delete vizz;
}


void GUI::GoToDocking()
{
    btnGoToDocking.setParent(ui->centralWidget);
    btnGoToDocking.setGeometry(70,20,250,30);
    btnGoToDocking.setText("Go To Docking Platform");
}

void GUI::Dock()
{
    btnDock.setParent(ui->centralWidget);
    btnDock.setGeometry(70,80,250,30);
    btnDock.setText("Dock");
}
void GUI::Point_fiducial_Detection()
{
    btnPF.setParent(ui->centralWidget);
    btnPF.setGeometry(70,140,250,30);
    btnPF.setText("Object Detection");

    Xposlbl_pf.setParent(ui->centralWidget);
    Xpos_pf.setParent(ui->centralWidget);

    Yposlbl_pf.setParent(ui->centralWidget);
    Ypos_pf.setParent(ui->centralWidget);

    Xposlbl_pf.setText("X_pf = ");
    Xposlbl_pf.setGeometry(360,140,50,30);
    Xpos_pf.setGeometry(420,140,80,30);


    Yposlbl_pf.setText("Y_pf = ");
    Yposlbl_pf.setGeometry(520,140,50,30);
    Ypos_pf.setGeometry(580,140,80,30);

    vizz = new Visualization();
    connect(&btnPF,SIGNAL(clicked()),vizz,SLOT(progStart()));
}

void GUI::AR_Detection()
{
    btnAR.setParent(ui->centralWidget);
    btnAR.setGeometry(70,200,250,30);
    btnAR.setText("AR Marker Detection");

    Xposlbl_AR.setParent(ui->centralWidget);
    Xpos_AR.setParent(ui->centralWidget);

    Yposlbl_AR.setParent(ui->centralWidget);
    Ypos_AR.setParent(ui->centralWidget);

    Thetalbl_AR.setParent(ui->centralWidget);
    Theta_AR.setParent(ui->centralWidget);


    Xposlbl_AR.setText("X_AR = ");
    Xposlbl_AR.setGeometry(360,200,50,30);
    Xpos_AR.setGeometry(420,200,80,30);


    Yposlbl_AR.setText("Y_AR = ");
    Yposlbl_AR.setGeometry(520,200,50,30);
    Ypos_AR.setGeometry(580,200,80,30);

    Thetalbl_AR.setText("Theta_AR = ");
    Thetalbl_AR.setGeometry(680,200,90,30);
    Theta_AR.setGeometry(790,200,80,30);

//    vizz = new Visualization();
//    connect(&btnAR,SIGNAL(clicked()),vizz,SLOT(progStart()));

}

void GUI::ProgQuit()
{
    btnExit.setParent(ui->centralWidget);
    btnExit.setGeometry(70,260,250,30);
    btnExit.setText("Exit");
    connect(&btnExit,SIGNAL(clicked()),this,SLOT(close()));
}

void GUI::updatePos()
{
   vizz2 = new Visualization();
   QString numX = QString::number(vizz2->getPosX());
   Xpos_pf.setText(numX);

   vizz3 = new Visualization();
   QString numY = QString::number(vizz3->getPOSY());
   Ypos_pf.setText(numY);
}
