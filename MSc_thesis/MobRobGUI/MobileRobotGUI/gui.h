#ifndef GUI_H
#define GUI_H

#include <QMainWindow>
#include <QPushButton>
#include "visualization.h"
#include <QLineEdit>
#include <QLabel>
#include <QCheckBox>
#include <QRadioButton>
#include <QTimer>

namespace Ui {
class GUI;
}

class GUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit GUI(QWidget *parent = 0);
    ~GUI();

    void GoToDocking();
    void Dock();
    void Point_fiducial_Detection();
    void AR_Detection();
    void ProgQuit();
public slots:
    void updatePos();


private:
    Ui::GUI *ui;

    Visualization *vizz;
    Visualization *vizz2;

    Visualization *vizz3;

    QPushButton btnGoToDocking;

    QPushButton btnDock;

    QPushButton btnPF;
    QPushButton btnAR;

    QPushButton btnExit;
   // QCheckBox btnXpos;
    //QRadioButton btnXX;

    QTimer *timer;

    QLineEdit Xpos_pf;
    QLineEdit Ypos_pf;

    QLabel Xposlbl_pf;
    QLabel Yposlbl_pf;


    QLineEdit Xpos_AR;
    QLineEdit Ypos_AR;
    QLineEdit Theta_AR;

    QLabel Xposlbl_AR;
    QLabel Yposlbl_AR;
    QLabel Thetalbl_AR;

};

#endif // GUI_H
