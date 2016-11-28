#include "gui.h"
#include "visualization.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GUI w;
    w.showMaximized();

    return a.exec();
}
