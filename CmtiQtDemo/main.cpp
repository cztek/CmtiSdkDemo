#include "CmtiQtDemoDialog.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CmtiQtDemoDialog w;
    w.show();
    return a.exec();
}
