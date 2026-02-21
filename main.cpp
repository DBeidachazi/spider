#include "mainwindow.h"

#include <QApplication>
#include <QtPlugin>

#ifdef Q_OS_MACOS
Q_IMPORT_PLUGIN(QDarwinCameraPermissionPlugin)
#endif

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
