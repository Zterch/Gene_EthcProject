#include "mainwindow.h"

#include <QApplication>

#include "utils/stylehelper.h"
#include <QStyleFactory>
#include <QFile>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // 设置应用程序信息
    app.setApplicationName("EtherCAT Controller");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("Zterch");

    // 设置全局样式
    app.setStyle(QStyleFactory::create("Fusion"));
    StyleHelper::setIndustrialStyle(&app);

    // 创建并显示主窗口
    MainWindow mainWindow;
    mainWindow.setWindowTitle("EtherCAT 主站控制系统-六轴机械臂 v1.0.0");
    mainWindow.show();

    return app.exec();
}
