#include "stylehelper.h"
#include <QFile>
#include <QPalette>
#include <QFont>  // 添加这行
#include <QDebug>

void StyleHelper::setIndustrialStyle(QApplication *app)
{
    // 设置调色板
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(44, 62, 80));
    palette.setColor(QPalette::WindowText, QColor(236, 240, 241));
    palette.setColor(QPalette::Base, QColor(52, 73, 94));
    palette.setColor(QPalette::AlternateBase, QColor(44, 62, 80));
    palette.setColor(QPalette::ToolTipBase, QColor(44, 62, 80));
    palette.setColor(QPalette::ToolTipText, QColor(236, 240, 241));
    palette.setColor(QPalette::Text, QColor(236, 240, 241));
    palette.setColor(QPalette::Button, QColor(52, 73, 94));
    palette.setColor(QPalette::ButtonText, QColor(236, 240, 241));
    palette.setColor(QPalette::BrightText, QColor(231, 76, 60));
    palette.setColor(QPalette::Link, QColor(52, 152, 219));
    palette.setColor(QPalette::Highlight, QColor(52, 152, 219));
    palette.setColor(QPalette::HighlightedText, Qt::white);
    app->setPalette(palette);

    // 加载样式表
    QString styleSheet = loadStyleSheet();
    if (!styleSheet.isEmpty()) {
        app->setStyleSheet(styleSheet);
    }

    // 设置字体
    QFont font("Noto Sans", 10);
    font.setWeight(QFont::Medium);
    app->setFont(font);
}

QString StyleHelper::loadStyleSheet()
{
    QFile file(":/resources/styles/darkstyle.qss");
    if (file.open(QFile::ReadOnly)) {
        QString styleSheet = QLatin1String(file.readAll());
        file.close();
        qDebug() << "成功加载样式表，大小：" << styleSheet.size();  // 添加调试
        return styleSheet;
    } else {
        qDebug() << "加载样式表失败，使用内置样式";
        // 内置样式...
    }

    // 如果资源文件加载失败，使用内置样式
    return R"(
        QMainWindow {
            background-color: #2C3E50;
        }
        QMenuBar {
            background-color: #34495E;
            color: #ECF0F1;
        }
        QToolBar {
            background-color: #34495E;
            border: none;
        }
        QGroupBox {
            font-weight: bold;
            border: 2px solid #3498DB;
            border-radius: 5px;
            margin-top: 10px;
            color: #ECF0F1;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            left: 10px;
            padding: 0 5px;
            background-color: #2C3E50;
        }
        QPushButton {
            background-color: #3498DB;
            color: white;
            border: none;
            border-radius: 3px;
            padding: 8px 15px;
            font-weight: bold;
            min-width: 80px;
        }
        QPushButton:hover {
            background-color: #2980B9;
        }
        QPushButton:pressed {
            background-color: #1C6EA4;
        }
        QPushButton:disabled {
            background-color: #7F8C8D;
            color: #BDC3C7;
        }
        QLineEdit, QSpinBox, QDoubleSpinBox {
            background-color: #ECF0F1;
            border: 2px solid #BDC3C7;
            border-radius: 3px;
            padding: 5px;
            color: #2C3E50;
        }
        QLabel {
            color: #ECF0F1;
            font-weight: bold;
        }
        QTabWidget::pane {
            border: 1px solid #3498DB;
            background-color: #34495E;
        }
        QTabBar::tab {
            background-color: #2C3E50;
            color: #ECF0F1;
            padding: 8px 15px;
        }
        QTabBar::tab:selected {
            background-color: #3498DB;
            color: white;
        }
        QPlainTextEdit {
            background-color: #ECF0F1;
            color: #2C3E50;
            border: 1px solid #BDC3C7;
            border-radius: 3px;
            font-family: "Monospace";
        }
        QStatusBar {
            background-color: #34495E;
            color: #ECF0F1;
        }
    )";
}
