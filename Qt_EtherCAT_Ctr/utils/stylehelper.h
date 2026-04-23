#ifndef STYLEHELPER_H
#define STYLEHELPER_H

#include <QApplication>

class StyleHelper
{
public:
    static void setIndustrialStyle(QApplication *app);

private:
    static QString loadStyleSheet();
};

#endif // STYLEHELPER_H
