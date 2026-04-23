#include "settings.h"

Settings::Settings(QObject *parent)
    : QSettings("Zterch", "EtherCAT_Ctr", parent)
{
}

Settings::~Settings()
{
}

QString Settings::getServerIP() const
{
    return value("Connection/ServerIP", "127.0.0.1").toString();
}

void Settings::setServerIP(const QString &ip)
{
    setValue("Connection/ServerIP", ip);
}

quint16 Settings::getServerPort() const
{
    return value("Connection/ServerPort", 33333).toInt();
}

void Settings::setServerPort(quint16 port)
{
    setValue("Connection/ServerPort", port);
}

QByteArray Settings::getWindowGeometry() const
{
    return value("Window/Geometry").toByteArray();
}

void Settings::setWindowGeometry(const QByteArray &geometry)
{
    setValue("Window/Geometry", geometry);
}

QByteArray Settings::getWindowState() const
{
    return value("Window/State").toByteArray();
}

void Settings::setWindowState(const QByteArray &state)
{
    setValue("Window/State", state);
}
