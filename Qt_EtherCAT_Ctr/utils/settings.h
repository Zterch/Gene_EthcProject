#ifndef SETTINGS_H
#define SETTINGS_H

#include <QSettings>

class Settings : public QSettings
{
    Q_OBJECT

public:
    explicit Settings(QObject *parent = nullptr);
    ~Settings();

    QString getServerIP() const;
    void setServerIP(const QString &ip);

    quint16 getServerPort() const;
    void setServerPort(quint16 port);

    QByteArray getWindowGeometry() const;
    void setWindowGeometry(const QByteArray &geometry);

    QByteArray getWindowState() const;
    void setWindowState(const QByteArray &state);
};

#endif // SETTINGS_H
