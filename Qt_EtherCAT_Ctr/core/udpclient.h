#ifndef UDPCLIENT_H
#define UDPCLIENT_H

#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>

class UdpClient : public QObject
{
    Q_OBJECT

public:
    explicit UdpClient(QObject *parent = nullptr);
    ~UdpClient();

    bool connectToHost(const QString &host, quint16 remotePort, quint16 localPort = 0);
    void disconnect();
    bool isConnected() const;

    bool sendData(const QByteArray &data);
    QByteArray sendAndReceive(const QByteArray &data, int timeout = 2000);

    QString getLocalAddress() const;
    quint16 getLocalPort() const;

signals:
    void dataReceived(const QByteArray &data);
    void connectionStateChanged(bool connected);
    void errorOccurred(const QString &error);

private slots:
    void onReadyRead();
    void onSocketError(QAbstractSocket::SocketError error);

private:
    QUdpSocket *m_socket;
    QHostAddress m_remoteAddress;
    quint16 m_remotePort;
    quint16 m_localPort;
    bool m_isConnected;
};

#endif // UDPCLIENT_H
