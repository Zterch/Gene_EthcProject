#include "udpclient.h"
#include <QNetworkDatagram>
#include <QDebug>
#include <QEventLoop>  // 添加这行
#include <QTimer>       // 确保QTimer已包含

UdpClient::UdpClient(QObject *parent)
    : QObject(parent)
    , m_socket(nullptr)
    , m_remotePort(0)
    , m_localPort(0)
    , m_isConnected(false)
{
    m_socket = new QUdpSocket(this);

    connect(m_socket, &QUdpSocket::readyRead, this, &UdpClient::onReadyRead);
    connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this, &UdpClient::onSocketError);
}

UdpClient::~UdpClient()
{
    if (m_socket) {
        m_socket->close();
    }
    m_isConnected = false;
}

bool UdpClient::connectToHost(const QString &host, quint16 remotePort, quint16 localPort)
{
    if (m_isConnected) {
        if (m_socket) {
            m_socket->close();
        }
        m_isConnected = false;
    }

    m_remoteAddress = QHostAddress(host);
    m_remotePort = remotePort;

    if (localPort > 0) {
        if (!m_socket->bind(QHostAddress::Any, localPort)) {
            qDebug() << "绑定本地端口失败:" << m_socket->errorString();
            return false;
        }
    } else {
        if (!m_socket->bind()) {
            qDebug() << "绑定套接字失败:" << m_socket->errorString();
            return false;
        }
    }

    m_localPort = m_socket->localPort();
    m_isConnected = true;

    qDebug() << "UDP连接成功:" << host << ":" << remotePort << "本地端口:" << m_localPort;
    emit connectionStateChanged(true);

    return true;
}

void UdpClient::disconnect()
{
    if (m_socket) {
        m_socket->close();
    }

    m_isConnected = false;
    emit connectionStateChanged(false);
}

bool UdpClient::isConnected() const
{
    return m_isConnected;
}

bool UdpClient::sendData(const QByteArray &data)
{
    if (!m_isConnected || !m_socket) {
        qDebug() << "发送失败: 未连接";
        return false;
    }

    qint64 bytesSent = m_socket->writeDatagram(data, m_remoteAddress, m_remotePort);
    if (bytesSent != data.size()) {
        qDebug() << "发送数据失败: 期望发送" << data.size() << "字节，实际发送" << bytesSent << "字节";
        return false;
    }

    qDebug() << "发送数据成功:" << bytesSent << "字节";
    return true;
}

// 简化sendAndReceive函数，避免复杂的异步等待
QByteArray UdpClient::sendAndReceive(const QByteArray &data, int timeout)
{
    if (!sendData(data)) {
        return QByteArray();
    }

    // 简单实现：阻塞等待一小段时间
    QByteArray receivedData;
    QEventLoop loop;
    QTimer timer;

    connect(this, &UdpClient::dataReceived, &loop, &QEventLoop::quit);
    timer.setSingleShot(true);
    timer.start(timeout);
    connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

    loop.exec();

    // 暂时返回空数据，实际应用中应返回接收到的数据
    return receivedData;
}

QString UdpClient::getLocalAddress() const
{
    if (m_socket) {
        return m_socket->localAddress().toString();
    }
    return QString();
}

quint16 UdpClient::getLocalPort() const
{
    return m_localPort;
}

void UdpClient::onReadyRead()
{
    while (m_socket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_socket->receiveDatagram();
        QByteArray data = datagram.data();
        emit dataReceived(data);
        qDebug() << "收到数据:" << data.size() << "字节";
    }
}

void UdpClient::onSocketError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    QString errorMsg = m_socket->errorString();
    qDebug() << "Socket错误:" << errorMsg;
    emit errorOccurred(errorMsg);
}
