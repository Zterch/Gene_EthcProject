#include "ethercatcommander.h"
#include <QDataStream>
#include <QDebug>
#include <cstring>

#pragma pack(push, 1)
struct KguPidParamItemPacked {
    uint16_t  paramNo;
    uint16_t  reserved;
    float     value;
    float     minValue;
    float     maxValue;
    uint16_t  isValid;
    char      name[16];
    char      description[32];
};

struct KguPidParamsPacketPacked {
    uint8_t   axisId;
    uint8_t   paramCount;
    uint16_t  resultCode;
    KguPidParamItemPacked params[8];
};
#pragma pack(pop)

EtherCATCommander::EtherCATCommander(QObject *parent)
    : QObject(parent)
    , m_udpClient(new UdpClient(this))
    , m_heartbeatTimer(new QTimer(this))
    , m_statusReportTimer(new QTimer(this))
    , m_isConnected(false)
    , m_heartbeatCounter(0)
    , m_axisCount(6)
    , m_activeAxisCount(6)
    , m_hasReceivedStatus(false)
{
    // 【修复】分开resize和初始化
    m_slaveStates.resize(6);
    for (int i = 0; i < 6; i++) m_slaveStates[i] = STATE_UNKNOWN;

    m_actualPositions.resize(6);
    for (int i = 0; i < 6; i++) m_actualPositions[i] = 0.0f;

    m_actualVelocities.resize(6);
    for (int i = 0; i < 6; i++) m_actualVelocities[i] = 0.0f;

    m_actualTorques.resize(6);
    for (int i = 0; i < 6; i++) m_actualTorques[i] = 0.0f;

    memset(&m_lastStatusReport, 0, sizeof(m_lastStatusReport));

    connect(m_udpClient, &UdpClient::dataReceived, this, &EtherCATCommander::onDataReceived);
    connect(m_udpClient, &UdpClient::connectionStateChanged, this, &EtherCATCommander::connectionStateChanged);
    connect(m_udpClient, &UdpClient::errorOccurred, this, &EtherCATCommander::errorOccurred);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &EtherCATCommander::onHeartbeatTimeout);
}

EtherCATCommander::~EtherCATCommander()
{
    if (m_heartbeatTimer->isActive()) {
        m_heartbeatTimer->stop();
    }
    disconnect();
}

bool EtherCATCommander::connectToMaster(const QString &ip, quint16 port)
{
    m_serverIp = ip;
    m_serverPort = port;

    bool result = m_udpClient->connectToHost(ip, port, 33334);
    if (result) {
        m_isConnected = true;
        m_heartbeatTimer->start(HEARTBEAT_INTERVAL);
        m_masterState = "已连接";
        m_hasReceivedStatus = false;

        QByteArray linkData;
        quint16 localPort = 33334;
        linkData.append(reinterpret_cast<const char*>(&localPort), sizeof(localPort));
        QByteArray linkCmd = buildCommandHeader(CMD_LINK, linkData);
        m_udpClient->sendData(linkCmd);

        emit logMessage(QString("连接到服务器: %1:%2").arg(ip).arg(port), false);
        emit connectionStateChanged(true);
    }
    return result;
}

void EtherCATCommander::disconnect()
{
    if (m_heartbeatTimer->isActive()) {
        m_heartbeatTimer->stop();
    }
    m_udpClient->disconnect();
    m_isConnected = false;
    m_masterState = "未连接";
    m_hasReceivedStatus = false;
    emit logMessage("断开连接", false);
}

bool EtherCATCommander::isConnected() const
{
    return m_isConnected;
}

// setAxisCount函数中的修改（第103-106行）
void EtherCATCommander::setAxisCount(int count)
{
    if (count < 1 || count > 8) return;
    m_axisCount = count;

    // 【修复】分开resize和初始化
    m_slaveStates.resize(count);
    for (int i = 0; i < count; i++) m_slaveStates[i] = STATE_UNKNOWN;

    m_actualPositions.resize(count);
    for (int i = 0; i < count; i++) m_actualPositions[i] = 0.0f;

    m_actualVelocities.resize(count);
    for (int i = 0; i < count; i++) m_actualVelocities[i] = 0.0f;

    m_actualTorques.resize(count);
    for (int i = 0; i < count; i++) m_actualTorques[i] = 0.0f;
}

int EtherCATCommander::getAxisCount() const
{
    return m_axisCount;
}

int EtherCATCommander::getActiveAxisCount() const
{
    return m_activeAxisCount;
}

bool EtherCATCommander::sendCommand(CommandType cmd, const QByteArray &data)
{
    QByteArray msg = buildCommandHeader(cmd, data);
    return m_udpClient->sendData(msg);
}

QByteArray EtherCATCommander::buildCommandHeader(CommandType cmd, const QByteArray &data)
{
    QByteArray msg;
    int headerSize = sizeof(SuprRMsg);
    int totalSize = headerSize + data.size();
    msg.resize(totalSize);

    SuprRMsg *header = reinterpret_cast<SuprRMsg*>(msg.data());
    memset(header, 0, headerSize);

    header->mCmd = cmd;
    header->mFrameId = m_heartbeatCounter++;
    header->mDataLength = static_cast<uint16_t>(data.size());
    for (int i = 0; i < 8; i++) {
        header->mModuleFlag[i] = 0x01;
    }

    if (!data.isEmpty()) {
        memcpy(msg.data() + headerSize, data.constData(), data.size());
    }

    header->mCRC = 0;
    return msg;
}

void EtherCATCommander::createMaster()
{
    sendCommand(CMD_IGH_CREATE_MASTERS);
    emit logMessage("发送命令: 创建主站", false);
}

void EtherCATCommander::destroyMaster()
{
    sendCommand(CMD_DESTROY_MASTER);
    emit logMessage("发送命令: 销毁主站", false);
}

void EtherCATCommander::scanSlaves()
{
    sendCommand(CMD_IGH_SCAN_SLAVES);
    emit logMessage("发送命令: 扫描从站", false);
}

void EtherCATCommander::configurePDOs()
{
    sendCommand(CMD_IGH_CONF_SERVO_PDOS);
    emit logMessage("发送命令: 配置PDO", false);
}

void EtherCATCommander::activateMaster()
{
    sendCommand(CMD_IGH_START_OP);
    emit logMessage("发送命令: 激活主站", false);
}

void EtherCATCommander::clearErrors()
{
    faultReset(0x3F);
    emit logMessage("发送命令: 清除错误", false);
}

// 多轴CiA 402控制
void EtherCATCommander::driveShutdown(int axisMask)
{
    QByteArray data;
    data.append(static_cast<char>(axisMask));
    // 填充到至少29字节，确保超过MSG_LONG_MSG_DATA_OFFSET(28)
    while (data.size() < 29) {
        data.append(static_cast<char>(0));
    }
    sendCommand(CMD_IGH_FSA_SHUTDOWN, data);
    emit logMessage(QString("发送CiA402: Shutdown (轴掩码: 0x%1, 长度:%2)")
                   .arg(axisMask, 2, 16, QChar('0')).arg(data.size()), false);
}

void EtherCATCommander::driveSwitchOn(int axisMask)
{
    QByteArray data;
    data.append(static_cast<char>(axisMask));
    while (data.size() < 29) {
        data.append(static_cast<char>(0));
    }
    sendCommand(CMD_IGH_FSA_SWITCH_ON, data);
    emit logMessage(QString("发送CiA402: Switch On (轴掩码: 0x%1)")
                   .arg(axisMask, 2, 16, QChar('0')), false);
}

void EtherCATCommander::driveEnableOP(int axisMask)
{
    QByteArray data;
    data.append(static_cast<char>(axisMask));
    while (data.size() < 29) {
        data.append(static_cast<char>(0));
    }
    sendCommand(CMD_IGH_FSA_ENOP, data);
    emit logMessage(QString("发送CiA402: Enable Operation (轴掩码: 0x%1)")
                   .arg(axisMask, 2, 16, QChar('0')), false);
}

void EtherCATCommander::driveHalt(int axisMask)
{
    QByteArray data;
    data.append(static_cast<char>(axisMask));
    while (data.size() < 29) {
        data.append(static_cast<char>(0));
    }
    sendCommand(CMD_IGH_FSA_HALT, data);
    emit logMessage(QString("发送CiA402: Halt (轴掩码: 0x%1)")
                   .arg(axisMask, 2, 16, QChar('0')), false);
}

void EtherCATCommander::quickStop(int axisMask)
{
    driveHalt(axisMask);
    emit logMessage("发送CiA402: Quick Stop", false);
}

void EtherCATCommander::faultReset(int axisMask)
{
    QByteArray data;
    data.append(static_cast<char>(axisMask));
    while (data.size() < 29) {
        data.append(static_cast<char>(0));
    }
    sendCommand(CMD_IGH_CLEAR_ERROR, data);
    emit logMessage(QString("发送CiA402: Fault Reset (轴掩码: 0x%1)")
                   .arg(axisMask, 2, 16, QChar('0')), false);
}

void EtherCATCommander::emergencyStop(int axisMask)
{
    driveHalt(axisMask);
    emit logMessage("执行紧急停止", false);
}

// 单轴速度控制
void EtherCATCommander::jogVelocity(int axis, float velocity)
{
    if (axis < 0 || axis >= m_axisCount) {
        emit errorOccurred(QString("轴号错误: %1").arg(axis));
        return;
    }

    QByteArray data = buildVelocityCommand(axis, velocity);
    if (m_udpClient->sendData(data)) {
        emit logMessage(QString("发送速度命令: 轴%1 = %2 度/秒").arg(axis).arg(velocity), false);
    }
}

QByteArray EtherCATCommander::buildVelocityCommand(int axis, float velocity)
{
    QByteArray velData;
    velData.resize(sizeof(UpperVelMsg));
    UpperVelMsg *vel = reinterpret_cast<UpperVelMsg*>(velData.data());

    memset(vel, 0, sizeof(UpperVelMsg));
    vel->mModuleFlag[axis] = 0x01;
    vel->mVel[axis] = velocity;

    return buildCommandHeader(CMD_HAND_VEL_MM, velData);
}

// 单轴位置控制
void EtherCATCommander::moveRelative(int axis, float position)
{
    if (axis < 0 || axis >= m_axisCount) return;

    QByteArray data;
    data.resize(8 + 8 * sizeof(float));

    uint8_t *moduleFlag = reinterpret_cast<uint8_t*>(data.data());
    float *positions = reinterpret_cast<float*>(data.data() + 8);

    memset(data.data(), 0, data.size());
    moduleFlag[axis] = 0x01;
    positions[axis] = position;

    sendCommand(CMD_HAND_RELA_POS_MM, data);
    emit logMessage(QString("发送相对运动: 轴%1 = %2 度").arg(axis).arg(position), false);
}

void EtherCATCommander::moveAbsolute(int axis, float position)
{
    if (axis < 0 || axis >= m_axisCount) return;

    QByteArray data;
    data.resize(8 + 8 * sizeof(float));

    uint8_t *moduleFlag = reinterpret_cast<uint8_t*>(data.data());
    float *positions = reinterpret_cast<float*>(data.data() + 8);

    memset(data.data(), 0, data.size());
    moduleFlag[axis] = 0x01;
    positions[axis] = position;

    sendCommand(CMD_HAND_ABS_POS_M, data);
    emit logMessage(QString("发送绝对运动: 轴%1 = %2 度").arg(axis).arg(position), false);
}

// PID参数操作
bool EtherCATCommander::readPidParams(int axis)
{
    if (!m_isConnected) {
        emit logMessage("错误: 未连接到主站", true);
        return false;
    }
    if (axis < 0 || axis >= m_axisCount) return false;

    QByteArray msg = buildPidReadRequest(axis);
    if (m_udpClient->sendData(msg)) {
        emit logMessage(QString("发送读取PID参数请求: 轴%1").arg(axis), false);
        return true;
    }
    return false;
}

bool EtherCATCommander::writePidParams(int axis, const QVector<QPair<int, float>> &params)
{
    if (!m_isConnected) {
        emit logMessage("错误: 未连接到主站", true);
        return false;
    }
    if (axis < 0 || axis >= m_axisCount) return false;
    if (params.isEmpty()) {
        emit logMessage("错误: 没有要写入的参数", true);
        return false;
    }

    QByteArray msg = buildPidWriteRequest(axis, params);
    if (m_udpClient->sendData(msg)) {
        emit logMessage(QString("发送写入PID参数请求: 轴%1, %2个参数").arg(axis).arg(params.size()), false);
        return true;
    }
    return false;
}

bool EtherCATCommander::savePidParamsToFlash(int axis)
{
    if (!m_isConnected) {
        emit logMessage("错误: 未连接到主站", true);
        return false;
    }
    if (axis < 0 || axis >= m_axisCount) return false;

    QByteArray data;
    data.append(static_cast<char>(axis));

    QByteArray msg = buildCommandHeader(CMD_PID_SAVE_TO_FLASH, data);
    if (m_udpClient->sendData(msg)) {
        emit logMessage(QString("发送保存到Flash请求: 轴%1").arg(axis), false);
        return true;
    }
    return false;
}

QByteArray EtherCATCommander::buildPidReadRequest(int axis)
{
    QByteArray data;
    data.append(static_cast<char>(axis));
    return buildCommandHeader(CMD_PID_READ_PARAMS, data);
}

QByteArray EtherCATCommander::buildPidWriteRequest(int axis, const QVector<QPair<int, float>> &params)
{
    QByteArray data;
    data.append(static_cast<char>(axis));
    data.append(static_cast<char>(qMin(params.size(), 8)));
    data.append(static_cast<char>(0));
    data.append(static_cast<char>(0));

    for (int i = 0; i < params.size() && i < 8; i++) {
        uint16_t paramNo = static_cast<uint16_t>(params[i].first);
        data.append(reinterpret_cast<const char*>(&paramNo), sizeof(paramNo));
        float value = params[i].second;
        data.append(reinterpret_cast<const char*>(&value), sizeof(value));
    }

    return buildCommandHeader(CMD_PID_WRITE_PARAMS, data);
}

// 在 parsePidParamsResponse 函数中添加更详细的调试和容错
void EtherCATCommander::parsePidParamsResponse(const QByteArray &data)
{
    qDebug() << "开始解析PID参数，数据大小:" << data.size();

    if (data.size() < 4) {
        qDebug() << "PID响应太短";
        return;
    }

    uint8_t axisId = static_cast<uint8_t>(data.at(0));
    uint8_t paramCount = static_cast<uint8_t>(data.at(1));

    qDebug() << "PID响应头: axis=" << axisId << "count=" << paramCount;

    if (axisId >= m_axisCount || paramCount == 0 || paramCount > 8) {
        qDebug() << "无效参数";
        return;
    }

    // 【改进】更智能的结构体大小检测
    int actualParamSize = 66;  // 默认
    int payloadSize = data.size() - 4;

    // 计算实际每个参数占用的字节数
    if (payloadSize >= paramCount * 68) {
        // 有足够的数据按68字节解析
        actualParamSize = 68;
        qDebug() << "检测到68字节对齐";
    } else if (payloadSize >= paramCount * 66) {
        // 按66字节解析
        actualParamSize = 66;
        qDebug() << "使用66字节结构体";
    } else {
        // 数据不足，动态计算（向下取整）
        actualParamSize = payloadSize / paramCount;
        qDebug() << "警告: 动态计算参数大小为" << actualParamSize;
    }

    // 【关键改进】用第一个参数验证猜测是否正确
    if (paramCount >= 2) {
        // 读取参数0的paramNo（应该在偏移4）
        uint16_t param0_No = *reinterpret_cast<const uint16_t*>(data.constData() + 4);
        // 读取参数1的paramNo（测试不同步长）
        uint16_t param1_No_66 = *reinterpret_cast<const uint16_t*>(data.constData() + 4 + 66);
        uint16_t param1_No_68 = *reinterpret_cast<const uint16_t*>(data.constData() + 4 + 68);

        qDebug() << "验证: param0=" << param0_No
                 << "param1_66=" << param1_No_66
                 << "param1_68=" << param1_No_68;

        // 参数0应该是31，参数1应该是32
        if (param0_No == 31) {
            if (param1_No_68 == 32) {
                actualParamSize = 68;
                qDebug() << "验证确认: 使用68字节";
            } else if (param1_No_66 == 32) {
                actualParamSize = 66;
                qDebug() << "验证确认: 使用66字节";
            }
        }
    }

    QVector<QPair<int, float>> params;

    for (int i = 0; i < paramCount; i++) {
        int offset = 4 + (i * actualParamSize);

        if (offset + 20 > data.size()) {
            qDebug() << "参数" << i << "超出范围";
            break;
        }

        const char* p = data.constData() + offset;
        uint16_t pNo = *reinterpret_cast<const uint16_t*>(p + 0);
        float value = *reinterpret_cast<const float*>(p + 4);
        uint16_t isValid = *reinterpret_cast<const uint16_t*>(p + 16);

        // 读取name用于调试
        char name[17] = {0};
        memcpy(name, p + 18, 16);

        qDebug() << "  参数" << i << "(偏移" << offset << "): Pn" << pNo
                 << "值=" << value << "isValid=" << isValid << "name=" << name;

        if (isValid != 0 && (pNo == 31 || pNo == 32 || pNo == 33 || pNo == 34 ||
                            pNo == 63 || pNo == 64 || pNo == 65)) {
            params.append(qMakePair((int)pNo, value));
        }
    }

    qDebug() << "解析完成，有效参数:" << params.size();
    emit pidParamsReceived(axisId, params);
}

// 数据解析
void EtherCATCommander::onDataReceived(const QByteArray &data)
{
    qDebug() << "收到数据包，大小:" << data.size() << "字节";

    // 1. 检查是否是 MasterStatusReport (约272字节)
    if (data.size() == sizeof(MasterStatusReport)) {
        const MasterStatusReport *report = reinterpret_cast<const MasterStatusReport*>(data.constData());
        if (report->slaveCount <= 8 && report->masterState <= 10) {
            parseStatusReport(report);
            return;
        }
    }

    // 2. 【关键修复】改进PID响应识别 - 更宽松的条件
    // 主站发送格式: axisId(1) + paramCount(1) + resultCode(2) + params[]
    if (data.size() >= 70) {  // 至少1个参数 (4 + 66 = 70)
        uint8_t axisId = static_cast<uint8_t>(data.at(0));
        uint8_t paramCount = static_cast<uint8_t>(data.at(1));
        uint16_t resultCode;
        memcpy(&resultCode, data.constData() + 2, 2);

        qDebug() << "可能为PID响应: axis=" << axisId
                 << "count=" << paramCount
                 << "result=" << resultCode
                 << "datasize=" << data.size();

        // 校验合理性
        if (axisId < 8 && paramCount >= 1 && paramCount <= 8) {
            // 检查第一个参数号
            uint16_t firstParamNo;
            memcpy(&firstParamNo, data.constData() + 4, 2);

            qDebug() << "第一个参数号:" << firstParamNo;

            // 只要是合理范围内的参数号就接受
            if (firstParamNo >= 31 && firstParamNo <= 9999) {
                qDebug() << "确认为PID参数响应，开始解析";
                parsePidParamsResponse(data);
                return;
            }
        }
    }

    // 3. CMD_LINK 确认 (4字节，值为1)
    if (data.size() == 4) {
        int32_t val = 0;
        memcpy(&val, data.constData(), 4);
        if (val == 1) {
            m_masterState = "已连接";
            emit masterStateChanged(m_masterState);
            emit logMessage("收到连接确认", false);
            return;
        }

        // 【修复】也检查是否为PID错误响应 (axis,0,0,errCode)
        uint8_t axisId = static_cast<uint8_t>(data.at(0));
        uint8_t errCode = static_cast<uint8_t>(data.at(3));
        if (axisId < 8 && data.at(1) == 0 && data.at(2) == 0 && errCode != 0) {
            static const char* errStr[] = {"", "主站未激活", "轴号无效", "AL状态非OP",
                "", "从站未使能", ""};
            QString msg = (errCode < 7 && errStr[errCode][0]) ?
                QString::fromUtf8(errStr[errCode]) : QString("错误码%1").arg(errCode);
            emit logMessage(QString("PID读取失败 轴%1: %2").arg(axisId).arg(msg), true);
            emit pidParamsReceived(axisId, QVector<QPair<int, float>>());
            return;
        }
    }

    // 4. SuprRMsg 封装的消息
    if (data.size() >= (int)sizeof(SuprRMsg)) {
        const SuprRMsg *header = reinterpret_cast<const SuprRMsg*>(data.constData());

        qDebug() << "收到SuprRMsg: cmd=" << header->mCmd
                 << "dataLength=" << header->mDataLength;

        if (header->mCmd >= CMD_HEARTBEAT && header->mCmd <= CMD_PID_SAVE_TO_FLASH) {
            parseSuprRMsg(header, data.size());

            // 【修复】检查SuprRMsg的数据部分是否包含PID参数
            int payloadSize = data.size() - sizeof(SuprRMsg);
            if (payloadSize >= 70) {
                const char* payload = data.constData() + sizeof(SuprRMsg);
                uint8_t axisId = static_cast<uint8_t>(payload[0]);
                uint8_t paramCount = static_cast<uint8_t>(payload[1]);

                if (axisId < 8 && paramCount >= 1 && paramCount <= 8) {
                    uint16_t firstParamNo;
                    memcpy(&firstParamNo, payload + 4, 2);

                    if (firstParamNo >= 31 && firstParamNo <= 9999) {
                        qDebug() << "SuprRMsg中包含PID参数，提取解析";
                        // 提取payload部分单独解析
                        QByteArray pidData(payload, payloadSize);
                        parsePidParamsResponse(pidData);
                        return;
                    }
                }
            }
            return;
        }
    }

    // 5. PID写入响应 (4字节)
    if (data.size() == 4) {
        uint8_t axisId = static_cast<uint8_t>(data.at(0));
        uint8_t successCount = static_cast<uint8_t>(data.at(1));
        uint8_t totalCount = static_cast<uint8_t>(data.at(2));

        if ((axisId < 8 || axisId == 0xFF) && successCount <= totalCount && totalCount <= 8) {
            emit pidWriteResult(axisId, successCount, totalCount);
            return;
        }
    }

    // 6. PID保存响应 (4字节)
    if (data.size() == 4) {
        uint8_t axisId = static_cast<uint8_t>(data.at(0));
        uint8_t success = static_cast<uint8_t>(data.at(1));
        if ((axisId < 8 || axisId == 0xFF) && (success == 0 || success == 1)) {
            emit pidSaveResult(axisId, success == 1);
            return;
        }
    }

    qDebug() << "收到无法识别的数据格式，大小:" << data.size();
}

void EtherCATCommander::parseStatusReport(const MasterStatusReport *report)
{
    memcpy(&m_lastStatusReport, report, sizeof(MasterStatusReport));
    m_hasReceivedStatus = true;
    m_activeAxisCount = qMin((int)report->slaveCount, m_axisCount);

    emit statusReportReceived(*report);

    for (int i = 0; i < m_activeAxisCount; ++i) {
        const SlavePdoData &pdo = report->slaves[i];
        emit pdoDataUpdated(i, pdo);

        float position = pdo.actualPosition /1000;
        float velocity = (pdo.actualVelocity * 6.0f) / 101.0f;
        float torque = pdo.actualTorque / 1000.0f;

        m_actualPositions[i] = position;
        m_actualVelocities[i] = velocity;
        m_actualTorques[i] = torque;

        emit positionUpdated(i, position);
        emit velocityUpdated(i, velocity);
        emit torqueUpdated(i, torque);

        SlaveState newState = parseSlaveState(pdo.statusWord);
        if (m_slaveStates[i] != newState) {
            m_slaveStates[i] = newState;
            emit slaveStateChanged(i, newState);
        }
    }
}

void EtherCATCommander::parseSuprRMsg(const SuprRMsg *msg, int totalSize)
{
    Q_UNUSED(totalSize)

    switch (msg->mCmd) {
        case CMD_HEARTBEAT:
            break;
        case CMD_LINK:
            m_masterState = "已连接";
            emit masterStateChanged(m_masterState);
            emit logMessage("收到连接确认", false);
            break;
        case CMD_IGH_CREATE_MASTERS:
            m_masterState = "主站已创建";
            emit masterStateChanged(m_masterState);
            emit logMessage("主站创建成功", false);
            break;
        case CMD_IGH_SCAN_SLAVES:
            emit logMessage("从站扫描完成", false);
            break;
        case CMD_IGH_START_OP:
            m_masterState = "运行中(OP)";
            emit masterStateChanged(m_masterState);
            emit logMessage("主站已进入OP模式", false);
            break;
        case CMD_IGH_FSA_SWITCH_ON:
            emit logMessage("驱动器已上电", false);
            break;
        case CMD_IGH_FSA_ENOP:
            emit logMessage("驱动器已使能", false);
            break;
        case CMD_DESTROY_MASTER:
            m_masterState = "主站已销毁";
            emit masterStateChanged(m_masterState);
            emit logMessage("主站已销毁", false);
            break;
        default:
            break;
    }
}

EtherCATCommander::SlaveState EtherCATCommander::parseSlaveState(uint16_t statusWord)
{
    if (statusWord & 0x0008) return STATE_ERROR;

    uint8_t stateBits = statusWord & 0x006F;
    switch (stateBits) {
        case 0x0000: return STATE_NOT_READY;
        case 0x0040: return STATE_SWITCH_ON_DISABLED;
        case 0x0021: return STATE_READY_TO_SWITCH_ON;
        case 0x0023: return STATE_SWITCHED_ON;
        case 0x0027: return STATE_OP;
        default:
            if ((statusWord & 0x0237) == 0x0237) return STATE_OP;
            if ((statusWord & 0x0A37) == 0x0A37) return STATE_OP;
            return STATE_UNKNOWN;
    }
}

void EtherCATCommander::updateMasterState(uint8_t alState)
{
    QString newState;
    switch (alState) {
        case 0x00: newState = "未初始化"; break;
        case 0x01: newState = "初始化"; break;
        case 0x02: newState = "预操作"; break;
        case 0x04: newState = "安全操作"; break;
        case 0x08: newState = "运行中(OP)"; break;
        default: newState = QString("AL=0x%1").arg(alState, 2, 16, QChar('0'));
    }

    if (m_masterState != newState) {
        m_masterState = newState;
        emit masterStateChanged(newState);
    }
}

QVector<EtherCATCommander::SlaveState> EtherCATCommander::getSlaveStates() const
{
    return m_slaveStates;
}

EtherCATCommander::SlaveState EtherCATCommander::getSlaveState(int axis) const
{
    if (axis >= 0 && axis < m_slaveStates.size()) {
        return m_slaveStates[axis];
    }
    return STATE_UNKNOWN;
}

QString EtherCATCommander::getMasterState() const
{
    return m_masterState;
}

float EtherCATCommander::getActualPosition(int axis) const
{
    if (axis >= 0 && axis < m_actualPositions.size()) {
        return m_actualPositions[axis];
    }
    return 0.0f;
}

float EtherCATCommander::getActualVelocity(int axis) const
{
    if (axis >= 0 && axis < m_actualVelocities.size()) {
        return m_actualVelocities[axis];
    }
    return 0.0f;
}

float EtherCATCommander::getActualTorque(int axis) const
{
    if (axis >= 0 && axis < m_actualTorques.size()) {
        return m_actualTorques[axis];
    }
    return 0.0f;
}

void EtherCATCommander::onHeartbeatTimeout()
{
    if (!m_isConnected) return;
    m_heartbeatCounter++;
}

bool EtherCATCommander::setVelocities(const QVector<int>& axes, const QVector<float>& velocities)
{
    if (axes.size() != velocities.size() || axes.isEmpty())
        return false;

    // 数据包格式：8字节moduleFlag + 8个float
    QByteArray data;
    data.resize(8 + 8 * sizeof(float));
    uint8_t* moduleFlag = reinterpret_cast<uint8_t*>(data.data());
    float* velArray = reinterpret_cast<float*>(data.data() + 8);

    memset(moduleFlag, 0, 8);
    memset(velArray, 0, 8 * sizeof(float));

    for (int i = 0; i < axes.size(); ++i) {
        int axis = axes[i];
        if (axis < 0 || axis >= m_axisCount) continue;
        moduleFlag[axis] = 0x01;
        velArray[axis] = velocities[i];
    }

    return sendCommand(CMD_HAND_VEL_MM, data);
}

bool EtherCATCommander::moveRelative(const QVector<int>& axes, const QVector<float>& positions)
{
    if (axes.size() != positions.size() || axes.isEmpty())
        return false;

    QByteArray data;
    data.resize(8 + 8 * sizeof(float));
    uint8_t* moduleFlag = reinterpret_cast<uint8_t*>(data.data());
    float* posArray = reinterpret_cast<float*>(data.data() + 8);

    memset(moduleFlag, 0, 8);
    memset(posArray, 0, 8 * sizeof(float));

    for (int i = 0; i < axes.size(); ++i) {
        int axis = axes[i];
        if (axis < 0 || axis >= m_axisCount) continue;
        moduleFlag[axis] = 0x01;
        posArray[axis] = positions[i];
    }

    return sendCommand(CMD_HAND_RELA_POS_MM, data);
}

bool EtherCATCommander::moveAbsolute(const QVector<int>& axes, const QVector<float>& positions)
{
    if (axes.size() != positions.size() || axes.isEmpty())
        return false;

    QByteArray data;
    data.resize(8 + 8 * sizeof(float));
    uint8_t* moduleFlag = reinterpret_cast<uint8_t*>(data.data());
    float* posArray = reinterpret_cast<float*>(data.data() + 8);

    memset(moduleFlag, 0, 8);
    memset(posArray, 0, 8 * sizeof(float));

    for (int i = 0; i < axes.size(); ++i) {
        int axis = axes[i];
        if (axis < 0 || axis >= m_axisCount) continue;
        moduleFlag[axis] = 0x01;
        posArray[axis] = positions[i];
    }

    return sendCommand(CMD_HAND_ABS_POS_M, data);
}
