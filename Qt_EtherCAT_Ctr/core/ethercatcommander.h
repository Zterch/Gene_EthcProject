#ifndef ETHERCATCOMMANDER_H
#define ETHERCATCOMMANDER_H

#include <QObject>
#include <QTimer>
#include <QVector>
#include <QMap>
#include <cstdint>
#include "udpclient.h"

#pragma pack(push, 1)

struct SuprRMsg {
    int32_t   mCmd;
    int32_t   mFrameId;
    uint8_t   mModuleFlag[8];
    uint16_t  mDataLength;
    uint16_t  mCRC;
};

struct UpperVelMsg {
    uint8_t  mModuleFlag[8];
    float    mVel[8];
};

struct SlavePdoData {
    uint16_t  statusWord;
    int32_t   actualPosition;
    int32_t   actualVelocity;
    int16_t   actualTorque;
    int32_t   followingError;
    uint16_t  errorCode;
    uint16_t  controlWord;
    int32_t   targetPosition;
    int32_t   targetVelocity;
    int16_t   targetTorque;
};

struct MasterStatusReport {
    int32_t   mFrameId;
    uint32_t  mTimestamp;
    uint8_t   masterState;
    uint8_t   masterALState;
    uint8_t   slaveCount;
    uint8_t   reserved;
    SlavePdoData slaves[8];
    uint32_t  cycleCount;
    uint32_t  domainWC;
};

#pragma pack(pop)

class EtherCATCommander : public QObject
{
    Q_OBJECT

public:
    enum CommandType {
        CMD_HEARTBEAT           = 101,
        CMD_LINK                = 102,
        CMD_UNLINK              = 103,
        CMD_IGH_CREATE_MASTERS  = 105,
        CMD_IGH_SCAN_SLAVES     = 106,
        CMD_IGH_CONF_SERVO_PDOS = 109,
        CMD_IGH_START_OP        = 110,
        CMD_HAND_VEL_MM         = 111,
        CMD_HAND_RELA_POS_MM    = 112,
        CMD_HAND_ABS_POS_M      = 113,
        CMD_IGH_FSA_SHUTDOWN    = 114,
        CMD_IGH_FSA_SWITCH_ON   = 115,
        CMD_IGH_FSA_ENOP        = 116,
        CMD_IGH_FSA_HALT        = 117,
        CMD_IGH_CLEAR_ERROR     = 118,
        CMD_DESTROY_MASTER      = 119,
        CMD_MASTER_OP_KEYDOWN   = 120,
        CMD_PID_READ_PARAMS     = 130,
        CMD_PID_WRITE_PARAMS    = 131,
        CMD_PID_SAVE_TO_FLASH   = 132
    };

    enum SlaveState {
        STATE_UNKNOWN = 0,
        STATE_NOT_READY,
        STATE_SWITCH_ON_DISABLED,
        STATE_READY_TO_SWITCH_ON,
        STATE_SWITCHED_ON,
        STATE_OP,
        STATE_ERROR
    };

    explicit EtherCATCommander(QObject *parent = nullptr);
    ~EtherCATCommander();

    // 连接管理
    bool connectToMaster(const QString &ip, quint16 port);
    void disconnect();
    bool isConnected() const;

    // 轴配置
    void setAxisCount(int count);
    int getAxisCount() const;
    int getActiveAxisCount() const;

    // 主站生命周期控制
    void createMaster();
    void configurePDOs();
    void activateMaster();
    void destroyMaster();
    void scanSlaves();
    void clearErrors();

    // CiA 402 从站状态控制 - 支持多轴
    void driveShutdown(int axisMask = 0x3F);
    void driveSwitchOn(int axisMask = 0x3F);
    void driveEnableOP(int axisMask = 0x3F);
    void driveHalt(int axisMask = 0x3F);
    void quickStop(int axisMask = 0x3F);
    void faultReset(int axisMask = 0x3F);

    // 运动控制命令 - 指定轴
    void jogVelocity(int axis, float velocity);
    void moveRelative(int axis, float position);
    void moveAbsolute(int axis, float position);
    void emergencyStop(int axisMask = 0x3F);

    // 状态查询
    QVector<SlaveState> getSlaveStates() const;
    SlaveState getSlaveState(int axis) const;
    QString getMasterState() const;
    float getActualPosition(int axis) const;
    float getActualVelocity(int axis) const;
    float getActualTorque(int axis) const;

    // PID参数操作
    bool readPidParams(int axis);
    bool writePidParams(int axis, const QVector<QPair<int, float>> &params);
    bool savePidParamsToFlash(int axis);

    // 多轴命令
    bool setVelocities(const QVector<int>& axes, const QVector<float>& velocities);
    bool moveRelative(const QVector<int>& axes, const QVector<float>& positions);
    bool moveAbsolute(const QVector<int>& axes, const QVector<float>& positions);

signals:
    void connectionStateChanged(bool connected);
    void masterStateChanged(const QString &state);
    void slaveStateChanged(int slaveId, SlaveState state);
    void positionUpdated(int axis, float position);
    void velocityUpdated(int axis, float velocity);
    void torqueUpdated(int axis, float torque);
    void errorOccurred(const QString &error);
    void logMessage(const QString &message, bool isError = false);
    void statusReportReceived(const MasterStatusReport &report);
    void pdoDataUpdated(int axis, const SlavePdoData &pdoData);
    void pidParamsReceived(int axis, const QVector<QPair<int, float>> &params);
    void pidWriteResult(int axis, int successCount, int totalCount);
    void pidSaveResult(int axis, bool success);

private slots:
    void onDataReceived(const QByteArray &data);
    void onHeartbeatTimeout();

private:
    QByteArray buildCommandHeader(CommandType cmd, const QByteArray &data = QByteArray());
    QByteArray buildVelocityCommand(int axis, float velocity);
    QByteArray buildPositionCommand(int axis, float position, bool isAbsolute);
    QByteArray buildMultiAxisCommand(CommandType cmd, int axisMask);

    bool sendCommand(CommandType cmd, const QByteArray &data = QByteArray());
    void parseStatusReport(const MasterStatusReport *report);
    void parseSuprRMsg(const SuprRMsg *msg, int totalSize);
    void parsePidParamsResponse(const QByteArray &data);
    QByteArray buildPidReadRequest(int axis);
    QByteArray buildPidWriteRequest(int axis, const QVector<QPair<int, float>> &params);

    SlaveState parseSlaveState(uint16_t statusWord);
    void updateMasterState(uint8_t alState);

    UdpClient *m_udpClient;
    QTimer *m_heartbeatTimer;
    QTimer *m_statusReportTimer;
    bool m_isConnected;
    QString m_masterState;

    QVector<SlaveState> m_slaveStates;
    QVector<float> m_actualPositions;
    QVector<float> m_actualVelocities;
    QVector<float> m_actualTorques;

    int m_heartbeatCounter;
    int m_axisCount;
    int m_activeAxisCount;

    QString m_serverIp;
    quint16 m_serverPort;

    MasterStatusReport m_lastStatusReport;
    bool m_hasReceivedStatus;

    static const int HEARTBEAT_INTERVAL = 1000;
};

#endif // ETHERCATCOMMANDER_H
