#include "statuspanel.h"
#include "ui_statuspanel.h"
#include <QDateTime>
#include <QDebug>
#include <cmath>
#include <QTableWidgetItem>

// 静态成员常量类外定义
const int StatusPanel::MAX_AXES;

StatusPanel::StatusPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StatusPanel),
    m_commander(nullptr),
    m_updateTimer(new QTimer(this)),
    m_currentAxis(0),
    m_runTimeCounter(0)
{
    ui->setupUi(this);

    // 【安全初始化】先检查ui是否正确加载
    if (!ui->tableAxesStatus) {
        qDebug() << "StatusPanel: tableAxesStatus is null!";
        return;
    }

    // 初始化数组
    for (int i = 0; i < MAX_AXES; i++) {
        m_hasPdoData[i] = false;
        memset(&m_lastPdoData[i], 0, sizeof(SlavePdoData));
        m_slaveStates[i] = EtherCATCommander::STATE_UNKNOWN;
    }

    setupUI();
    initAxesTable();

    // 【关键】先连接信号，再启动定时器
    connect(m_updateTimer, &QTimer::timeout, this, &StatusPanel::onUpdateTimer);
    connect(ui->tableAxesStatus, &QTableWidget::cellClicked,
            this, &StatusPanel::onTableCellClicked);

    // 延迟启动定时器，避免初始化期间的竞争条件
    QTimer::singleShot(100, this, [this]() {
        m_updateTimer->start(50);  // 20Hz刷新
    });
}

StatusPanel::~StatusPanel()
{
    if (m_updateTimer) {
        m_updateTimer->stop();
    }
    delete ui;
}

void StatusPanel::setCommander(EtherCATCommander *commander)
{
    // 【安全检查】避免重复连接
    if (m_commander == commander) return;

    // 断开旧连接
    if (m_commander) {
        disconnect(m_commander, nullptr, this, nullptr);
    }

    m_commander = commander;

    if (m_commander) {
        connect(m_commander, &EtherCATCommander::connectionStateChanged,
                this, &StatusPanel::onConnectionStateChanged);
        connect(m_commander, &EtherCATCommander::masterStateChanged,
                this, &StatusPanel::onMasterStateChanged);
        connect(m_commander, &EtherCATCommander::slaveStateChanged,
                this, &StatusPanel::onSlaveStateChanged);
        connect(m_commander, &EtherCATCommander::positionUpdated,
                this, &StatusPanel::onPositionUpdated);
        connect(m_commander, &EtherCATCommander::velocityUpdated,
                this, &StatusPanel::onVelocityUpdated);
        connect(m_commander, &EtherCATCommander::torqueUpdated,
                this, &StatusPanel::onTorqueUpdated);
        connect(m_commander, &EtherCATCommander::pdoDataUpdated,
                this, &StatusPanel::onPdoDataUpdated);
        connect(m_commander, &EtherCATCommander::statusReportReceived,
                this, &StatusPanel::onStatusReportReceived);

        qDebug() << "StatusPanel: 信号连接完成，支持6轴显示";
    }
}

void StatusPanel::setCurrentAxis(int axis)
{
    if (axis >= 0 && axis < MAX_AXES) {
        m_currentAxis = axis;
        updateCurrentAxisDisplay(axis);

        // 高亮表格中的当前行
        if (ui->tableAxesStatus) {
            ui->tableAxesStatus->selectRow(axis);
        }
    }
}

void StatusPanel::setupUI()
{
    if (!ui->lblConnectionStatus || !ui->lblMasterState) return;

    ui->lblConnectionStatus->setStyleSheet("color: red; font-weight: bold;");
    ui->lblMasterState->setStyleSheet("color: gray; font-weight: bold;");

    onConnectionStateChanged(false);
    onMasterStateChanged("未连接");
}

void StatusPanel::initAxesTable()
{
    if (!ui->tableAxesStatus) return;

    // 设置8行（8个轴：6轴旋转+1轴伸缩+1轴力传感器）
    ui->tableAxesStatus->setRowCount(MAX_AXES);

    // 设置列宽
    ui->tableAxesStatus->setColumnWidth(0, 50);   // 轴号
    ui->tableAxesStatus->setColumnWidth(1, 80);  // 状态字
    ui->tableAxesStatus->setColumnWidth(2, 100); // CiA402状态
    ui->tableAxesStatus->setColumnWidth(3, 70);  // 错误码
    ui->tableAxesStatus->setColumnWidth(4, 100); // 实际位置
    ui->tableAxesStatus->setColumnWidth(5, 100); // 目标位置
    ui->tableAxesStatus->setColumnWidth(6, 100); // 实际速度
    ui->tableAxesStatus->setColumnWidth(7, 100); // 目标速度
    ui->tableAxesStatus->setColumnWidth(8, 100); // 实际扭矩
    ui->tableAxesStatus->setColumnWidth(9, 100); // 跟随误差

    // 初始化所有行
    for (int i = 0; i < MAX_AXES; i++) {
        // 轴号
        QTableWidgetItem *itemAxis = new QTableWidgetItem(QString("轴%1").arg(i));
        itemAxis->setTextAlignment(Qt::AlignCenter);
        itemAxis->setFlags(itemAxis->flags() & ~Qt::ItemIsEditable);
        ui->tableAxesStatus->setItem(i, 0, itemAxis);

        // 初始化其他列为"--"
        for (int col = 1; col < 10; col++) {
            QTableWidgetItem *item = new QTableWidgetItem("--");
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            ui->tableAxesStatus->setItem(i, col, item);
        }
    }

    // 默认选中第一行
    ui->tableAxesStatus->selectRow(0);
}

void StatusPanel::updateAxisRow(int axis, const SlavePdoData &pdoData)
{
    if (axis < 0 || axis >= MAX_AXES) return;
    if (!ui->tableAxesStatus) return;

    // 保存数据
    m_lastPdoData[axis] = pdoData;
    m_hasPdoData[axis] = true;

    // 检查item是否存在，不存在则创建
    auto getItem = [&](int row, int col) -> QTableWidgetItem* {
        QTableWidgetItem *item = ui->tableAxesStatus->item(row, col);
        if (!item) {
            item = new QTableWidgetItem("--");
            item->setTextAlignment(Qt::AlignCenter);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            ui->tableAxesStatus->setItem(row, col, item);
        }
        return item;
    };

    // 第八轴(六维力传感器)单独显示
    if (axis == 7) {
        QString axisLabel = QString("轴%1(力传感器)").arg(axis);
        getItem(axis, 0)->setText(axisLabel);
        // 传感器不显示CiA402状态字，显示数据有效状态
        getItem(axis, 1)->setText("N/A");
        getItem(axis, 2)->setText("运行中");
        getItem(axis, 2)->setForeground(QColor("green"));
        getItem(axis, 3)->setText("N/A");
        // 在位置列显示Fx, Fy, Fz
        getItem(axis, 4)->setText("见下方数据");
        getItem(axis, 5)->setText("--");
        getItem(axis, 6)->setText("--");
        getItem(axis, 7)->setText("--");
        getItem(axis, 8)->setText("--");
        getItem(axis, 9)->setText("--");
        return;
    }

    // 轴号标签（第七轴特殊标记）
    QString axisLabel = (axis == 6) ? QString("轴%1(伸缩)").arg(axis) : QString("轴%1").arg(axis);
    getItem(axis, 0)->setText(axisLabel);

    // 状态字 (十六进制)
    QString stateWord = QString("0x%1").arg(pdoData.statusWord, 4, 16, QChar('0')).toUpper();
    getItem(axis, 1)->setText(stateWord);

    // CiA402状态字符串
    QString ciaState = getCiA402StateString(pdoData.statusWord);
    getItem(axis, 2)->setText(ciaState);

    // 根据状态设置颜色
    QString color = "black";
    if (pdoData.statusWord & 0x0008) {
        color = "red";  // 故障
    } else if (pdoData.statusWord == 0x0237 || pdoData.statusWord == 0x0A37) {
        color = "green";  // OP状态
    } else if ((pdoData.statusWord & 0x006F) == 0x0027) {
        color = "blue";  // Operation Enabled
    }
    getItem(axis, 2)->setForeground(QColor(color));

    // 错误码
    if (pdoData.errorCode != 0) {
        QString errCode = QString("0x%1").arg(pdoData.errorCode, 4, 16, QChar('0')).toUpper();
        getItem(axis, 3)->setText(errCode);
        getItem(axis, 3)->setForeground(QColor("red"));
    } else {
        getItem(axis, 3)->setText("无");
        getItem(axis, 3)->setForeground(QColor("green"));
    }

    // 第七轴使用mm单位，其他轴使用度
    if (axis == 6) {
        // 第七轴(伸缩轴): 实际位置单位为mm
        float actualPos = pdoData.actualPosition / 1000.0f;  // mm
        getItem(axis, 4)->setText(QString::number(actualPos, 'f', 3) + " mm");

        // 目标位置
        float targetPos = pdoData.targetPosition / 1000.0f;  // mm
        getItem(axis, 5)->setText(QString::number(targetPos, 'f', 3) + " mm");

        // 实际速度: 计数/秒 -> mm/s
        // 减速比3.705，丝杠导程10mm，编码器131072计数/圈
        // mm/s = counts/s * 导程 / (编码器计数 * 减速比)
        float actualVel = pdoData.actualVelocity * 10.0f / (131072.0f * 3.705f);
        getItem(axis, 6)->setText(QString::number(actualVel, 'f', 3) + " mm/s");

        // 目标速度
        float targetVel = pdoData.targetVelocity * 10.0f / (131072.0f * 3.705f);
        getItem(axis, 7)->setText(QString::number(targetVel, 'f', 3) + " mm/s");

        // 实际扭矩 (转换为Nm)
        float actualTorque = pdoData.actualTorque / 1000.0f;
        getItem(axis, 8)->setText(QString::number(actualTorque, 'f', 3) + " Nm");

        // 跟随误差 (mm)
        float followError = pdoData.followingError / 1000.0f;  // mm
        getItem(axis, 9)->setText(QString::number(followError, 'f', 4) + " mm");
        if (fabs(followError) > 0.5f) {  // 第七轴使用0.5mm容差
            getItem(axis, 9)->setForeground(QColor("red"));
        } else {
            getItem(axis, 9)->setForeground(QColor("black"));
        }
    } else {
        // 普通旋转轴: 使用度单位
        float actualPos = pdoData.actualPosition / 1000.0f;  // 度
        getItem(axis, 4)->setText(QString::number(actualPos, 'f', 3) + " °");

        // 目标位置
        float targetPos = pdoData.targetPosition * 360.0f / 105906176.0f;
        getItem(axis, 5)->setText(QString::number(targetPos, 'f', 3) + " °");

        // 实际速度 (转换为度/秒)
        float actualVel = (pdoData.actualVelocity * 6.0f) / 101.0f;
        getItem(axis, 6)->setText(QString::number(actualVel, 'f', 3) + " °/s");

        // 目标速度
        float targetVel = pdoData.targetVelocity * 360.0f / 105906176.0f;
        getItem(axis, 7)->setText(QString::number(targetVel, 'f', 3) + " °/s");

        // 实际扭矩 (转换为Nm)
        float actualTorque = pdoData.actualTorque / 1000.0f;
        getItem(axis, 8)->setText(QString::number(actualTorque, 'f', 3) + " Nm");

        // 跟随误差 (度)
        float followError = pdoData.followingError * 360.0f / 105906176.0f;
        getItem(axis, 9)->setText(QString::number(followError, 'f', 4) + " °");
        if (fabs(followError) > 1.0f) {
            getItem(axis, 9)->setForeground(QColor("red"));
        } else {
            getItem(axis, 9)->setForeground(QColor("black"));
        }
    }

    // 如果这是当前选中的轴，更新详细显示
    if (axis == m_currentAxis) {
        updateCurrentAxisDisplay(axis);
    }
}

void StatusPanel::updateCurrentAxisDisplay(int axis)
{
    if (axis < 0 || axis >= MAX_AXES || !m_hasPdoData[axis]) return;
    if (!ui->lblCurrentAxisId || !ui->lblCurrentStateWord || !ui->lblCurrentPos) return;

    const SlavePdoData &pdo = m_lastPdoData[axis];

    // 第七轴特殊标记
    QString axisLabel = (axis == 6) ? QString("轴 %1 (伸缩电机)").arg(axis) : QString("轴 %1").arg(axis);
    ui->lblCurrentAxisId->setText(axisLabel);
    ui->lblCurrentStateWord->setText(QString("0x%1").arg(pdo.statusWord, 4, 16, QChar('0')).toUpper());

    if (axis == 6) {
        // 第七轴(伸缩轴): 使用mm单位
        float actualPos = pdo.actualPosition / 1000.0f;  // mm
        ui->lblCurrentPos->setText(QString::number(actualPos, 'f', 3) + " mm");

        // 速度: 计数/秒 -> mm/s
        float actualVel = pdo.actualVelocity * 10.0f / (131072.0f * 3.705f);
        ui->lblCurrentVel->setText(QString::number(actualVel, 'f', 3) + " mm/s");

        float actualTorque = pdo.actualTorque / 1000.0f;
        ui->lblCurrentTorque->setText(QString::number(actualTorque, 'f', 3) + " Nm");
    } else {
        // 普通旋转轴: 使用度单位
        float actualPos = pdo.actualPosition / 1000.0f;  // 度
        ui->lblCurrentPos->setText(QString::number(actualPos, 'f', 3) + " °");

        float actualVel = (pdo.actualVelocity * 6.0f) / 101.0f;
        ui->lblCurrentVel->setText(QString::number(actualVel, 'f', 3) + " °/s");

        float actualTorque = pdo.actualTorque / 1000.0f;
        ui->lblCurrentTorque->setText(QString::number(actualTorque, 'f', 3) + " Nm");
    }
}

void StatusPanel::onConnectionStateChanged(bool connected)
{
    if (!ui->lblConnectionStatus) return;

    ui->lblConnectionStatus->setText(connected ? "已连接" : "未连接");
    QString color = connected ? "green" : "red";
    ui->lblConnectionStatus->setStyleSheet(QString("color: %1; font-weight: bold;").arg(color));

    if (!connected) {
        // 清空表格数据
        if (!ui->tableAxesStatus) return;

        for (int i = 0; i < MAX_AXES; i++) {
            for (int col = 1; col < 10; col++) {
                QTableWidgetItem *item = ui->tableAxesStatus->item(i, col);
                if (item) {
                    item->setText("--");
                    item->setForeground(QColor("black"));
                }
            }
            m_hasPdoData[i] = false;
        }
    }
}

void StatusPanel::onMasterStateChanged(const QString &state)
{
    if (!ui->lblMasterState) return;

    ui->lblMasterState->setText(state);

    QString color = "black";
    if (state.contains("运行") || state.contains("OP")) color = "green";
    else if (state.contains("错误")) color = "red";
    else if (state.contains("停止")) color = "orange";

    ui->lblMasterState->setStyleSheet(QString("color: %1; font-weight: bold;").arg(color));
}

void StatusPanel::onSlaveStateChanged(int slaveId, EtherCATCommander::SlaveState state)
{
    if (slaveId >= 0 && slaveId < MAX_AXES) {
        m_slaveStates[slaveId] = state;
    }
}

void StatusPanel::onPositionUpdated(int axis, float position)
{
    Q_UNUSED(position)
    if (axis == m_currentAxis && m_hasPdoData[axis]) {
        updateCurrentAxisDisplay(axis);
    }
}

void StatusPanel::onVelocityUpdated(int axis, float velocity)
{
    Q_UNUSED(velocity)
    if (axis == m_currentAxis && m_hasPdoData[axis]) {
        updateCurrentAxisDisplay(axis);
    }
}

void StatusPanel::onTorqueUpdated(int axis, float torque)
{
    Q_UNUSED(torque)
    if (axis == m_currentAxis && m_hasPdoData[axis]) {
        updateCurrentAxisDisplay(axis);
    }
}

void StatusPanel::onPdoDataUpdated(int axis, const SlavePdoData &pdoData)
{
    if (axis >= 0 && axis < MAX_AXES) {
        updateAxisRow(axis, pdoData);
    }
}

void StatusPanel::onStatusReportReceived(const MasterStatusReport &report)
{
    // 批量更新所有轴的状态
    int activeCount = qMin((int)report.slaveCount, MAX_AXES);
    for (int i = 0; i < activeCount; i++) {
        updateAxisRow(i, report.slaves[i]);
    }
    
    // 更新六维力传感器数据显示
    updateForceSensorDisplay();
}

void StatusPanel::onUpdateTimer()
{
    m_runTimeCounter++;

    if (!ui->lblRunTime) return;

    int seconds = m_runTimeCounter / 20;  // 50ms间隔
    ui->lblRunTime->setText(QString("%1:%2:%3")
        .arg(seconds / 3600, 2, 10, QChar('0'))
        .arg((seconds % 3600) / 60, 2, 10, QChar('0'))
        .arg(seconds % 60, 2, 10, QChar('0')));
}

void StatusPanel::onTableCellClicked(int row, int column)
{
    Q_UNUSED(column)
    if (row >= 0 && row < MAX_AXES) {
        setCurrentAxis(row);
        qDebug() << "StatusPanel: 用户选择轴" << row;
    }
}

QString StatusPanel::getStateColor(EtherCATCommander::SlaveState state)
{
    switch (state) {
    case EtherCATCommander::STATE_OP: return "green";
    case EtherCATCommander::STATE_SWITCHED_ON: return "blue";
    case EtherCATCommander::STATE_READY_TO_SWITCH_ON: return "orange";
    case EtherCATCommander::STATE_SWITCH_ON_DISABLED: return "gray";
    case EtherCATCommander::STATE_ERROR: return "red";
    default: return "black";
    }
}

QString StatusPanel::getStateString(EtherCATCommander::SlaveState state)
{
    switch (state) {
    case EtherCATCommander::STATE_OP: return "OP";
    case EtherCATCommander::STATE_SWITCHED_ON: return "SwitchedOn";
    case EtherCATCommander::STATE_READY_TO_SWITCH_ON: return "Ready";
    case EtherCATCommander::STATE_SWITCH_ON_DISABLED: return "Disabled";
    case EtherCATCommander::STATE_ERROR: return "Error";
    case EtherCATCommander::STATE_NOT_READY: return "NotReady";
    default: return "Unknown";
    }
}

QString StatusPanel::getCiA402StateString(uint16_t statusWord)
{
    // 检查故障
    if (statusWord & 0x0008) {
        return "故障";
    }

    // 检查KDE特殊状态 (完整16位状态字)
    switch (statusWord) {
        case 0x0A31: return "KDE就绪";
        case 0x0A33: return "KDE准备运动";
        case 0x0A37: return "KDE运行";
        case 0x0A18: return "KDE故障";
        case 0x0A50: return "KDE开关禁用";
        default: break;
    }

    // 标准CiA 402状态字解析 (低6位)
    uint8_t stateBits = statusWord & 0x006F;
    switch (stateBits) {
        case 0x0000: return "未就绪";
        case 0x0040: return "开关禁用";
        case 0x0021: return "准备开关";
        case 0x0023: return "已开关";
        case 0x0027: return "运行使能";
        case 0x0007: return "快速停止";
        default:
            // 检查远程控制位
            if (statusWord & 0x0200) {
                return QString("远程(%1)").arg(stateBits, 2, 16, QChar('0'));
            }
            return QString("未知(%1)").arg(stateBits, 2, 16, QChar('0'));
    }
}

void StatusPanel::updateForceSensorDisplay()
{
    if (!m_commander) return;
    if (!ui->lblFxValue || !ui->lblFyValue || !ui->lblFzValue) return;
    if (!ui->lblMxValue || !ui->lblMyValue || !ui->lblMzValue) return;
    if (!ui->lblForceStatusValue || !ui->lblForceTempValue || !ui->lblForceCounterValue) return;

    // 获取六维力传感器数据
    float fx = m_commander->getForceSensorFx();
    float fy = m_commander->getForceSensorFy();
    float fz = m_commander->getForceSensorFz();
    float mx = m_commander->getTorqueSensorMx();
    float my = m_commander->getTorqueSensorMy();
    float mz = m_commander->getTorqueSensorMz();
    uint32_t status = m_commander->getForceSensorStatus();
    uint32_t counter = m_commander->getForceSensorCounter();
    float temp = m_commander->getForceSensorTemp();

    // 更新显示
    ui->lblFxValue->setText(QString("%1 N").arg(fx, 6, 'f', 3));
    ui->lblFyValue->setText(QString("%1 N").arg(fy, 6, 'f', 3));
    ui->lblFzValue->setText(QString("%1 N").arg(fz, 6, 'f', 3));
    ui->lblMxValue->setText(QString("%1 Nm").arg(mx, 6, 'f', 3));
    ui->lblMyValue->setText(QString("%1 Nm").arg(my, 6, 'f', 3));
    ui->lblMzValue->setText(QString("%1 Nm").arg(mz, 6, 'f', 3));
    ui->lblForceStatusValue->setText(QString("0x%1").arg(status, 8, 16, QChar('0')));
    ui->lblForceTempValue->setText(QString("%1 °C").arg(temp, 0, 'f', 1));
    ui->lblForceCounterValue->setText(QString("%1").arg(counter));
}
