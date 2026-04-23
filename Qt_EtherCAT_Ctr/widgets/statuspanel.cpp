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

    // 设置6行（6个轴）
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

    // 实际位置 (转换为度)
    //float actualPos = pdoData.actualPosition * 360.0f / 105906176.0f;
    float actualPos = pdoData.actualPosition / 1000.0f;
    getItem(axis, 4)->setText(QString::number(actualPos, 'f', 3));

    // 目标位置
    float targetPos = pdoData.targetPosition * 360.0f / 105906176.0f;
    getItem(axis, 5)->setText(QString::number(targetPos, 'f', 3));

    // 实际速度 (转换为度/秒)
    float actualVel = (pdoData.actualVelocity * 6.0f) / 101.0f;
    getItem(axis, 6)->setText(QString::number(actualVel, 'f', 3));

    // 目标速度
    float targetVel = pdoData.targetVelocity * 360.0f / 105906176.0f;
    getItem(axis, 7)->setText(QString::number(targetVel, 'f', 3));

    // 实际扭矩 (转换为Nm)
    float actualTorque = pdoData.actualTorque / 1000.0f;
    getItem(axis, 8)->setText(QString::number(actualTorque, 'f', 3));

    // 跟随误差
    float followError = pdoData.followingError * 360.0f / 105906176.0f;
    getItem(axis, 9)->setText(QString::number(followError, 'f', 4));
    if (fabs(followError) > 1.0f) {
        getItem(axis, 9)->setForeground(QColor("red"));
    } else {
        getItem(axis, 9)->setForeground(QColor("black"));
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

    ui->lblCurrentAxisId->setText(QString("轴 %1").arg(axis));
    ui->lblCurrentStateWord->setText(QString("0x%1").arg(pdo.statusWord, 4, 16, QChar('0')).toUpper());

    //float actualPos = pdo.actualPosition * 360.0f / 105906176.0f;
    float actualPos = pdo.actualPosition / 1000.0f;
    ui->lblCurrentPos->setText(QString::number(actualPos, 'f', 3) + " °");

    float actualVel = (pdo.actualVelocity * 6.0f) / 101.0f;
    ui->lblCurrentVel->setText(QString::number(actualVel, 'f', 3) + " °/s");

    float actualTorque = pdo.actualTorque / 1000.0f;
    ui->lblCurrentTorque->setText(QString::number(actualTorque, 'f', 3) + " Nm");
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
