#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "utils/settings.h"
#include <QMessageBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDebug>
#include <QTextCursor>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_commander(new EtherCATCommander(this))
    , m_statusPanel(new StatusPanel(this))
    , m_pidConfigTab(new PidConfigTab(this))
    , m_currentAxis(0)
{
    ui->setupUi(this);
    // 在 ui->setupUi(this) 之后
    m_axisCheckGroup = new QButtonGroup(this);
    m_axisCheckGroup->setExclusive(false);

    // 使用列表存储指针，并检查空值 (8轴，含第七轴伸缩电机和第八轴六维力传感器)
    QList<QCheckBox*> axisChecks = {
        ui->chkAxis0, ui->chkAxis1, ui->chkAxis2,
        ui->chkAxis3, ui->chkAxis4, ui->chkAxis5, ui->chkAxis6, ui->chkAxis7
    };

    for (int i = 0; i < axisChecks.size(); ++i) {
        if (axisChecks[i]) {
            m_axisCheckGroup->addButton(axisChecks[i], i);
        } else {
            qDebug() << "警告: 复选框 chkAxis" << i << "为空，请检查 UI 文件";
        }
    }

    setWindowTitle("EtherCAT 主站控制系统 v2.0 - 8轴(6轴+伸缩+力传感器)");

    // 【关键修复】检查statusTab是否有布局，没有则创建
    if (!ui->statusTab->layout()) {
        QVBoxLayout *layout = new QVBoxLayout(ui->statusTab);
        ui->statusTab->setLayout(layout);
    }

    // 设置状态面板
    m_statusPanel->setCommander(m_commander);
    ui->statusTab->layout()->addWidget(m_statusPanel);

    // 创建并添加PID配置Tab
    m_pidConfigTab->setCommander(m_commander);
    ui->tabWidget->addTab(m_pidConfigTab, "PID参数配置");

    // 初始化轴选择下拉框 - 8轴(含第七轴伸缩电机和第八轴六维力传感器)
    if (ui->comboCurrentAxis) {
        ui->comboCurrentAxis->clear();
        for (int i = 0; i < 8; i++) {
            QString axisName;
            if (i == 6) axisName = QString("轴 %1 (伸缩)").arg(i);
            else if (i == 7) axisName = QString("轴 %1 (力传感器)").arg(i);
            else axisName = QString("轴 %1").arg(i);
            ui->comboCurrentAxis->addItem(axisName, i);
        }
        ui->comboCurrentAxis->setCurrentIndex(0);

        // 连接轴选择信号
        connect(ui->comboCurrentAxis, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &MainWindow::on_comboCurrentAxis_currentIndexChanged);
    }

    // 初始化多轴复选框组
    m_axisCheckGroup = new QButtonGroup(this);
    m_axisCheckGroup->setExclusive(false);   // 允许多选

    // 将8个复选框加入组，并关联ID
    QCheckBox* chkAxes[8] = {
        ui->chkAxis0, ui->chkAxis1, ui->chkAxis2,
        ui->chkAxis3, ui->chkAxis4, ui->chkAxis5, ui->chkAxis6, ui->chkAxis7
    };
    for (int i = 0; i < 8; ++i) {
        m_axisCheckGroup->addButton(chkAxes[i], i);
    }

    // 连接复选框状态变化信号（可选，用于联动或调试）
    connect(m_axisCheckGroup, QOverload<int, bool>::of(&QButtonGroup::buttonToggled),
            this, [this](int id, bool checked){
                // 可在此添加额外处理，如更新UI提示
                Q_UNUSED(id); Q_UNUSED(checked);
            });

    // 设置连接
    setupConnections();

    // 加载设置
    loadSettings();

    // 初始化控件状态
    enableControls(false);

    // 显示欢迎消息
    if (ui->txtLog) {
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   "EtherCAT CiA 402 八轴控制系统已启动");
        ui->txtLog->appendPlainText("支持8轴独立控制(6轴旋转+1轴伸缩+1轴力传感器)，使用轴选择下拉框切换当前控制轴");
        ui->txtLog->appendPlainText("注意: 第七轴(伸缩电机)使用单圈编码器，断电后需要重新回零");
    }
}

MainWindow::~MainWindow()
{
    saveSettings();
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (m_commander && m_commander->isConnected()) {
        m_commander->disconnect();
    }
    event->accept();
}

void MainWindow::setupPidTab()
{
    ui->tabWidget->addTab(m_pidConfigTab, "PID参数配置");
    m_pidConfigTab->setCommander(m_commander);
}

void MainWindow::setupConnections()
{
    connect(m_commander, &EtherCATCommander::connectionStateChanged,
            this, &MainWindow::onConnectionStateChanged);
    connect(m_commander, &EtherCATCommander::masterStateChanged,
            this, &MainWindow::onMasterStateChanged);
    connect(m_commander, &EtherCATCommander::slaveStateChanged,
            this, &MainWindow::onSlaveStateChanged);
    connect(m_commander, &EtherCATCommander::errorOccurred,
            this, &MainWindow::onErrorOccurred);
    connect(m_commander, &EtherCATCommander::logMessage,
            this, &MainWindow::onLogMessage);

    // 状态上报信号连接
    connect(m_commander, &EtherCATCommander::statusReportReceived,
            this, [this](const MasterStatusReport &report) {
        // 收到状态上报，确保控制按钮状态正确
        if (report.masterALState == 0x08) {  // OP状态
            enableControls(true);
            // 检查是否有至少一个从站进入OP
            bool anySlaveOp = false;
            for (int i = 0; i < report.slaveCount && i < 8; i++) {
                if (report.slaves[i].statusWord == 0x0237 ||
                    report.slaves[i].statusWord == 0x0A37) {
                    anySlaveOp = true;
                    break;
                }
            }
            if (anySlaveOp) {
                enableMotionControls(true);
            }
        }
    });

    // Tab切换时更新
    connect(ui->tabWidget, &QTabWidget::currentChanged,
            this, &MainWindow::onTabChanged);
}

void MainWindow::onTabChanged(int index)
{
    if (ui->tabWidget->widget(index) == m_pidConfigTab) {
        // 同步当前选中的轴到PID配置Tab
        m_pidConfigTab->setCurrentAxis(m_currentAxis);
    }
}

void MainWindow::loadSettings()
{
    Settings settings;
    ui->txtServerIP->setText(settings.getServerIP());
    ui->spinServerPort->setValue(settings.getServerPort());
}

void MainWindow::saveSettings()
{
    Settings settings;
    settings.setServerIP(ui->txtServerIP->text());
    settings.setServerPort(ui->spinServerPort->value());
}

void MainWindow::enableControls(bool enabled)
{
    qDebug() << "enableControls:" << enabled;

    // 主站生命周期按钮
    ui->btnCreateMaster->setEnabled(enabled);
    ui->btnScanSlaves->setEnabled(enabled);
    ui->btnConfigurePDO->setEnabled(enabled);
    ui->btnActivateMaster->setEnabled(enabled);
    ui->btnDestroyMaster->setEnabled(enabled);
    ui->btnClearErrors->setEnabled(enabled);

    // 6轴整体控制按钮
    ui->btnAllShutdown->setEnabled(enabled);
    ui->btnAllSwitchOn->setEnabled(enabled);
    ui->btnAllEnableOp->setEnabled(enabled);
    ui->btnAllHalt->setEnabled(enabled);
    ui->btnAllFaultReset->setEnabled(enabled);

    // 单轴控制按钮
    ui->btnAxisShutdown->setEnabled(enabled);
    ui->btnAxisSwitchOn->setEnabled(enabled);
    ui->btnAxisEnableOp->setEnabled(enabled);
    ui->btnAxisHalt->setEnabled(enabled);

    // 轴选择下拉框
    ui->comboCurrentAxis->setEnabled(enabled);

    // 紧急控制
    ui->btnEmergencyStop->setEnabled(enabled);

    // 运动控制默认禁用（等待从站OP）
    if (!enabled) {
        enableMotionControls(false);
    }
}

void MainWindow::enableMotionControls(bool enabled)
{
    qDebug() << "enableMotionControls:" << enabled;

    ui->btnJogPositive->setEnabled(enabled);
    ui->btnJogNegative->setEnabled(enabled);
    ui->btnStopMotion->setEnabled(enabled);
    ui->btnMoveRelative->setEnabled(enabled);
    ui->btnMoveAbsolute->setEnabled(enabled);
    ui->spinVelocity->setEnabled(enabled);
    ui->spinPosition->setEnabled(enabled);
}

// ===== 连接控制 =====
void MainWindow::on_btnConnect_clicked()
{
    QString ip = ui->txtServerIP->text();
    quint16 port = ui->spinServerPort->value();

    if (ip.isEmpty()) {
        QMessageBox::warning(this, "警告", "请输入服务器IP地址");
        return;
    }

    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               QString("正在连接到 %1:%2...").arg(ip).arg(port));

    if (m_commander->connectToMaster(ip, port)) {
        ui->statusbar->showMessage(QString("已连接到 %1:%2").arg(ip).arg(port), 3000);
        // 设置轴数为7 (6轴旋转+1轴伸缩电机)
        m_commander->setAxisCount(7);
    } else {
        QMessageBox::critical(this, "错误", "连接失败，请检查网络和端口设置");
    }
}

void MainWindow::on_btnDisconnect_clicked()
{
    m_commander->disconnect();
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "已断开连接");
}

// ===== 主站生命周期 =====
void MainWindow::on_btnCreateMaster_clicked()
{
    m_commander->createMaster();
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送命令: 创建主站 (8轴: 6轴旋转+1轴伸缩+1轴力传感器)");
}

void MainWindow::on_btnScanSlaves_clicked()
{
    m_commander->scanSlaves();
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送命令: 扫描从站");
}

void MainWindow::on_btnConfigurePDO_clicked()
{
    m_commander->configurePDOs();
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送命令: 配置PDO (8轴: 6轴旋转+1轴伸缩+1轴力传感器)");
}

void MainWindow::on_btnActivateMaster_clicked()
{
    m_commander->activateMaster();
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送命令: 激活主站 (配置DC+激活)");
}

void MainWindow::on_btnDestroyMaster_clicked()
{
    m_commander->destroyMaster();
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送命令: 销毁主站");
}

void MainWindow::on_btnClearErrors_clicked()
{
    m_commander->clearErrors();
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送命令: 清除所有轴错误");
}

// ===== 8轴整体控制 =====
void MainWindow::on_btnAllShutdown_clicked()
{
    m_commander->driveShutdown(0xFF);  // 0b11111111 = 8轴
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送CiA402: 全部Shutdown (轴掩码: 0xFF, 8轴)");
}

void MainWindow::on_btnAllSwitchOn_clicked()
{
    m_commander->driveSwitchOn(0xFF);  // 8轴
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送CiA402: 全部Switch On (轴掩码: 0xFF, 8轴)");
}

void MainWindow::on_btnAllEnableOp_clicked()
{
    m_commander->driveEnableOP(0xFF);  // 8轴
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送CiA402: 全部Enable Operation (轴掩码: 0xFF, 8轴)");
}

void MainWindow::on_btnAllHalt_clicked()
{
    m_commander->driveHalt(0xFF);  // 8轴
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送CiA402: 全部Halt (轴掩码: 0xFF, 8轴)");
}

void MainWindow::on_btnAllFaultReset_clicked()
{
    m_commander->faultReset(0xFF);  // 8轴
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "发送CiA402: 全部Fault Reset (轴掩码: 0xFF, 8轴)");
}

// ===== 单轴控制 =====
void MainWindow::on_btnAxisShutdown_clicked()
{
    int axisMask = 1 << m_currentAxis;
    m_commander->driveShutdown(axisMask);
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               QString("发送CiA402: 轴%1 Shutdown (掩码: 0x%2)")
                               .arg(m_currentAxis).arg(axisMask, 2, 16, QChar('0')));
}

void MainWindow::on_btnAxisSwitchOn_clicked()
{
    int axisMask = 1 << m_currentAxis;
    m_commander->driveSwitchOn(axisMask);
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               QString("发送CiA402: 轴%1 Switch On (掩码: 0x%2)")
                               .arg(m_currentAxis).arg(axisMask, 2, 16, QChar('0')));
}

void MainWindow::on_btnAxisEnableOp_clicked()
{
    int axisMask = 1 << m_currentAxis;
    m_commander->driveEnableOP(axisMask);
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               QString("发送CiA402: 轴%1 Enable Operation (掩码: 0x%2)")
                               .arg(m_currentAxis).arg(axisMask, 2, 16, QChar('0')));
}

void MainWindow::on_btnAxisHalt_clicked()
{
    int axisMask = 1 << m_currentAxis;
    m_commander->driveHalt(axisMask);
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               QString("发送CiA402: 轴%1 Halt (掩码: 0x%2)")
                               .arg(m_currentAxis).arg(axisMask, 2, 16, QChar('0')));
}

// ===== 轴选择 =====
void MainWindow::on_comboCurrentAxis_currentIndexChanged(int index)
{
    m_currentAxis = ui->comboCurrentAxis->itemData(index).toInt();
    
    // 第七轴特殊提示
    if (m_currentAxis == 6) {
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   QString("当前控制轴切换为: 轴%1 (伸缩电机)").arg(m_currentAxis));
        // 更新运动控制单位显示
        ui->label_3->setText("速度 (mm/s):");
        ui->label_4->setText("位置 (mm):");
        ui->spinVelocity->setRange(-10.0, 10.0);
        ui->spinVelocity->setSuffix(" mm/s");
        ui->spinPosition->setRange(0.0, 10.0);
        ui->spinPosition->setSuffix(" mm");
    } else {
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   QString("当前控制轴切换为: 轴%1").arg(m_currentAxis));
        // 恢复旋转轴单位
        ui->label_3->setText("速度 (度/秒):");
        ui->label_4->setText("位置 (度):");
        ui->spinVelocity->setRange(-360.0, 360.0);
        ui->spinVelocity->setSuffix(" °/s");
        ui->spinPosition->setRange(-360.0, 360.0);
        ui->spinPosition->setSuffix(" °");
    }

    // 联动：清空所有复选框，只勾选当前轴
    for (int i = 0; i < 7; ++i) {
        QCheckBox* chk = qobject_cast<QCheckBox*>(m_axisCheckGroup->button(i));
        if (chk) {
            chk->setChecked(i == m_currentAxis);
        }
    }

    // 同步更新PID配置Tab和状态面板
    if (m_pidConfigTab) {
        m_pidConfigTab->setCurrentAxis(m_currentAxis);
    }
    if (m_statusPanel) {
        m_statusPanel->setCurrentAxis(m_currentAxis);
    }
}

// ===== 紧急控制 =====
void MainWindow::on_btnEmergencyStop_clicked()
{
    //if (QMessageBox::question(this, "确认", "确认执行紧急停止？\n这将停止所有8个轴！",
    //                         QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
        m_commander->emergencyStop(0xFF);  // 8轴紧急停止
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   "执行紧急停止 - 全部8轴(含伸缩电机和力传感器)");
    //}
}

// ===== 第七轴(伸缩电机)特殊控制 =====
void MainWindow::on_btnAxis7Homing_clicked()
{
    if (!m_commander || !m_commander->isConnected()) {
        ui->txtLog->appendPlainText("错误: 未连接到主站，无法执行回零");
        return;
    }

    float targetHomeMm = 0.0f;  // 默认回到0mm
    float homingSpeedMmPerSec = 2.0f;  // 默认速度2mm/s

    if (m_commander->axis7Homing(targetHomeMm, homingSpeedMmPerSec)) {
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   QString("第七轴(伸缩电机)开始回零: 目标 %.1fmm, 速度 %.1fmm/s")
                                   .arg(targetHomeMm).arg(homingSpeedMmPerSec));
    } else {
        ui->txtLog->appendPlainText("错误: 第七轴回零命令发送失败");
    }
}

void MainWindow::on_btnAxis7ResetPos_clicked()
{
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "第七轴位置重置功能暂未实现");
}

// ===== 运动控制（使用当前选中的轴） =====
void MainWindow::on_btnJogPositive_clicked()
{
    float velocity = ui->spinVelocity->value();
    QVector<int> axes = getSelectedAxes();
    if (axes.isEmpty()) {
        // 若没有选中任何轴，则默认使用当前单轴（安全策略）
        axes.append(m_currentAxis);
    }

    QVector<float> velocities(axes.size(), velocity);
    if (!m_commander->setVelocities(axes, velocities)) {
        ui->txtLog->appendPlainText("发送多轴速度命令失败");
    } else {
        QString axisStr;
        for (int a : axes) axisStr += QString::number(a) + " ";
        ui->txtLog->appendPlainText(QString("多轴正转: 轴[%1] 速度%2").arg(axisStr).arg(velocity));
    }
}

void MainWindow::on_btnJogNegative_clicked()
{
    float velocity = -ui->spinVelocity->value();   // 取负值
    QVector<int> axes = getSelectedAxes();
    if (axes.isEmpty()) {
        axes.append(m_currentAxis);
    }

    QVector<float> velocities(axes.size(), velocity);
    if (!m_commander->setVelocities(axes, velocities)) {
        ui->txtLog->appendPlainText("发送多轴速度命令失败");
    } else {
        QString axisStr;
        for (int a : axes) axisStr += QString::number(a) + " ";
        ui->txtLog->appendPlainText(QString("多轴反转: 轴[%1] 速度%2").arg(axisStr).arg(velocity));
    }
}

void MainWindow::on_btnStopMotion_clicked()
{
    float velocity = 0.0f;
    QVector<int> axes = getSelectedAxes();
    if (axes.isEmpty()) {
        axes.append(m_currentAxis);
    }

    QVector<float> velocities(axes.size(), velocity);
    if (!m_commander->setVelocities(axes, velocities)) {
        ui->txtLog->appendPlainText("发送多轴停止命令失败");
    } else {
        QString axisStr;
        for (int a : axes) axisStr += QString::number(a) + " ";
        ui->txtLog->appendPlainText(QString("多轴停止: 轴[%1]").arg(axisStr));
    }
}

void MainWindow::on_btnMoveRelative_clicked()
{
    float position = ui->spinPosition->value();
    QVector<int> axes = getSelectedAxes();
    if (axes.isEmpty()) {
        axes.append(m_currentAxis);
    }

    QVector<float> positions(axes.size(), position);
    if (!m_commander->moveRelative(axes, positions)) {
        ui->txtLog->appendPlainText("发送多轴相对位置命令失败");
    } else {
        QString axisStr;
        for (int a : axes) axisStr += QString::number(a) + " ";
        ui->txtLog->appendPlainText(QString("多轴相对运动: 轴[%1] 位置%2").arg(axisStr).arg(position));
    }
}

void MainWindow::on_btnMoveAbsolute_clicked()
{
    float position = ui->spinPosition->value();
    QVector<int> axes = getSelectedAxes();
    if (axes.isEmpty()) {
        axes.append(m_currentAxis);
    }

    QVector<float> positions(axes.size(), position);
    if (!m_commander->moveAbsolute(axes, positions)) {
        ui->txtLog->appendPlainText("发送多轴绝对位置命令失败");
    } else {
        QString axisStr;
        for (int a : axes) axisStr += QString::number(a) + " ";
        ui->txtLog->appendPlainText(QString("多轴绝对运动: 轴[%1] 位置%2").arg(axisStr).arg(position));
    }
}

void MainWindow::on_btnClearLog_clicked()
{
    ui->txtLog->clear();
}

// ===== EtherCAT事件处理 =====
void MainWindow::onConnectionStateChanged(bool connected)
{
    ui->btnConnect->setEnabled(!connected);
    ui->btnDisconnect->setEnabled(connected);

    if (connected) {
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   "连接成功 - 可以创建主站");
        // 连接成功后立即启用生命周期按钮
        enableControls(true);
    } else {
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   "连接断开");
        enableControls(false);
        enableMotionControls(false);
    }
}

void MainWindow::onMasterStateChanged(const QString &state)
{
    ui->statusbar->showMessage(state, 3000);
    qDebug() << "主站状态变化:" << state;

    // 只要主站不是错误状态，就启用控制
    if (state.contains("OP") || state.contains("预操作") ||
        state.contains("安全操作") || state.contains("运行") ||
        state.contains("创建")) {
        enableControls(true);
    }

    if (state.contains("错误") || state.contains("销毁") || state.contains("未连接")) {
        enableControls(false);
    }
}

void MainWindow::onSlaveStateChanged(int slaveId, EtherCATCommander::SlaveState state)
{
    qDebug() << "从站" << slaveId << "状态变化:" << state;

    // 从站进入OP时启用运动控制
    if (state == EtherCATCommander::STATE_OP) {
        // 如果当前选中的轴进入OP，启用运动控制
        if (slaveId == m_currentAxis) {
            enableMotionControls(true);
        }
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   QString("轴%1已进入Operation Enabled状态").arg(slaveId));
    } else if (state == EtherCATCommander::STATE_ERROR) {
        ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                                   QString("⚠️ 轴%1发生错误！").arg(slaveId));
    }
}

void MainWindow::onErrorOccurred(const QString &error)
{
    ui->txtLog->appendPlainText(QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") +
                               "错误: " + error);
    QMessageBox::critical(this, "错误", error);
}

void MainWindow::onLogMessage(const QString &message, bool isError)
{
    QString timestamp = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ");
    ui->txtLog->appendPlainText(timestamp + message);

    if (isError) {
        QTextCursor cursor = ui->txtLog->textCursor();
        cursor.movePosition(QTextCursor::End);
        ui->txtLog->setTextCursor(cursor);
    }
}

void MainWindow::updateAxisDisplay()
{
    // 更新UI显示当前轴状态
    ui->statusbar->showMessage(QString("当前轴: %1").arg(m_currentAxis), 2000);
}

int MainWindow::getSelectedAxisMask() const
{
    int mask = 0;
    if (!m_axisCheckGroup) return mask;

    QList<QAbstractButton*> buttons = m_axisCheckGroup->buttons();
    for (QAbstractButton* btn : buttons) {
        if (btn && btn->isChecked()) {
            int id = m_axisCheckGroup->id(btn);
            if (id >= 0) mask |= (1 << id);
        }
    }
    return mask;
}

QVector<int> MainWindow::getSelectedAxes() const
{
    QVector<int> axes;
    if (!m_axisCheckGroup) return axes;  // 安全保护

    QList<QAbstractButton*> buttons = m_axisCheckGroup->buttons();
    for (QAbstractButton* btn : buttons) {
        if (btn && btn->isChecked()) {
            int id = m_axisCheckGroup->id(btn);
            if (id >= 0) axes.append(id);
        }
    }
    return axes;
}

void MainWindow::on_btnMultiAxisControl_clicked()
{
    MultiAxisControlDialog dialog(m_commander, this);
    dialog.exec();  // 模态对话框
}
