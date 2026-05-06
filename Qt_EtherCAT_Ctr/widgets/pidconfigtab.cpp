#include "pidconfigtab.h"
#include "ui_pidconfigtab.h"
#include "ethercatcommander.h"
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QDebug>

PidConfigTab::PidConfigTab(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PidConfigTab)
    , m_commander(nullptr)
    , m_currentAxis(0)
    , m_isLoading(false)
    , m_readTimeoutTimer(nullptr)    // 初始化
    , m_writeTimeoutTimer(nullptr)   // 新增初始化
    , m_saveTimeoutTimer(nullptr)    // 新增初始化
{
    ui->setupUi(this);

    // 初始化参数定义 (与主站保持一致)
    m_paramDefs = {
        {31, "Cur_Kp", "电流环比例增益", 0, 9999, 2000, ""},
        {32, "Cur_Ki", "电流环积分增益", 0, 9999, 300, ""},
        {33, "Vel_Kp", "速度环比例增益", 0, 9999, 1800, ""},
        {34, "Vel_Ki", "速度环积分增益", 0, 9999, 300, ""},
        {65, "Pos_Kp1", "位置环比例增益1(粗调)", 0, 24, 22, ""},
        {63, "Pos_Kp2", "位置环比例增益2(细调)", 1000, 5000, 2000, ""},
        {64, "Pos_FF", "位置前馈百分比", 0, 100, 0, "%"}
    };

    // 初始化轴选择 - 7轴(含第七轴伸缩电机)
    ui->comboAxis->clear();
    for (int i = 0; i < 7; i++) {
        QString axisName = (i == 6) ? QString("轴 %1 (伸缩)").arg(i) : QString("轴 %1").arg(i);
        ui->comboAxis->addItem(axisName, i);
    }

    setupParamTable();
    setUIEnabled(false);

    // 连接信号槽
    connect(ui->comboAxis, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PidConfigTab::onAxisChanged);
    connect(ui->btnRead, &QPushButton::clicked, this, &PidConfigTab::onReadParamsClicked);
    connect(ui->btnWrite, &QPushButton::clicked, this, &PidConfigTab::onWriteParamsClicked);
    connect(ui->btnSaveFlash, &QPushButton::clicked, this, &PidConfigTab::onSaveToFlashClicked);
    connect(ui->btnLoadDefaults, &QPushButton::clicked, this, &PidConfigTab::onLoadDefaultsClicked);
}

PidConfigTab::~PidConfigTab()
{
    // 清理定时器
    if (m_readTimeoutTimer) {
        m_readTimeoutTimer->stop();
        delete m_readTimeoutTimer;
    }
    if (m_writeTimeoutTimer) {
        m_writeTimeoutTimer->stop();
        delete m_writeTimeoutTimer;
    }
    if (m_saveTimeoutTimer) {
        m_saveTimeoutTimer->stop();
        delete m_saveTimeoutTimer;
    }
    delete ui;
}

void PidConfigTab::setupParamTable()
{
    ui->tableParams->setRowCount(m_paramDefs.size());

    for (int i = 0; i < m_paramDefs.size(); i++) {
        const PidParamDef &def = m_paramDefs[i];

        // 参数号 - 使用深色背景，白色字体
        QTableWidgetItem *itemNo = new QTableWidgetItem(
            QString("Pn%1\n(%2)").arg(def.paramNo, 3, 10, QChar('0')).arg(def.name));
        itemNo->setFlags(itemNo->flags() & ~Qt::ItemIsEditable);
        itemNo->setTextAlignment(Qt::AlignCenter);
        // 修改：使用深蓝色背景，白色字体
        itemNo->setBackground(QColor(0, 51, 102)); // 深蓝
        itemNo->setForeground(QColor(255, 255, 255)); // 白色
        QFont font = itemNo->font();
        font.setBold(true);
        itemNo->setFont(font);
        ui->tableParams->setItem(i, 0, itemNo);

        // 参数名称
        QTableWidgetItem *itemName = new QTableWidgetItem(def.name);
        itemName->setFlags(itemName->flags() & ~Qt::ItemIsEditable);
        itemName->setTextAlignment(Qt::AlignCenter);
        itemName->setBackground(QColor(0, 51, 102)); // 淡紫
        ui->tableParams->setItem(i, 1, itemName);

        // 描述
        QTableWidgetItem *itemDesc = new QTableWidgetItem(def.description);
        itemDesc->setFlags(itemDesc->flags() & ~Qt::ItemIsEditable);
        ui->tableParams->setItem(i, 2, itemDesc);

        // 当前值 - 使用SpinBox
        QDoubleSpinBox *spin = new QDoubleSpinBox;
        spin->setRange(def.minValue, def.maxValue);
        spin->setValue(def.defaultValue);
        spin->setDecimals(0);
        spin->setAlignment(Qt::AlignCenter);
        spin->setProperty("row", i);
        spin->setProperty("paramNo", def.paramNo);
        // 设置SpinBox样式
        //spin->setStyleSheet("background-color: #2b2b2b; color: white; border: 1px solid white;");
        ui->tableParams->setCellWidget(i, 3, spin);

        // 范围
        QString rangeStr = QString("%1 ~ %2").arg(def.minValue).arg(def.maxValue);
        QTableWidgetItem *itemRange = new QTableWidgetItem(rangeStr);
        itemRange->setFlags(itemRange->flags() & ~Qt::ItemIsEditable);
        itemRange->setForeground(QColor(80, 80, 80));
        itemRange->setTextAlignment(Qt::AlignCenter);
        itemRange->setBackground(QColor(245, 245, 245)); // 浅灰
        ui->tableParams->setItem(i, 4, itemRange);

        // 单位
        QTableWidgetItem *itemUnit = new QTableWidgetItem(def.unit);
        itemUnit->setFlags(itemUnit->flags() & ~Qt::ItemIsEditable);
        itemUnit->setTextAlignment(Qt::AlignCenter);
        ui->tableParams->setItem(i, 5, itemUnit);
    }

    // 设置行高
    for (int i = 0; i < ui->tableParams->rowCount(); i++) {
        ui->tableParams->setRowHeight(i, 40);
    }

    ui->tableParams->resizeColumnsToContents();
    ui->tableParams->horizontalHeader()->setStretchLastSection(false);
    ui->tableParams->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    ui->tableParams->verticalHeader()->setVisible(false);
    ui->tableParams->setAlternatingRowColors(true);
    ui->tableParams->setStyleSheet("QTableWidget { gridline-color: #cccccc; }");
}

void PidConfigTab::setCommander(EtherCATCommander *commander)
{
    if (m_commander) {
        disconnect(m_commander, nullptr, this, nullptr);
    }

    m_commander = commander;

    if (m_commander) {
        // 【确认】这些信号槽连接是否正确
        bool ok1 = connect(m_commander, &EtherCATCommander::pidParamsReceived,
                this, &PidConfigTab::onPidParamsReceived);
        bool ok2 = connect(m_commander, &EtherCATCommander::pidWriteResult,
                this, &PidConfigTab::onWriteResult);
        bool ok3 = connect(m_commander, &EtherCATCommander::pidSaveResult,
                this, &PidConfigTab::onSaveResult);
        bool ok4 = connect(m_commander, &EtherCATCommander::connectionStateChanged,
                this, &PidConfigTab::onConnectionChanged);

        qDebug() << "信号槽连接状态: pidParamsReceived=" << ok1
                 << "pidWriteResult=" << ok2
                 << "pidSaveResult=" << ok3
                 << "connectionStateChanged=" << ok4;

        setUIEnabled(m_commander->isConnected());
    }
}

void PidConfigTab::setCurrentAxis(int axis)
{
    int index = ui->comboAxis->findData(axis);
    if (index >= 0) {
        ui->comboAxis->setCurrentIndex(index);
    }
}

void PidConfigTab::onAxisChanged(int index)
{
    m_currentAxis = ui->comboAxis->itemData(index).toInt();
    showStatusMessage(QString("已切换到轴%1，请读取参数").arg(m_currentAxis));
}

void PidConfigTab::onReadParamsClicked()
{
    if (!m_commander) return;

    ui->btnRead->setEnabled(false);
    ui->btnRead->setText("读取中...");
    showStatusMessage(QString("正在读取轴%1的PID参数...").arg(m_currentAxis));

    // 启动超时定时器
    QTimer::singleShot(5000, this, [this]() {
        if (ui->btnRead->text() == "读取中...") {
            ui->btnRead->setEnabled(true);
            ui->btnRead->setText("从驱动器读取");
            showStatusMessage("读取超时，请检查连接", true);
        }
    });

    if (!m_commander->readPidParams(m_currentAxis)) {
        ui->btnRead->setEnabled(true);
        ui->btnRead->setText("从驱动器读取");
        showStatusMessage("发送读取命令失败", true);
    }
}

void PidConfigTab::onWriteParamsClicked()
{
    if (!m_commander) return;

    auto params = collectParamValues();
    if (params.isEmpty()) {
        QMessageBox::warning(this, "警告", "没有可写入的参数");
        return;
    }

    if (QMessageBox::question(this, "确认写入",
        QString("确定要将%1个参数写入轴%2的驱动器RAM吗？\n"
               "注意: 写入RAM的参数断电后会丢失，如需永久保存请点击'保存到Flash'。")
        .arg(params.size()).arg(m_currentAxis),
        QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    ui->btnWrite->setEnabled(false);
    ui->btnWrite->setText("写入中...");

    // 创建超时定时器
    if (!m_writeTimeoutTimer) {
        m_writeTimeoutTimer = new QTimer(this);
        m_writeTimeoutTimer->setSingleShot(true);
    }

    // 断开之前的连接（如果有）
    disconnect(m_writeTimeoutTimer, &QTimer::timeout, nullptr, nullptr);

    // 连接超时槽
    connect(m_writeTimeoutTimer, &QTimer::timeout, this, [this]() {
        if (ui->btnWrite->text() == "写入中...") {
            ui->btnWrite->setEnabled(true);
            ui->btnWrite->setText("写入驱动器");
            showStatusMessage("写入超时，请检查连接", true);
            QMessageBox::warning(this, "超时", "PID参数写入超时");
        }
    });

    m_writeTimeoutTimer->start(5000);  // 5秒超时

    if (!m_commander->writePidParams(m_currentAxis, params)) {
        ui->btnWrite->setEnabled(true);
        ui->btnWrite->setText("写入驱动器");
        showStatusMessage("发送写入命令失败", true);

        if (m_writeTimeoutTimer) {
            m_writeTimeoutTimer->stop();
        }
    }
}

void PidConfigTab::onSaveToFlashClicked()
{
    if (!m_commander) return;

    if (QMessageBox::warning(this, "确认保存",
        "保存到Flash会覆盖驱动器内部存储的参数，且频繁写入可能影响Flash寿命。\n"
        "确定要继续吗？",
        QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    ui->btnSaveFlash->setEnabled(false);
    ui->btnSaveFlash->setText("保存中...");

    // 创建超时定时器
    if (!m_saveTimeoutTimer) {
        m_saveTimeoutTimer = new QTimer(this);
        m_saveTimeoutTimer->setSingleShot(true);
    }

    // 断开之前的连接
    disconnect(m_saveTimeoutTimer, &QTimer::timeout, nullptr, nullptr);

    // 连接超时槽
    connect(m_saveTimeoutTimer, &QTimer::timeout, this, [this]() {
        if (ui->btnSaveFlash->text() == "保存中...") {
            ui->btnSaveFlash->setEnabled(true);
            ui->btnSaveFlash->setText("保存到Flash");
            showStatusMessage("保存超时，请检查连接", true);
            QMessageBox::warning(this, "超时", "保存到Flash超时");
        }
    });

    m_saveTimeoutTimer->start(5000);  // 5秒超时

    if (!m_commander->savePidParamsToFlash(m_currentAxis)) {
        ui->btnSaveFlash->setEnabled(true);
        ui->btnSaveFlash->setText("保存到Flash");
        showStatusMessage("发送保存命令失败", true);

        if (m_saveTimeoutTimer) {
            m_saveTimeoutTimer->stop();
        }
    }
}

void PidConfigTab::onLoadDefaultsClicked()
{
    if (QMessageBox::question(this, "确认", "确定要恢复所有参数为默认值吗？",
                             QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
        return;
    }

    for (int i = 0; i < m_paramDefs.size(); i++) {
        setParamValue(i, m_paramDefs[i].defaultValue);
    }

    showStatusMessage("已恢复默认值，请点击'写入驱动器'生效");
}

void PidConfigTab::onPidParamsReceived(int axis, const QVector<QPair<int, float>> &params)
{
    qDebug() << "onPidParamsReceived被调用: axis=" << axis
             << "params.size=" << params.size();

    // 【注意】不要忽略axis，用于验证是否匹配当前选中的轴
    int currentAxis = ui->comboAxis->currentData().toInt();
    if (axis != currentAxis) {
        qDebug() << "警告: 接收到的轴" << axis << "与当前选中轴" << currentAxis << "不匹配";
        // 仍然更新，但记录警告
    }

    ui->btnRead->setEnabled(true);
    ui->btnRead->setText("从驱动器读取");

    if (params.isEmpty()) {
        showStatusMessage("读取失败或未获取到有效参数", true);
        return;
    }

    // 【调试】打印每个参数
    for (const auto& pair : params) {
        qDebug() << "  更新UI: Pn" << pair.first << "=" << pair.second;
    }

    updateParamValues(params);
    showStatusMessage(QString("成功读取%1个参数").arg(params.size()));
}

void PidConfigTab::onWriteResult(int axis, int successCount, int totalCount)
{
    qDebug() << "PID写入结果信号: axis=" << axis
             << "success=" << successCount << "total=" << totalCount;

    ui->btnWrite->setEnabled(true);
    ui->btnWrite->setText("写入驱动器");

    // 停止超时定时器
    if (m_writeTimeoutTimer) {
        m_writeTimeoutTimer->stop();
    }

    if (successCount == totalCount && totalCount > 0) {
        showStatusMessage(QString("✓ 成功写入轴%1的全部%2个参数").arg(axis).arg(totalCount));
        // 不弹窗，只显示状态
    } else if (successCount > 0) {
        showStatusMessage(QString("⚠️ 部分成功: %1/%2").arg(successCount).arg(totalCount), true);
        QMessageBox::warning(this, "部分失败",
            QString("仅成功写入 %1/%2 个参数").arg(successCount).arg(totalCount));
    } else {
        showStatusMessage("❌ 写入失败", true);
        QMessageBox::critical(this, "失败", "PID参数写入失败，请检查连接");
    }
}

void PidConfigTab::onSaveResult(int axis, bool success)
{
    qDebug() << "PID保存结果信号: axis=" << axis << "success=" << success;

    ui->btnSaveFlash->setEnabled(true);
    ui->btnSaveFlash->setText("保存到Flash");

    // 停止超时定时器
    if (m_saveTimeoutTimer) {
        m_saveTimeoutTimer->stop();
    }

    if (success) {
        showStatusMessage(QString("✓ 轴%1参数已永久保存").arg(axis));
        // 不弹窗，避免重复
    } else {
        showStatusMessage(QString("❌ 轴%1保存失败").arg(axis), true);
        QMessageBox::critical(this, "失败", "保存到Flash失败，请检查连接和驱动器状态");
    }
}

void PidConfigTab::onConnectionChanged(bool connected)
{
    setUIEnabled(connected);
    if (!connected) {
        showStatusMessage("连接已断开");
    } else {
        showStatusMessage("已连接，请选择轴并读取参数");
    }
}

void PidConfigTab::updateParamValues(const QVector<QPair<int, float>> &params)
{
    m_isLoading = true;

    for (const auto &pair : params) {
        int paramNo = pair.first;
        float value = pair.second;

        // 查找对应的行
        for (int i = 0; i < m_paramDefs.size(); i++) {
            if (m_paramDefs[i].paramNo == paramNo) {
                setParamValue(i, value);
                break;
            }
        }
    }

    m_isLoading = false;
}

QVector<QPair<int, float>> PidConfigTab::collectParamValues()
{
    QVector<QPair<int, float>> params;

    for (int i = 0; i < m_paramDefs.size(); i++) {
        double value = getParamValue(i);
        params.append(qMakePair(m_paramDefs[i].paramNo, static_cast<float>(value)));
    }

    return params;
}

double PidConfigTab::getParamValue(int row)
{
    QWidget *widget = ui->tableParams->cellWidget(row, 3);
    QDoubleSpinBox *spin = qobject_cast<QDoubleSpinBox*>(widget);
    if (spin) {
        return spin->value();
    }
    return 0;
}

void PidConfigTab::setParamValue(int row, double value)
{
    QWidget *widget = ui->tableParams->cellWidget(row, 3);
    QDoubleSpinBox *spin = qobject_cast<QDoubleSpinBox*>(widget);
    if (spin) {
        spin->setValue(value);
    }
}

void PidConfigTab::setUIEnabled(bool enabled)
{
    ui->comboAxis->setEnabled(enabled);
    ui->btnRead->setEnabled(enabled);
    ui->btnWrite->setEnabled(enabled);
    ui->btnSaveFlash->setEnabled(enabled);
    ui->btnLoadDefaults->setEnabled(enabled);

    // 表格中的spinbox
    for (int i = 0; i < ui->tableParams->rowCount(); i++) {
        QWidget *widget = ui->tableParams->cellWidget(i, 3);
        if (widget) {
            widget->setEnabled(enabled);
        }
    }
}

void PidConfigTab::showStatusMessage(const QString &msg, bool isError)
{
    ui->lblStatus->setText("状态: " + msg);
    if (isError) {
        ui->lblStatus->setStyleSheet("color: red; padding: 5px; border-top: 1px solid #cccccc; font-weight: bold;");
    } else {
        ui->lblStatus->setStyleSheet("color: green; padding: 5px; border-top: 1px solid #cccccc;");
    }
}
