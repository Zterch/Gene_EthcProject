#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QButtonGroup>
#include "ethercatcommander.h"
#include "statuspanel.h"
#include "pidconfigtab.h"
#include "multiaxiscontroldialog.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    // 连接控制
    void on_btnConnect_clicked();
    void on_btnDisconnect_clicked();

    // 主站生命周期
    void on_btnCreateMaster_clicked();
    void on_btnScanSlaves_clicked();
    void on_btnConfigurePDO_clicked();
    void on_btnActivateMaster_clicked();
    void on_btnDestroyMaster_clicked();
    void on_btnClearErrors_clicked();

    // 6轴整体控制
    void on_btnAllShutdown_clicked();
    void on_btnAllSwitchOn_clicked();
    void on_btnAllEnableOp_clicked();
    void on_btnAllHalt_clicked();
    void on_btnAllFaultReset_clicked();

    // 单轴控制（新增）
    void on_btnAxisShutdown_clicked();
    void on_btnAxisSwitchOn_clicked();
    void on_btnAxisEnableOp_clicked();
    void on_btnAxisHalt_clicked();

    // 运动控制（使用当前选中的轴）
    void on_btnJogPositive_clicked();
    void on_btnJogNegative_clicked();
    void on_btnStopMotion_clicked();
    void on_btnMoveRelative_clicked();
    void on_btnMoveAbsolute_clicked();

    // 紧急控制 【添加这行】
    void on_btnEmergencyStop_clicked();

    // 第七轴(伸缩电机)特殊控制
    void on_btnAxis7Homing_clicked();
    void on_btnAxis7ResetPos_clicked();

    // 轴选择（新增）
    void on_comboCurrentAxis_currentIndexChanged(int index);

    // 日志
    void on_btnClearLog_clicked();

    // EtherCAT事件处理
    void onConnectionStateChanged(bool connected);
    void onMasterStateChanged(const QString &state);
    void onSlaveStateChanged(int slaveId, EtherCATCommander::SlaveState state);
    void onErrorOccurred(const QString &error);
    void onLogMessage(const QString &message, bool isError);
    void onTabChanged(int index);

    void on_btnMultiAxisControl_clicked();  // 新增按钮槽

private:
    void setupConnections();
    void setupPidTab();
    void loadSettings();
    void saveSettings();
    void enableControls(bool enabled);
    void enableMotionControls(bool enabled);
    void updateAxisDisplay();

    Ui::MainWindow *ui;
    EtherCATCommander *m_commander;
    StatusPanel *m_statusPanel;
    PidConfigTab *m_pidConfigTab;

    int m_currentAxis = 0;  // 当前选中的轴 (0-5)

    QButtonGroup *m_axisCheckGroup;

    // 新增辅助函数
    int getSelectedAxisMask() const;          // 返回选中轴的掩码（0~0x3F）
    QVector<int> getSelectedAxes() const;     // 返回选中轴的索引列表
};

#endif // MAINWINDOW_H
