#ifndef STATUSPANEL_H
#define STATUSPANEL_H

#include <QWidget>
#include <QTimer>
#include <QTableWidget>
#include "ethercatcommander.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class StatusPanel;
}
QT_END_NAMESPACE

class StatusPanel : public QWidget
{
    Q_OBJECT

public:
    explicit StatusPanel(QWidget *parent = nullptr);
    ~StatusPanel();

    void setCommander(EtherCATCommander *commander);
    void setCurrentAxis(int axis);  // 设置当前显示的轴

public slots:
    void onConnectionStateChanged(bool connected);
    void onMasterStateChanged(const QString &state);
    void onSlaveStateChanged(int slaveId, EtherCATCommander::SlaveState state);
    void onPositionUpdated(int axis, float position);
    void onVelocityUpdated(int axis, float velocity);
    void onTorqueUpdated(int axis, float torque);
    void onPdoDataUpdated(int axis, const SlavePdoData &pdoData);
    void onStatusReportReceived(const MasterStatusReport &report);

private slots:
    void onUpdateTimer();
    void onTableCellClicked(int row, int column);

private:
    void setupUI();
    void initAxesTable();
    void updateAxisRow(int axis, const SlavePdoData &pdoData);
    void updateCurrentAxisDisplay(int axis);
    void updateForceSensorDisplay();  // 更新六维力传感器数据显示
    QString getStateColor(EtherCATCommander::SlaveState state);
    QString getCiA402StateString(uint16_t statusWord);
    QString getStateString(EtherCATCommander::SlaveState state);

    Ui::StatusPanel *ui;
    EtherCATCommander *m_commander;
    QTimer *m_updateTimer;

    static const int MAX_AXES = 8;  // 支持8轴（含第七轴伸缩电机和第八轴六维力传感器）
    int m_currentAxis;  // 当前选中的轴
    bool m_hasPdoData[MAX_AXES];
    SlavePdoData m_lastPdoData[MAX_AXES];
    EtherCATCommander::SlaveState m_slaveStates[MAX_AXES];

    // 运行时间计数
    int m_runTimeCounter;
};

#endif // STATUSPANEL_H
