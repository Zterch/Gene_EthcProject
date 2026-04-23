#ifndef PIDCONFIGTAB_H
#define PIDCONFIGTAB_H

#include <QWidget>
#include <QVector>
#include <QPair>

// 前向声明UI类
namespace Ui {
class PidConfigTab;
}

class EtherCATCommander;

struct PidParamDef {
    int paramNo;           // KGU参数号 (如31, 32等)
    QString name;          // 显示名称
    QString description;   // 详细描述
    double minValue;       // 最小值
    double maxValue;       // 最大值
    double defaultValue;   // 默认值
    QString unit;          // 单位
};

class PidConfigTab : public QWidget
{
    Q_OBJECT

public:
    explicit PidConfigTab(QWidget *parent = nullptr);
    ~PidConfigTab();

    void setCommander(EtherCATCommander *commander);
    void setCurrentAxis(int axis);

public slots:
    void onReadParamsClicked();
    void onWriteParamsClicked();
    void onSaveToFlashClicked();
    void onLoadDefaultsClicked();

    void onPidParamsReceived(int axis, const QVector<QPair<int, float>> &params);
    void onWriteResult(int axis, int successCount, int totalCount);
    void onSaveResult(int axis, bool success);
    void onConnectionChanged(bool connected);

private slots:
    void onAxisChanged(int index);

private:
    void setupParamTable();
    void updateParamValues(const QVector<QPair<int, float>> &params);
    QVector<QPair<int, float>> collectParamValues();
    void setUIEnabled(bool enabled);
    void showStatusMessage(const QString &msg, bool isError = false);

    double getParamValue(int row);
    void setParamValue(int row, double value);

    Ui::PidConfigTab *ui;
    EtherCATCommander *m_commander;

    QVector<PidParamDef> m_paramDefs;
    int m_currentAxis;
    bool m_isLoading;

    QTimer *m_readTimeoutTimer;
    QTimer *m_writeTimeoutTimer;   // 新增
    QTimer *m_saveTimeoutTimer;    // 新增
};

#endif // PIDCONFIGTAB_H
