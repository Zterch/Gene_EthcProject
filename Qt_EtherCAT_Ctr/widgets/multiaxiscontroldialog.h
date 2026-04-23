#ifndef MULTIAXISCONTROLDIALOG_H
#define MULTIAXISCONTROLDIALOG_H

#include <QDialog>
#include <QVector>

QT_BEGIN_NAMESPACE
namespace Ui { class MultiAxisControlDialog; }
QT_END_NAMESPACE

class EtherCATCommander;

class MultiAxisControlDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MultiAxisControlDialog(EtherCATCommander *commander, QWidget *parent = nullptr);
    ~MultiAxisControlDialog();

private slots:
    void on_btnApplyVelocity_clicked();
    void on_btnApplyRelativePos_clicked();
    void on_btnApplyAbsolutePos_clicked();
    void on_btnClose_clicked();

private:
    Ui::MultiAxisControlDialog *ui;
    EtherCATCommander *m_commander;

    // 辅助函数：获取各轴速度值（按轴索引0~5）
    QVector<float> getAxisVelocities() const;
    // 获取各轴位置值
    QVector<float> getAxisPositions() const;
};

#endif // MULTIAXISCONTROLDIALOG_H
