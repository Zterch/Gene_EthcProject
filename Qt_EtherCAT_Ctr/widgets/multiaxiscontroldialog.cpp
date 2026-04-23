#include "multiaxiscontroldialog.h"
#include "ui_multiaxiscontroldialog.h"
#include "ethercatcommander.h"
#include <QDebug>

MultiAxisControlDialog::MultiAxisControlDialog(EtherCATCommander *commander, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MultiAxisControlDialog),
    m_commander(commander)
{
    ui->setupUi(this);
    setWindowTitle("多轴独立控制");

    // 可设置固定大小
    setFixedSize(600, 400);

    // 初始化各轴输入框范围（可根据需要调整）
    QList<QDoubleSpinBox*> velBoxes = {
        ui->spinVel0, ui->spinVel1, ui->spinVel2,
        ui->spinVel3, ui->spinVel4, ui->spinVel5
    };
    for (auto *box : velBoxes) {
        box->setRange(-360.0, 360.0);
        box->setValue(0.0);
        box->setSuffix(" °/s");
    }

    QList<QDoubleSpinBox*> posBoxes = {
        ui->spinPos0, ui->spinPos1, ui->spinPos2,
        ui->spinPos3, ui->spinPos4, ui->spinPos5
    };
    for (auto *box : posBoxes) {
        box->setRange(-360.0, 360.0);
        box->setValue(0.0);
        box->setSuffix(" °");
    }
}

MultiAxisControlDialog::~MultiAxisControlDialog()
{
    delete ui;
}

QVector<float> MultiAxisControlDialog::getAxisVelocities() const
{
    QVector<float> v(6, 0.0f);
    v[0] = static_cast<float>(ui->spinVel0->value());
    v[1] = static_cast<float>(ui->spinVel1->value());
    v[2] = static_cast<float>(ui->spinVel2->value());
    v[3] = static_cast<float>(ui->spinVel3->value());
    v[4] = static_cast<float>(ui->spinVel4->value());
    v[5] = static_cast<float>(ui->spinVel5->value());
    return v;
}

QVector<float> MultiAxisControlDialog::getAxisPositions() const
{
    QVector<float> p(6, 0.0f);
    p[0] = static_cast<float>(ui->spinPos0->value());
    p[1] = static_cast<float>(ui->spinPos1->value());
    p[2] = static_cast<float>(ui->spinPos2->value());
    p[3] = static_cast<float>(ui->spinPos3->value());
    p[4] = static_cast<float>(ui->spinPos4->value());
    p[5] = static_cast<float>(ui->spinPos5->value());
    return p;
}

void MultiAxisControlDialog::on_btnApplyVelocity_clicked()
{
    if (!m_commander) return;

    // 获取所有轴的速度值（轴0~5）
    QVector<float> velocities = getAxisVelocities();
    // 构造轴索引列表 0..5
    QVector<int> axes = {0,1,2,3,4,5};

    if (m_commander->setVelocities(axes, velocities)) {
        qDebug() << "多轴速度独立设置成功";
    } else {
        qDebug() << "多轴速度独立设置失败";
    }
}

void MultiAxisControlDialog::on_btnApplyRelativePos_clicked()
{
    if (!m_commander) return;

    QVector<float> positions = getAxisPositions();
    QVector<int> axes = {0,1,2,3,4,5};

    if (m_commander->moveRelative(axes, positions)) {
        qDebug() << "多轴相对位置独立设置成功";
    } else {
        qDebug() << "多轴相对位置独立设置失败";
    }
}

void MultiAxisControlDialog::on_btnApplyAbsolutePos_clicked()
{
    if (!m_commander) return;

    QVector<float> positions = getAxisPositions();
    QVector<int> axes = {0,1,2,3,4,5};

    if (m_commander->moveAbsolute(axes, positions)) {
        qDebug() << "多轴绝对位置独立设置成功";
    } else {
        qDebug() << "多轴绝对位置独立设置失败";
    }
}

void MultiAxisControlDialog::on_btnClose_clicked()
{
    accept();  // 或 reject()
}
