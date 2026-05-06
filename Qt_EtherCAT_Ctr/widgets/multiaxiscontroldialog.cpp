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
    setWindowTitle("多轴独立控制 (含第七轴伸缩电机)");

    // 可设置固定大小
    setFixedSize(700, 450);

    // 初始化各轴输入框范围（轴0-5为旋转轴，轴6为伸缩轴）
    QList<QDoubleSpinBox*> velBoxes = {
        ui->spinVel0, ui->spinVel1, ui->spinVel2,
        ui->spinVel3, ui->spinVel4, ui->spinVel5, ui->spinVel6
    };
    for (int i = 0; i < velBoxes.size(); ++i) {
        if (i == 6) {
            // 第七轴(伸缩轴)使用 mm/s
            velBoxes[i]->setRange(-10.0, 10.0);
            velBoxes[i]->setValue(0.0);
            velBoxes[i]->setSuffix(" mm/s");
            velBoxes[i]->setSingleStep(0.1);
        } else {
            // 旋转轴使用 deg/s
            velBoxes[i]->setRange(-360.0, 360.0);
            velBoxes[i]->setValue(0.0);
            velBoxes[i]->setSuffix(" °/s");
        }
    }

    QList<QDoubleSpinBox*> posBoxes = {
        ui->spinPos0, ui->spinPos1, ui->spinPos2,
        ui->spinPos3, ui->spinPos4, ui->spinPos5, ui->spinPos6
    };
    for (int i = 0; i < posBoxes.size(); ++i) {
        if (i == 6) {
            // 第七轴(伸缩轴)使用 mm，范围 0-10mm
            posBoxes[i]->setRange(0.0, 10.0);
            posBoxes[i]->setValue(0.0);
            posBoxes[i]->setSuffix(" mm");
            posBoxes[i]->setSingleStep(0.1);
        } else {
            // 旋转轴使用 deg
            posBoxes[i]->setRange(-360.0, 360.0);
            posBoxes[i]->setValue(0.0);
            posBoxes[i]->setSuffix(" °");
        }
    }
}

MultiAxisControlDialog::~MultiAxisControlDialog()
{
    delete ui;
}

QVector<float> MultiAxisControlDialog::getAxisVelocities() const
{
    QVector<float> v(7, 0.0f);
    v[0] = static_cast<float>(ui->spinVel0->value());
    v[1] = static_cast<float>(ui->spinVel1->value());
    v[2] = static_cast<float>(ui->spinVel2->value());
    v[3] = static_cast<float>(ui->spinVel3->value());
    v[4] = static_cast<float>(ui->spinVel4->value());
    v[5] = static_cast<float>(ui->spinVel5->value());
    v[6] = static_cast<float>(ui->spinVel6->value());
    return v;
}

QVector<float> MultiAxisControlDialog::getAxisPositions() const
{
    QVector<float> p(7, 0.0f);
    p[0] = static_cast<float>(ui->spinPos0->value());
    p[1] = static_cast<float>(ui->spinPos1->value());
    p[2] = static_cast<float>(ui->spinPos2->value());
    p[3] = static_cast<float>(ui->spinPos3->value());
    p[4] = static_cast<float>(ui->spinPos4->value());
    p[5] = static_cast<float>(ui->spinPos5->value());
    p[6] = static_cast<float>(ui->spinPos6->value());
    return p;
}

void MultiAxisControlDialog::on_btnApplyVelocity_clicked()
{
    if (!m_commander) return;

    // 获取所有轴的速度值（轴0~6，含第七轴伸缩电机）
    QVector<float> velocities = getAxisVelocities();
    // 构造轴索引列表 0..6
    QVector<int> axes = {0,1,2,3,4,5,6};

    if (m_commander->setVelocities(axes, velocities)) {
        qDebug() << "多轴速度独立设置成功(含第七轴伸缩电机)";
    } else {
        qDebug() << "多轴速度独立设置失败";
    }
}

void MultiAxisControlDialog::on_btnApplyRelativePos_clicked()
{
    if (!m_commander) return;

    QVector<float> positions = getAxisPositions();
    QVector<int> axes = {0,1,2,3,4,5,6};

    if (m_commander->moveRelative(axes, positions)) {
        qDebug() << "多轴相对位置独立设置成功(含第七轴伸缩电机)";
    } else {
        qDebug() << "多轴相对位置独立设置失败";
    }
}

void MultiAxisControlDialog::on_btnApplyAbsolutePos_clicked()
{
    if (!m_commander) return;

    QVector<float> positions = getAxisPositions();
    QVector<int> axes = {0,1,2,3,4,5,6};

    if (m_commander->moveAbsolute(axes, positions)) {
        qDebug() << "多轴绝对位置独立设置成功(含第七轴伸缩电机)";
    } else {
        qDebug() << "多轴绝对位置独立设置失败";
    }
}

void MultiAxisControlDialog::on_btnAxis7Homing_clicked()
{
    if (!m_commander) return;

    // 第七轴回零: 回到0mm位置，速度2mm/s
    if (m_commander->axis7Homing(0.0f, 2.0f)) {
        qDebug() << "第七轴回零命令已发送";
    } else {
        qDebug() << "第七轴回零命令发送失败";
    }
}

void MultiAxisControlDialog::on_btnClose_clicked()
{
    accept();  // 或 reject()
}
