/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDial>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_3;
    QFormLayout *formLayout_2;
    QLabel *label_5;
    QComboBox *person_cb;
    QLabel *label;
    QComboBox *mesh_cb;
    QPushButton *add_pb;
    QPushButton *del_pb;
    QGroupBox *move_gb;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QPushButton *up;
    QPushButton *left;
    QPushButton *right;
    QPushButton *down;
    QDial *giro;
    QHBoxLayout *horizontalLayout_7;
    QGroupBox *interacion_gb;
    QVBoxLayout *verticalLayout_6;
    QGridLayout *gridLayout_2;
    QComboBox *int1_cb;
    QComboBox *interaction_cb;
    QPushButton *ainteraction_pb;
    QComboBox *int2_cb;
    QVBoxLayout *verticalLayout_5;
    QListWidget *interaction_lw;
    QHBoxLayout *horizontalLayout_6;
    QSpacerItem *horizontalSpacer;
    QPushButton *rinteraction_pb;
    QHBoxLayout *horizontalLayout_5;
    QGroupBox *pose_gb;
    QVBoxLayout *verticalLayout_3;
    QFormLayout *formLayout;
    QLabel *label_2;
    QSpinBox *x_sb;
    QLabel *label_3;
    QSpinBox *z_sb;
    QLabel *label_4;
    QDoubleSpinBox *rot_sb;
    QPushButton *setPose_pb;
    QPushButton *random;
    QGroupBox *points_gb;
    QVBoxLayout *verticalLayout_4;
    QTextEdit *point_te;
    QCheckBox *autoM_cb;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_6;
    QSpinBox *speed_sb;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *load_pb;
    QPushButton *save_pb;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(686, 591);
        verticalLayout_2 = new QVBoxLayout(guiDlg);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        groupBox_2 = new QGroupBox(guiDlg);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        horizontalLayout_3 = new QHBoxLayout(groupBox_2);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_5);

        person_cb = new QComboBox(groupBox_2);
        person_cb->setObjectName(QString::fromUtf8("person_cb"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, person_cb);

        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label);

        mesh_cb = new QComboBox(groupBox_2);
        mesh_cb->setObjectName(QString::fromUtf8("mesh_cb"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, mesh_cb);

        add_pb = new QPushButton(groupBox_2);
        add_pb->setObjectName(QString::fromUtf8("add_pb"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, add_pb);

        del_pb = new QPushButton(groupBox_2);
        del_pb->setObjectName(QString::fromUtf8("del_pb"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, del_pb);


        horizontalLayout_3->addLayout(formLayout_2);


        horizontalLayout->addWidget(groupBox_2);

        move_gb = new QGroupBox(guiDlg);
        move_gb->setObjectName(QString::fromUtf8("move_gb"));
        verticalLayout = new QVBoxLayout(move_gb);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        up = new QPushButton(move_gb);
        up->setObjectName(QString::fromUtf8("up"));

        gridLayout->addWidget(up, 0, 1, 1, 1);

        left = new QPushButton(move_gb);
        left->setObjectName(QString::fromUtf8("left"));
        left->setCheckable(false);

        gridLayout->addWidget(left, 1, 0, 1, 1);

        right = new QPushButton(move_gb);
        right->setObjectName(QString::fromUtf8("right"));

        gridLayout->addWidget(right, 1, 2, 1, 1);

        down = new QPushButton(move_gb);
        down->setObjectName(QString::fromUtf8("down"));

        gridLayout->addWidget(down, 2, 1, 1, 1);

        giro = new QDial(move_gb);
        giro->setObjectName(QString::fromUtf8("giro"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(giro->sizePolicy().hasHeightForWidth());
        giro->setSizePolicy(sizePolicy);

        gridLayout->addWidget(giro, 1, 1, 1, 1);


        verticalLayout->addLayout(gridLayout);


        horizontalLayout->addWidget(move_gb);

        horizontalLayout->setStretch(0, 1);
        horizontalLayout->setStretch(1, 9);

        verticalLayout_2->addLayout(horizontalLayout);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        interacion_gb = new QGroupBox(guiDlg);
        interacion_gb->setObjectName(QString::fromUtf8("interacion_gb"));
        verticalLayout_6 = new QVBoxLayout(interacion_gb);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        int1_cb = new QComboBox(interacion_gb);
        int1_cb->setObjectName(QString::fromUtf8("int1_cb"));

        gridLayout_2->addWidget(int1_cb, 0, 0, 1, 1);

        interaction_cb = new QComboBox(interacion_gb);
        interaction_cb->setObjectName(QString::fromUtf8("interaction_cb"));

        gridLayout_2->addWidget(interaction_cb, 0, 1, 1, 1);

        ainteraction_pb = new QPushButton(interacion_gb);
        ainteraction_pb->setObjectName(QString::fromUtf8("ainteraction_pb"));

        gridLayout_2->addWidget(ainteraction_pb, 1, 1, 1, 1);

        int2_cb = new QComboBox(interacion_gb);
        int2_cb->setObjectName(QString::fromUtf8("int2_cb"));

        gridLayout_2->addWidget(int2_cb, 0, 2, 1, 1);


        verticalLayout_6->addLayout(gridLayout_2);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        interaction_lw = new QListWidget(interacion_gb);
        interaction_lw->setObjectName(QString::fromUtf8("interaction_lw"));

        verticalLayout_5->addWidget(interaction_lw);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer);

        rinteraction_pb = new QPushButton(interacion_gb);
        rinteraction_pb->setObjectName(QString::fromUtf8("rinteraction_pb"));

        horizontalLayout_6->addWidget(rinteraction_pb);


        verticalLayout_5->addLayout(horizontalLayout_6);


        verticalLayout_6->addLayout(verticalLayout_5);

        verticalLayout_6->setStretch(0, 1);
        verticalLayout_6->setStretch(1, 9);

        horizontalLayout_7->addWidget(interacion_gb);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        pose_gb = new QGroupBox(guiDlg);
        pose_gb->setObjectName(QString::fromUtf8("pose_gb"));
        verticalLayout_3 = new QVBoxLayout(pose_gb);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label_2 = new QLabel(pose_gb);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_2);

        x_sb = new QSpinBox(pose_gb);
        x_sb->setObjectName(QString::fromUtf8("x_sb"));
        x_sb->setMinimum(-10000);
        x_sb->setMaximum(10000);
        x_sb->setSingleStep(100);

        formLayout->setWidget(0, QFormLayout::FieldRole, x_sb);

        label_3 = new QLabel(pose_gb);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_3);

        z_sb = new QSpinBox(pose_gb);
        z_sb->setObjectName(QString::fromUtf8("z_sb"));
        z_sb->setMinimum(-10000);
        z_sb->setMaximum(10000);
        z_sb->setSingleStep(100);

        formLayout->setWidget(1, QFormLayout::FieldRole, z_sb);

        label_4 = new QLabel(pose_gb);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_4);

        rot_sb = new QDoubleSpinBox(pose_gb);
        rot_sb->setObjectName(QString::fromUtf8("rot_sb"));
        rot_sb->setMinimum(-3.14);
        rot_sb->setMaximum(3.14);
        rot_sb->setSingleStep(0.1);

        formLayout->setWidget(2, QFormLayout::FieldRole, rot_sb);

        setPose_pb = new QPushButton(pose_gb);
        setPose_pb->setObjectName(QString::fromUtf8("setPose_pb"));

        formLayout->setWidget(3, QFormLayout::FieldRole, setPose_pb);

        random = new QPushButton(pose_gb);
        random->setObjectName(QString::fromUtf8("random"));

        formLayout->setWidget(4, QFormLayout::FieldRole, random);


        verticalLayout_3->addLayout(formLayout);


        horizontalLayout_5->addWidget(pose_gb);

        points_gb = new QGroupBox(guiDlg);
        points_gb->setObjectName(QString::fromUtf8("points_gb"));
        verticalLayout_4 = new QVBoxLayout(points_gb);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        point_te = new QTextEdit(points_gb);
        point_te->setObjectName(QString::fromUtf8("point_te"));

        verticalLayout_4->addWidget(point_te);

        autoM_cb = new QCheckBox(points_gb);
        autoM_cb->setObjectName(QString::fromUtf8("autoM_cb"));

        verticalLayout_4->addWidget(autoM_cb);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_6 = new QLabel(points_gb);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_4->addWidget(label_6);

        speed_sb = new QSpinBox(points_gb);
        speed_sb->setObjectName(QString::fromUtf8("speed_sb"));
        speed_sb->setMaximum(3000);
        speed_sb->setSingleStep(25);
        speed_sb->setValue(100);

        horizontalLayout_4->addWidget(speed_sb);


        verticalLayout_4->addLayout(horizontalLayout_4);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        load_pb = new QPushButton(points_gb);
        load_pb->setObjectName(QString::fromUtf8("load_pb"));

        horizontalLayout_2->addWidget(load_pb);

        save_pb = new QPushButton(points_gb);
        save_pb->setObjectName(QString::fromUtf8("save_pb"));

        horizontalLayout_2->addWidget(save_pb);


        verticalLayout_4->addLayout(horizontalLayout_2);


        horizontalLayout_5->addWidget(points_gb);


        horizontalLayout_7->addLayout(horizontalLayout_5);


        verticalLayout_2->addLayout(horizontalLayout_7);

        QWidget::setTabOrder(left, up);
        QWidget::setTabOrder(up, down);
        QWidget::setTabOrder(down, right);
        QWidget::setTabOrder(right, giro);

        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "fakeHumanAgent", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("guiDlg", "Manage", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("guiDlg", "Person", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("guiDlg", "Mesh", 0, QApplication::UnicodeUTF8));
        mesh_cb->clear();
        mesh_cb->insertItems(0, QStringList()
         << QApplication::translate("guiDlg", "human01", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("guiDlg", "human03", 0, QApplication::UnicodeUTF8)
        );
        add_pb->setText(QApplication::translate("guiDlg", "Add", 0, QApplication::UnicodeUTF8));
        del_pb->setText(QApplication::translate("guiDlg", "Remove", 0, QApplication::UnicodeUTF8));
        move_gb->setTitle(QApplication::translate("guiDlg", "Move", 0, QApplication::UnicodeUTF8));
        up->setText(QApplication::translate("guiDlg", "UP", 0, QApplication::UnicodeUTF8));
        left->setText(QApplication::translate("guiDlg", "LEFT", 0, QApplication::UnicodeUTF8));
        right->setText(QApplication::translate("guiDlg", "RIGHT", 0, QApplication::UnicodeUTF8));
        down->setText(QApplication::translate("guiDlg", "DOWN", 0, QApplication::UnicodeUTF8));
        interacion_gb->setTitle(QApplication::translate("guiDlg", "Interaction", 0, QApplication::UnicodeUTF8));
        interaction_cb->clear();
        interaction_cb->insertItems(0, QStringList()
         << QApplication::translate("guiDlg", "isBusy", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("guiDlg", "interacting", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("guiDlg", "block", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("guiDlg", "softBlock", 0, QApplication::UnicodeUTF8)
        );
        ainteraction_pb->setText(QApplication::translate("guiDlg", "Add", 0, QApplication::UnicodeUTF8));
        rinteraction_pb->setText(QApplication::translate("guiDlg", "Remove", 0, QApplication::UnicodeUTF8));
        pose_gb->setTitle(QApplication::translate("guiDlg", "Pose", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("guiDlg", "X", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        x_sb->setToolTip(QApplication::translate("guiDlg", "mm", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_3->setText(QApplication::translate("guiDlg", "Z", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        z_sb->setToolTip(QApplication::translate("guiDlg", "mm", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_4->setText(QApplication::translate("guiDlg", "Rot", 0, QApplication::UnicodeUTF8));
        setPose_pb->setText(QApplication::translate("guiDlg", "Set pose", 0, QApplication::UnicodeUTF8));
        random->setText(QApplication::translate("guiDlg", "Random", 0, QApplication::UnicodeUTF8));
        points_gb->setTitle(QApplication::translate("guiDlg", "Points", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        point_te->setToolTip(QApplication::translate("guiDlg", "x,y", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        point_te->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        autoM_cb->setText(QApplication::translate("guiDlg", "Auto movement", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("guiDlg", "Speed", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        speed_sb->setToolTip(QApplication::translate("guiDlg", "mm/s", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        load_pb->setText(QApplication::translate("guiDlg", "Load", 0, QApplication::UnicodeUTF8));
        save_pb->setText(QApplication::translate("guiDlg", "Save", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
