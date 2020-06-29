/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QGroupBox *groupBox;
    QTableWidget *tableWidget;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *play_button;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *nf_button;
    QSpacerItem *horizontalSpacer_4;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *folderLoc;
    QSpacerItem *horizontalSpacer_5;
    QPushButton *browseButton;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QLabel *personCount;
    QFrame *line;
    QFrame *line_2;
    QWidget *layoutWidget3;
    QGridLayout *gridLayout;
    QLabel *label_3;
    QPushButton *browseButton_2;
    QSpacerItem *horizontalSpacer_6;
    QFrame *line_3;
    QWidget *widget;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_4;
    QSpinBox *playPeriod;
    QSpacerItem *horizontalSpacer;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QStringLiteral("guiDlg"));
        guiDlg->resize(898, 472);
        groupBox = new QGroupBox(guiDlg);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(412, 18, 413, 325));
        tableWidget = new QTableWidget(groupBox);
        if (tableWidget->columnCount() < 4)
            tableWidget->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        tableWidget->setObjectName(QStringLiteral("tableWidget"));
        tableWidget->setGeometry(QRect(4, 24, 405, 297));
        tableWidget->setRowCount(0);
        tableWidget->setColumnCount(4);
        layoutWidget = new QWidget(guiDlg);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(114, 384, 657, 50));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        play_button = new QPushButton(layoutWidget);
        play_button->setObjectName(QStringLiteral("play_button"));

        horizontalLayout->addWidget(play_button);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        nf_button = new QPushButton(layoutWidget);
        nf_button->setObjectName(QStringLiteral("nf_button"));

        horizontalLayout->addWidget(nf_button);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_4);


        verticalLayout->addLayout(horizontalLayout);

        layoutWidget1 = new QWidget(guiDlg);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(56, 20, 327, 52));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget1);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_2->addWidget(label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        folderLoc = new QLineEdit(layoutWidget1);
        folderLoc->setObjectName(QStringLiteral("folderLoc"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(folderLoc->sizePolicy().hasHeightForWidth());
        folderLoc->setSizePolicy(sizePolicy);
        folderLoc->setMinimumSize(QSize(200, 0));

        horizontalLayout_2->addWidget(folderLoc, 0, Qt::AlignHCenter);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_5);

        browseButton = new QPushButton(layoutWidget1);
        browseButton->setObjectName(QStringLiteral("browseButton"));

        horizontalLayout_2->addWidget(browseButton);


        verticalLayout_2->addLayout(horizontalLayout_2);

        layoutWidget2 = new QWidget(guiDlg);
        layoutWidget2->setObjectName(QStringLiteral("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(56, 84, 328, 36));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(layoutWidget2);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_3->addWidget(label_2);

        personCount = new QLabel(layoutWidget2);
        personCount->setObjectName(QStringLiteral("personCount"));

        horizontalLayout_3->addWidget(personCount);

        line = new QFrame(guiDlg);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(-3, 128, 397, 17));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(guiDlg);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(395, 2, 5, 337));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);
        layoutWidget3 = new QWidget(guiDlg);
        layoutWidget3->setObjectName(QStringLiteral("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(56, 170, 205, 50));
        gridLayout = new QGridLayout(layoutWidget3);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(layoutWidget3);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 0, 0, 1, 2);

        browseButton_2 = new QPushButton(layoutWidget3);
        browseButton_2->setObjectName(QStringLiteral("browseButton_2"));

        gridLayout->addWidget(browseButton_2, 1, 0, 1, 1);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_6, 1, 1, 1, 1);

        line_3 = new QFrame(guiDlg);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(0, 342, 895, 17));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        widget = new QWidget(guiDlg);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(58, 264, 257, 53));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(widget);
        label_4->setObjectName(QStringLiteral("label_4"));

        verticalLayout_3->addWidget(label_4);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        playPeriod = new QSpinBox(widget);
        playPeriod->setObjectName(QStringLiteral("playPeriod"));
        playPeriod->setMaximum(10000);
        playPeriod->setValue(1000);

        horizontalLayout_4->addWidget(playPeriod);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout_4);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "HumanSceneSim", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("guiDlg", "Current Position", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem = tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("guiDlg", "Time", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem1 = tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("guiDlg", "X", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem2 = tableWidget->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("guiDlg", "Y", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem3 = tableWidget->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("guiDlg", "Rotation", Q_NULLPTR));
        play_button->setText(QApplication::translate("guiDlg", "PLAY", Q_NULLPTR));
        nf_button->setText(QApplication::translate("guiDlg", "Next Frame", Q_NULLPTR));
        label->setText(QApplication::translate("guiDlg", "<html><head/><body><p><span style=\" font-weight:600; color:#004586;\">Load the dataset Folder</span></p></body></html>", Q_NULLPTR));
        browseButton->setText(QApplication::translate("guiDlg", "Browse", Q_NULLPTR));
        label_2->setText(QApplication::translate("guiDlg", "<html><head/><body><p><span style=\" font-weight:600; color:#004586;\">Total Person Detected</span></p></body></html>", Q_NULLPTR));
        personCount->setText(QApplication::translate("guiDlg", "<html><head/><body><p><br/></p></body></html>", Q_NULLPTR));
        label_3->setText(QApplication::translate("guiDlg", "<html><head/><body><p><span style=\" font-weight:600; color:#004586;\">Add a person's CSV</span></p></body></html>", Q_NULLPTR));
        browseButton_2->setText(QApplication::translate("guiDlg", "Browse", Q_NULLPTR));
        label_4->setText(QApplication::translate("guiDlg", "<html><head/><body><p><span style=\" font-weight:600; color:#004586;\">Play timeout (ms)</span></p></body></html>", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
