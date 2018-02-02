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
#include <QtGui/QFrame>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout_3;
    QGraphicsView *graphic;
    QFrame *frame_2;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *reloadButton;
    QPushButton *findObjectButton;
    QTextEdit *text_object;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_9;
    QVBoxLayout *verticalLayout_7;
    QGridLayout *gridLayout;
    QLineEdit *z_object;
    QLabel *label_15;
    QLabel *label_3;
    QLineEdit *x_object;
    QLineEdit *y_object;
    QLineEdit *rx_object;
    QLabel *label;
    QLineEdit *ry_object;
    QLabel *label_4;
    QLabel *label_2;
    QLineEdit *rz_object;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_5;
    QCheckBox *InitPose;
    QCheckBox *regularPose;
    QCheckBox *ObtainPose;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_8;
    QLineEdit *label_le;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_14;
    QSpinBox *ob_to_save;
    QVBoxLayout *verticalLayout_6;
    QPushButton *saveViewButton;
    QPushButton *goButton;
    QLabel *isObject;
    QPushButton *ResetPoseButton;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_17;
    QSlider *tx_Slider;
    QHBoxLayout *horizontalLayout_13;
    QLabel *label_18;
    QSlider *ty_Slider;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_19;
    QSlider *tz_Slider;
    QHBoxLayout *horizontalLayout;
    QLabel *label_5;
    QLabel *stateLabel;
    QSpacerItem *horizontalSpacer;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(658, 759);
        verticalLayout_3 = new QVBoxLayout(guiDlg);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        graphic = new QGraphicsView(guiDlg);
        graphic->setObjectName(QString::fromUtf8("graphic"));
        graphic->setMinimumSize(QSize(640, 480));
        graphic->setMaximumSize(QSize(640, 480));

        verticalLayout_3->addWidget(graphic);

        frame_2 = new QFrame(guiDlg);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setMaximumSize(QSize(16777215, 255));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        horizontalLayout_2 = new QHBoxLayout(frame_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        reloadButton = new QPushButton(frame_2);
        reloadButton->setObjectName(QString::fromUtf8("reloadButton"));
        reloadButton->setMinimumSize(QSize(50, 27));
        reloadButton->setMaximumSize(QSize(180, 27));

        verticalLayout->addWidget(reloadButton);

        findObjectButton = new QPushButton(frame_2);
        findObjectButton->setObjectName(QString::fromUtf8("findObjectButton"));
        findObjectButton->setMinimumSize(QSize(99, 27));
        findObjectButton->setMaximumSize(QSize(180, 27));

        verticalLayout->addWidget(findObjectButton);

        text_object = new QTextEdit(frame_2);
        text_object->setObjectName(QString::fromUtf8("text_object"));
        text_object->setMinimumSize(QSize(99, 30));
        text_object->setMaximumSize(QSize(180, 30));

        verticalLayout->addWidget(text_object);


        horizontalLayout_2->addLayout(verticalLayout);

        groupBox = new QGroupBox(frame_2);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy);
        verticalLayout_9 = new QVBoxLayout(groupBox);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        z_object = new QLineEdit(groupBox);
        z_object->setObjectName(QString::fromUtf8("z_object"));
        z_object->setMaximumSize(QSize(100, 27));

        gridLayout->addWidget(z_object, 1, 3, 1, 1);

        label_15 = new QLabel(groupBox);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setMaximumSize(QSize(103, 27));

        gridLayout->addWidget(label_15, 1, 0, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMaximumSize(QSize(100, 17));

        gridLayout->addWidget(label_3, 0, 2, 1, 1);

        x_object = new QLineEdit(groupBox);
        x_object->setObjectName(QString::fromUtf8("x_object"));
        x_object->setMaximumSize(QSize(100, 27));

        gridLayout->addWidget(x_object, 1, 1, 1, 1);

        y_object = new QLineEdit(groupBox);
        y_object->setObjectName(QString::fromUtf8("y_object"));
        y_object->setMaximumSize(QSize(100, 27));

        gridLayout->addWidget(y_object, 1, 2, 1, 1);

        rx_object = new QLineEdit(groupBox);
        rx_object->setObjectName(QString::fromUtf8("rx_object"));
        rx_object->setMaximumSize(QSize(100, 27));

        gridLayout->addWidget(rx_object, 3, 1, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMaximumSize(QSize(103, 27));

        gridLayout->addWidget(label, 3, 0, 1, 1);

        ry_object = new QLineEdit(groupBox);
        ry_object->setObjectName(QString::fromUtf8("ry_object"));
        ry_object->setMaximumSize(QSize(100, 27));

        gridLayout->addWidget(ry_object, 3, 2, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMaximumSize(QSize(100, 17));

        gridLayout->addWidget(label_4, 0, 3, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMaximumSize(QSize(100, 17));

        gridLayout->addWidget(label_2, 0, 1, 1, 1);

        rz_object = new QLineEdit(groupBox);
        rz_object->setObjectName(QString::fromUtf8("rz_object"));
        rz_object->setMaximumSize(QSize(100, 27));

        gridLayout->addWidget(rz_object, 3, 3, 1, 1);


        verticalLayout_7->addLayout(gridLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        InitPose = new QCheckBox(groupBox);
        InitPose->setObjectName(QString::fromUtf8("InitPose"));
        InitPose->setEnabled(true);
        InitPose->setMinimumSize(QSize(150, 0));
        InitPose->setMaximumSize(QSize(156, 22));

        verticalLayout_5->addWidget(InitPose);

        regularPose = new QCheckBox(groupBox);
        regularPose->setObjectName(QString::fromUtf8("regularPose"));
        regularPose->setEnabled(true);
        regularPose->setMinimumSize(QSize(150, 16));
        regularPose->setMaximumSize(QSize(156, 22));

        verticalLayout_5->addWidget(regularPose);

        ObtainPose = new QCheckBox(groupBox);
        ObtainPose->setObjectName(QString::fromUtf8("ObtainPose"));
        ObtainPose->setEnabled(false);
        ObtainPose->setMinimumSize(QSize(150, 0));
        ObtainPose->setMaximumSize(QSize(156, 22));

        verticalLayout_5->addWidget(ObtainPose);


        horizontalLayout_3->addLayout(verticalLayout_5);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setMaximumSize(QSize(100, 30));

        verticalLayout_2->addWidget(label_8);

        label_le = new QLineEdit(groupBox);
        label_le->setObjectName(QString::fromUtf8("label_le"));
        label_le->setMaximumSize(QSize(500, 27));

        verticalLayout_2->addWidget(label_le);


        horizontalLayout_3->addLayout(verticalLayout_2);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_14 = new QLabel(groupBox);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setMaximumSize(QSize(99, 30));

        verticalLayout_4->addWidget(label_14);

        ob_to_save = new QSpinBox(groupBox);
        ob_to_save->setObjectName(QString::fromUtf8("ob_to_save"));
        ob_to_save->setMaximumSize(QSize(99, 27));

        verticalLayout_4->addWidget(ob_to_save);


        horizontalLayout_3->addLayout(verticalLayout_4);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        saveViewButton = new QPushButton(groupBox);
        saveViewButton->setObjectName(QString::fromUtf8("saveViewButton"));
        saveViewButton->setMaximumSize(QSize(50, 16777215));

        verticalLayout_6->addWidget(saveViewButton);

        goButton = new QPushButton(groupBox);
        goButton->setObjectName(QString::fromUtf8("goButton"));
        goButton->setMaximumSize(QSize(50, 27));

        verticalLayout_6->addWidget(goButton);


        horizontalLayout_3->addLayout(verticalLayout_6);


        verticalLayout_7->addLayout(horizontalLayout_3);

        isObject = new QLabel(groupBox);
        isObject->setObjectName(QString::fromUtf8("isObject"));

        verticalLayout_7->addWidget(isObject);

        ResetPoseButton = new QPushButton(groupBox);
        ResetPoseButton->setObjectName(QString::fromUtf8("ResetPoseButton"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(ResetPoseButton->sizePolicy().hasHeightForWidth());
        ResetPoseButton->setSizePolicy(sizePolicy1);

        verticalLayout_7->addWidget(ResetPoseButton);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label_17 = new QLabel(groupBox);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        horizontalLayout_12->addWidget(label_17);

        tx_Slider = new QSlider(groupBox);
        tx_Slider->setObjectName(QString::fromUtf8("tx_Slider"));
        tx_Slider->setMinimum(-1000);
        tx_Slider->setMaximum(1000);
        tx_Slider->setOrientation(Qt::Horizontal);

        horizontalLayout_12->addWidget(tx_Slider);


        verticalLayout_8->addLayout(horizontalLayout_12);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        label_18 = new QLabel(groupBox);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        horizontalLayout_13->addWidget(label_18);

        ty_Slider = new QSlider(groupBox);
        ty_Slider->setObjectName(QString::fromUtf8("ty_Slider"));
        ty_Slider->setMinimum(-1000);
        ty_Slider->setMaximum(1000);
        ty_Slider->setOrientation(Qt::Horizontal);

        horizontalLayout_13->addWidget(ty_Slider);


        verticalLayout_8->addLayout(horizontalLayout_13);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        label_19 = new QLabel(groupBox);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        horizontalLayout_14->addWidget(label_19);

        tz_Slider = new QSlider(groupBox);
        tz_Slider->setObjectName(QString::fromUtf8("tz_Slider"));
        tz_Slider->setMinimum(-1000);
        tz_Slider->setMaximum(1000);
        tz_Slider->setOrientation(Qt::Horizontal);

        horizontalLayout_14->addWidget(tz_Slider);


        verticalLayout_8->addLayout(horizontalLayout_14);


        verticalLayout_7->addLayout(verticalLayout_8);


        verticalLayout_9->addLayout(verticalLayout_7);


        horizontalLayout_2->addWidget(groupBox);


        verticalLayout_3->addWidget(frame_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_5 = new QLabel(guiDlg);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout->addWidget(label_5);

        stateLabel = new QLabel(guiDlg);
        stateLabel->setObjectName(QString::fromUtf8("stateLabel"));

        horizontalLayout->addWidget(stateLabel);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QApplication::translate("guiDlg", "objectDetection", 0, QApplication::UnicodeUTF8));
        reloadButton->setText(QApplication::translate("guiDlg", "realoadVFH", 0, QApplication::UnicodeUTF8));
        findObjectButton->setText(QApplication::translate("guiDlg", "find the", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("guiDlg", "training", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("guiDlg", "pose object", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("guiDlg", "Y", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("guiDlg", "rotation object", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("guiDlg", "Z", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("guiDlg", "X", 0, QApplication::UnicodeUTF8));
        InitPose->setText(QApplication::translate("guiDlg", "Save Init Pose", 0, QApplication::UnicodeUTF8));
        regularPose->setText(QApplication::translate("guiDlg", "Save Regular Pose", 0, QApplication::UnicodeUTF8));
        ObtainPose->setText(QApplication::translate("guiDlg", "Obtain Pose of", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("guiDlg", "Label:", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("guiDlg", "Object ", 0, QApplication::UnicodeUTF8));
        saveViewButton->setText(QApplication::translate("guiDlg", "save", 0, QApplication::UnicodeUTF8));
        goButton->setText(QApplication::translate("guiDlg", "GO!", 0, QApplication::UnicodeUTF8));
        isObject->setText(QString());
        ResetPoseButton->setText(QApplication::translate("guiDlg", "Reset Pose to Save", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("guiDlg", "tx", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("guiDlg", "ty", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("guiDlg", "tz", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("guiDlg", "State:", 0, QApplication::UnicodeUTF8));
        stateLabel->setText(QApplication::translate("guiDlg", "AQUI VA EL ESTADO", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
