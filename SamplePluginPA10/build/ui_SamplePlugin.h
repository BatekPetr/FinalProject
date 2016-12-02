/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *_btn_Start;
    QPushButton *_btn_Select;
    QPushButton *_btn_dT_Sim;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_4;
    QRadioButton *_trackMPt;
    QRadioButton *_track1Pt;
    QSpinBox *_spinBox;
    QPushButton *_btn_Restart;
    QLabel *_label;
    QSlider *_slider;
    QButtonGroup *buttonGroup;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(420, 500);
        SamplePlugin->setMinimumSize(QSize(420, 50));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        horizontalLayoutWidget = new QWidget(dockWidgetContents);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 0, 396, 61));
        horizontalLayout_4 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        _btn_Start = new QPushButton(horizontalLayoutWidget);
        _btn_Start->setObjectName(QString::fromUtf8("_btn_Start"));

        horizontalLayout_4->addWidget(_btn_Start);

        _btn_Select = new QPushButton(horizontalLayoutWidget);
        _btn_Select->setObjectName(QString::fromUtf8("_btn_Select"));

        horizontalLayout_4->addWidget(_btn_Select);

        _btn_dT_Sim = new QPushButton(horizontalLayoutWidget);
        _btn_dT_Sim->setObjectName(QString::fromUtf8("_btn_dT_Sim"));

        horizontalLayout_4->addWidget(_btn_dT_Sim);

        horizontalLayoutWidget_2 = new QWidget(dockWidgetContents);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 60, 391, 61));
        horizontalLayout_5 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        _trackMPt = new QRadioButton(horizontalLayoutWidget_2);
        buttonGroup = new QButtonGroup(SamplePlugin);
        buttonGroup->setObjectName(QString::fromUtf8("buttonGroup"));
        buttonGroup->addButton(_trackMPt);
        _trackMPt->setObjectName(QString::fromUtf8("_trackMPt"));
        _trackMPt->setChecked(true);

        verticalLayout_4->addWidget(_trackMPt);

        _track1Pt = new QRadioButton(horizontalLayoutWidget_2);
        buttonGroup->addButton(_track1Pt);
        _track1Pt->setObjectName(QString::fromUtf8("_track1Pt"));
        _track1Pt->setChecked(false);

        verticalLayout_4->addWidget(_track1Pt);


        horizontalLayout_5->addLayout(verticalLayout_4);

        _spinBox = new QSpinBox(horizontalLayoutWidget_2);
        _spinBox->setObjectName(QString::fromUtf8("_spinBox"));
        _spinBox->setMaximum(1000);
        _spinBox->setSingleStep(50);
        _spinBox->setValue(1000);

        horizontalLayout_5->addWidget(_spinBox);

        _btn_Restart = new QPushButton(horizontalLayoutWidget_2);
        _btn_Restart->setObjectName(QString::fromUtf8("_btn_Restart"));

        horizontalLayout_5->addWidget(_btn_Restart);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QString::fromUtf8("_label"));
        _label->setGeometry(QRect(10, 140, 401, 320));
        QFont font;
        font.setPointSize(8);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        _label->setFont(font);
        _slider = new QSlider(dockWidgetContents);
        _slider->setObjectName(QString::fromUtf8("_slider"));
        _slider->setGeometry(QRect(0, 120, 400, 20));
        _slider->setAutoFillBackground(false);
        _slider->setMaximum(1000);
        _slider->setSingleStep(50);
        _slider->setPageStep(50);
        _slider->setOrientation(Qt::Horizontal);
        _slider->setInvertedAppearance(true);
        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QApplication::translate("SamplePlugin", "Doc&kWidget", 0, QApplication::UnicodeUTF8));
        _btn_Start->setText(QApplication::translate("SamplePlugin", "Start Visual Servoing", 0, QApplication::UnicodeUTF8));
        _btn_Select->setText(QApplication::translate("SamplePlugin", "Select Sequence", 0, QApplication::UnicodeUTF8));
        _btn_dT_Sim->setText(QApplication::translate("SamplePlugin", "dT Simulation", 0, QApplication::UnicodeUTF8));
        _trackMPt->setText(QApplication::translate("SamplePlugin", "Track Mu&ltiple Points", 0, QApplication::UnicodeUTF8));
        _track1Pt->setText(QApplication::translate("SamplePlugin", "Track &1 Point", 0, QApplication::UnicodeUTF8));
        _spinBox->setSuffix(QApplication::translate("SamplePlugin", " ms", 0, QApplication::UnicodeUTF8));
        _btn_Restart->setText(QApplication::translate("SamplePlugin", "Restart", 0, QApplication::UnicodeUTF8));
        _label->setText(QApplication::translate("SamplePlugin", "Label", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
