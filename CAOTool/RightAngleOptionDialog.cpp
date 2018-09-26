#include "RightAngleOptionDialog.h"

RightAngleOptionDialog::RightAngleOptionDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.spinBoxResolution->setValue(20);
	ui.checkBoxOptimization->setChecked(true);
	ui.doubleSpinBoxLayeringThreshold->setValue(0.7);
	ui.doubleSpinBoxLayeringThreshold->setSingleStep(0.1);
	ui.doubleSpinBoxLayeringThreshold->setMinimum(0.0);
	ui.doubleSpinBoxLayeringThreshold->setMaximum(1.0);
	ui.doubleSpinBoxSnappingThreshold->setValue(3.0);
	ui.lineEditOrientation->setText("0");
	ui.lineEditMinContourArea->setText("2");
	ui.lineEditMaxOBBRatio->setText("10");
	ui.checkBoxAllowTriangleContour->setChecked(false);
	ui.checkBoxAllowOverhang->setChecked(false);

	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

RightAngleOptionDialog::~RightAngleOptionDialog() {
}

int RightAngleOptionDialog::getResolution() {
	return ui.spinBoxResolution->value();
}

bool RightAngleOptionDialog::getOptimization() {
	return ui.checkBoxOptimization->isChecked();
}

double RightAngleOptionDialog::getLayeringThreshold() {
	return ui.doubleSpinBoxLayeringThreshold->value();
}

double RightAngleOptionDialog::getSnappingThreshold() {
	return ui.doubleSpinBoxSnappingThreshold->value();
}

double RightAngleOptionDialog::getOrientation() {
	return ui.lineEditOrientation->text().toDouble();
}

double RightAngleOptionDialog::getMinContourArea() {
	return ui.lineEditMinContourArea->text().toDouble();
}

double RightAngleOptionDialog::getMaxOBBRatio() {
	return ui.lineEditMaxOBBRatio->text().toDouble();
}

bool RightAngleOptionDialog::isAllowTriangleContour() {
	return ui.checkBoxAllowTriangleContour->isChecked();
}

bool RightAngleOptionDialog::isAllowOverhang() {
	return ui.checkBoxAllowOverhang->isChecked();
}

void RightAngleOptionDialog::onOK() {
	accept();
}

void RightAngleOptionDialog::onCancel() {
	reject();
}
