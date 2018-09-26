#include "CurveRightAngleOptionDialog.h"

CurveRightAngleOptionDialog::CurveRightAngleOptionDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.spinBoxEpsilon->setValue(16);
	ui.doubleSpinBoxCurveThreshold->setValue(2.0);
	ui.doubleSpinBoxCurveThreshold->setSingleStep(0.1);
	ui.doubleSpinBoxAngleThreshold->setValue(10.0);
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

CurveRightAngleOptionDialog::~CurveRightAngleOptionDialog() {
}

int CurveRightAngleOptionDialog::getEpsilon() {
	return ui.spinBoxEpsilon->value();
}

double CurveRightAngleOptionDialog::getCurveThreshold() {
	return ui.doubleSpinBoxCurveThreshold->value();
}

double CurveRightAngleOptionDialog::getAngleThreshold() {
	return ui.doubleSpinBoxAngleThreshold->value();
}

double CurveRightAngleOptionDialog::getLayeringThreshold() {
	return ui.doubleSpinBoxLayeringThreshold->value();
}

double CurveRightAngleOptionDialog::getSnappingThreshold() {
	return ui.doubleSpinBoxSnappingThreshold->value();
}

double CurveRightAngleOptionDialog::getOrientation() {
	return ui.lineEditOrientation->text().toDouble();
}

double CurveRightAngleOptionDialog::getMinContourArea() {
	return ui.lineEditMinContourArea->text().toDouble();
}

double CurveRightAngleOptionDialog::getMaxOBBRatio() {
	return ui.lineEditMaxOBBRatio->text().toDouble();
}

bool CurveRightAngleOptionDialog::isAllowTriangleContour() {
	return ui.checkBoxAllowTriangleContour->isChecked();
}

bool CurveRightAngleOptionDialog::isAllowOverhang() {
	return ui.checkBoxAllowOverhang->isChecked();
}

void CurveRightAngleOptionDialog::onOK() {
	accept();
}

void CurveRightAngleOptionDialog::onCancel() {
	reject();
}