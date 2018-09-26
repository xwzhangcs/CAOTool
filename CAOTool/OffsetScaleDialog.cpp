#include "OffsetScaleDialog.h"

OffsetScaleDialog::OffsetScaleDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.lineEditOffsetX->setText("0");
	ui.lineEditOffsetY->setText("0");
	ui.lineEditOffsetZ->setText("0");
	ui.doubleSpinBoxScale->setValue(0.3);
	ui.doubleSpinBoxScale->setSingleStep(0.1);
	ui.doubleSpinBoxScale->setMinimum(0.1);
	ui.doubleSpinBoxScale->setMaximum(10.0);

	connect(ui.pushButtonD1, SIGNAL(clicked()), this, SLOT(onD1()));
	connect(ui.pushButtonD2, SIGNAL(clicked()), this, SLOT(onD2()));
	connect(ui.pushButtonD3, SIGNAL(clicked()), this, SLOT(onD3()));
	connect(ui.pushButtonD4, SIGNAL(clicked()), this, SLOT(onD4()));
	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

OffsetScaleDialog::~OffsetScaleDialog() {
}

void OffsetScaleDialog::onD1() {
	ui.lineEditOffsetX->setText("0");
	ui.lineEditOffsetY->setText("0");
}

void OffsetScaleDialog::onD2() {
	ui.lineEditOffsetX->setText("749349.916482");
	ui.lineEditOffsetY->setText("4407879.580789001");
}

void OffsetScaleDialog::onD3() {
	ui.lineEditOffsetX->setText("0");
	ui.lineEditOffsetY->setText("0");
}

void OffsetScaleDialog::onD4() {
	ui.lineEditOffsetX->setText("435510.093075");
	ui.lineEditOffsetY->setText("3355541.032361");
}

void OffsetScaleDialog::setOffset(double x, double y, double z) {
	ui.lineEditOffsetX->setText(QString::number(x, 'f', 20));
	ui.lineEditOffsetY->setText(QString::number(y, 'f', 20));
	ui.lineEditOffsetZ->setText(QString::number(z, 'f', 20));
}

double OffsetScaleDialog::getOffsetX() {
	return ui.lineEditOffsetX->text().toDouble();
}

double OffsetScaleDialog::getOffsetY() {
	return ui.lineEditOffsetY->text().toDouble();
}

double OffsetScaleDialog::getOffsetZ() {
	return ui.lineEditOffsetZ->text().toDouble();
}

void OffsetScaleDialog::setScale(double scale) {
	ui.doubleSpinBoxScale->setValue(scale);
}

double OffsetScaleDialog::getScale() {
	return ui.doubleSpinBoxScale->value();
}

void OffsetScaleDialog::onOK() {
	accept();
}

void OffsetScaleDialog::onCancel() {
	reject();
}
