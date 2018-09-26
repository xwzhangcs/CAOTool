#include "AllOptionDialog.h"

AllOptionDialog::AllOptionDialog(QWidget *parent) : QDialog(parent) {
	ui.setupUi(this);

	ui.doubleSpinBoxAlpha->setValue(0.5);
	ui.doubleSpinBoxAlpha->setSingleStep(0.1);
	ui.doubleSpinBoxAlpha->setMinimum(0.0);
	ui.doubleSpinBoxAlpha->setMaximum(1.0);

	connect(ui.pushButtonOK, SIGNAL(clicked()), this, SLOT(onOK()));
	connect(ui.pushButtonCancel, SIGNAL(clicked()), this, SLOT(onCancel()));
}

AllOptionDialog::~AllOptionDialog() {
}

double AllOptionDialog::getAlpha() {
	return ui.doubleSpinBoxAlpha->value();
}

void AllOptionDialog::onOK() {
	accept();
}

void AllOptionDialog::onCancel() {
	reject();
}
