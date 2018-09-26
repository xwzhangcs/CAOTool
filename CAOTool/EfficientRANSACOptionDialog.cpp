#include "EfficientRANSACOptionDialog.h"
#include <QFileDialog>

EfficientRANSACOptionDialog::EfficientRANSACOptionDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	ui.lineEditCurveNumIterations->setText("200000");
	ui.lineEditCurveMinPoints->setText("200");
	ui.lineEditCurveMaxErrorRatioToRadius->setText("0.02");
	ui.lineEditCurveClusterEpsilon->setText("30");
	ui.lineEditCurveMinAngle->setText("90");
	ui.lineEditCurveMinRadius->setText("80");
	ui.lineEditCurveMaxRadius->setText("400");

	ui.lineEditLineNumIterations->setText("20000");
	//ui.lineEditLineMinPoints->setText("2");
	ui.lineEditLineMinPoints->setText("1");
	ui.lineEditLineMaxError->setText("3");
	//ui.lineEditLineClusterEpsilon->setText("1.5");
	ui.lineEditLineClusterEpsilon->setText("1");
	//ui.lineEditLineMinLength->setText("2");
	ui.lineEditLineMinLength->setText("1");
	ui.lineEditLineAngleThreshold->setText("10");

	ui.lineEditContourMaxError->setText("10");
	ui.lineEditContourAngleThreshold->setText("10");
	ui.doubleSpinBoxLayeringThreshold->setValue(0.8);
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
	connect(ui.pushButtonInputConfig, SIGNAL(clicked()), this, SLOT(onGetConfigFile()));
}

EfficientRANSACOptionDialog::~EfficientRANSACOptionDialog()
{
}

int EfficientRANSACOptionDialog::getCurveNumIterations() {
	return ui.lineEditCurveNumIterations->text().toInt();
}

int EfficientRANSACOptionDialog::getCurveMinPoints() {
	return ui.lineEditCurveMinPoints->text().toInt();
}

double EfficientRANSACOptionDialog::getCurveMaxErrorRatioToRadius() {
	return ui.lineEditCurveMaxErrorRatioToRadius->text().toDouble();
}

double EfficientRANSACOptionDialog::getCurveClusterEpsilon() {
	return ui.lineEditCurveClusterEpsilon->text().toDouble();
}

double EfficientRANSACOptionDialog::getCurveMinAngle() {
	return ui.lineEditCurveMinAngle->text().toDouble();
}

double EfficientRANSACOptionDialog::getCurveMinRadius() {
	return ui.lineEditCurveMinRadius->text().toDouble();
}

double EfficientRANSACOptionDialog::getCurveMaxRadius() {
	return ui.lineEditCurveMaxRadius->text().toDouble();
}

int EfficientRANSACOptionDialog::getLineNumIterations() {
	return ui.lineEditLineNumIterations->text().toInt();
}

double EfficientRANSACOptionDialog::getLineMinPoints() {
	return ui.lineEditLineMinPoints->text().toDouble();
}

double EfficientRANSACOptionDialog::getLineMaxError() {
	return ui.lineEditLineMaxError->text().toDouble();
}

double EfficientRANSACOptionDialog::getLineClusterEpsilon() {
	return ui.lineEditLineClusterEpsilon->text().toDouble();
}

double EfficientRANSACOptionDialog::getLineMinLength() {
	return ui.lineEditLineMinLength->text().toDouble();
}

double EfficientRANSACOptionDialog::getLineAngleThreshold() {
	return ui.lineEditLineAngleThreshold->text().toDouble();
}

double EfficientRANSACOptionDialog::getContourMaxError() {
	return ui.lineEditContourMaxError->text().toDouble();
}

double EfficientRANSACOptionDialog::getContourAngleThreshold() {
	return ui.lineEditContourAngleThreshold->text().toDouble();
}

double EfficientRANSACOptionDialog::getLayeringThreshold() {
	return ui.doubleSpinBoxLayeringThreshold->value();
}

double EfficientRANSACOptionDialog::getSnappingThreshold() {
	return ui.doubleSpinBoxSnappingThreshold->value();
}

double EfficientRANSACOptionDialog::getOrientation() {
	return ui.lineEditOrientation->text().toDouble();
}

double EfficientRANSACOptionDialog::getMinContourArea() {
	return ui.lineEditMinContourArea->text().toDouble();
}

double EfficientRANSACOptionDialog::getMaxOBBRatio() {
	return ui.lineEditMaxOBBRatio->text().toDouble();
}

bool EfficientRANSACOptionDialog::isAllowTriangleContour() {
	return ui.checkBoxAllowTriangleContour->isChecked();
}

bool EfficientRANSACOptionDialog::isAllowOverhang() {
	return ui.checkBoxAllowOverhang->isChecked();
}

QString EfficientRANSACOptionDialog::getConfigFile(){
	return ui.lineEditInputConfig->text();
}

void EfficientRANSACOptionDialog::onOK() {
	accept();
}

void EfficientRANSACOptionDialog::onCancel() {
	reject();
}

void EfficientRANSACOptionDialog::onGetConfigFile(){
	QString dir = QFileDialog::getOpenFileName(this, tr("Load config file..."), ui.lineEditInputConfig->text(), tr("Config file (*.json)"));
	if (!dir.isEmpty()) {
		ui.lineEditInputConfig->setText(dir);
	}
}