#ifndef EFFICIENTRANSACOPTIONDIALOG_H
#define EFFICIENTRANSACOPTIONDIALOG_H

#include <QDialog>
#include "ui_EfficientRANSACOptionDialog.h"

class EfficientRANSACOptionDialog : public QDialog
{
	Q_OBJECT

public:
	EfficientRANSACOptionDialog(QWidget *parent = Q_NULLPTR);
	~EfficientRANSACOptionDialog();

	int getCurveNumIterations();
	int getCurveMinPoints();
	double getCurveMaxErrorRatioToRadius();
	double getCurveClusterEpsilon();
	double getCurveMinAngle();
	double getCurveMinRadius();
	double getCurveMaxRadius();

	int getLineNumIterations();
	double getLineMinPoints();
	double getLineMaxError();
	double getLineClusterEpsilon();
	double getLineMinLength();
	double getLineAngleThreshold();

	double getContourMaxError();
	double getContourAngleThreshold();

	double getLayeringThreshold();
	double getSnappingThreshold();
	double getOrientation();
	double getMinContourArea();
	double getMaxOBBRatio();
	bool isAllowTriangleContour();
	bool isAllowOverhang();
	QString getConfigFile();

private:
	Ui::EfficientRANSACOptionDialog ui;

public slots:
	void onOK();
	void onCancel();
	void onGetConfigFile();
};

#endif