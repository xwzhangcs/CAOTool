#ifndef CURVEOPTIONDIALOG_H
#define CURVEOPTIONDIALOG_H

#include <QDialog>
#include "ui_CurveOptionDialog.h"

class CurveOptionDialog : public QDialog {
	Q_OBJECT

private:
	Ui::CurveOptionDialog ui;

public:
	CurveOptionDialog(QWidget *parent = 0);
	~CurveOptionDialog();

	int getEpsilon();
	double getCurveThreshold();
	double getLayeringThreshold();
	double getSnappingThreshold();
	double getOrientation();
	double getMinContourArea();
	double getMaxOBBRatio();
	bool isAllowTriangleContour();
	bool isAllowOverhang();

public slots:
	void onOK();
	void onCancel();
};

#endif // CURVEOPTIONDIALOG_H
