#ifndef DPOPTIONDIALOG_H
#define DPOPTIONDIALOG_H

#include <QDialog>
#include "ui_DPOptionDialog.h"

class DPOptionDialog : public QDialog {
	Q_OBJECT

private:
	Ui::DPOptionDialog ui;

public:
	DPOptionDialog(QWidget *parent = 0);
	~DPOptionDialog();

	int getEpsilon();
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

#endif // DPOPTIONDIALOG_H
