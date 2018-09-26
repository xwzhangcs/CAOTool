#ifndef OFFSETSCALEDIALOG_H
#define OFFSETSCALEDIALOG_H

#include <QDialog>
#include "ui_OffsetScaleDialog.h"

class OffsetScaleDialog : public QDialog {
	Q_OBJECT

private:
	Ui::OffsetScaleDialog ui;

public:
	OffsetScaleDialog(QWidget *parent = 0);
	~OffsetScaleDialog();

	void setOffset(double x, double y, double z);
	double getOffsetX();
	double getOffsetY();
	double getOffsetZ();
	void setScale(double scale);
	double getScale();

public slots:
	void onD1();
	void onD2();
	void onD3();
	void onD4();
	void onOK();
	void onCancel();
};

#endif // OFFSETSCALEDIALOG_H
