#ifndef ALLOPTIONDIALOG_H
#define ALLOPTIONDIALOG_H

#include <QDialog>
#include "ui_AllOptionDialog.h"

class AllOptionDialog : public QDialog {
	Q_OBJECT

private:
	Ui::AllOptionDialog ui;

public:
	AllOptionDialog(QWidget *parent = 0);
	~AllOptionDialog();

	double getAlpha();

public slots:
	void onOK();
	void onCancel();
};

#endif // ALLOPTIONDIALOG_H
