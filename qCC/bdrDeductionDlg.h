#ifndef BDR_DEDUCT_DLG_HEADER
#define BDR_DEDUCT_DLG_HEADER

#include "ui_bdrDeductionDlg.h"
#include "mainwindow.h"

class bdrDeductionDlg : public QDialog, public Ui::BDRDeductionDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrDeductionDlg(QWidget* parent = 0);

protected slots:
	

};

#endif
