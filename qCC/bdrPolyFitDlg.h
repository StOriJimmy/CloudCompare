#ifndef BDR_POLYFIT_DLG_HEADER
#define BDR_POLYFIT_DLG_HEADER

#include "ui_bdrPolyFitDlg.h"
#include "mainwindow.h"

class bdrPolyFitDlg : public QDialog, public Ui::BDRPolyFitDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrPolyFitDlg(QWidget* parent = 0);

protected slots:
	 
};

#endif
