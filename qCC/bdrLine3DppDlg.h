#ifndef BDR_LINE3DPP_DLG_HEADER
#define BDR_LINE3DPP_DLG_HEADER

#include "ui_bdrLine3DppDlg.h"
#include "mainwindow.h"

class bdrLine3DppDlg : public QDialog, public Ui::BDRLine3DppDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrLine3DppDlg(QWidget* parent = 0);

protected slots:

	//! Generate Lines on acceptation
	void GenerateLines();

protected:
	//! Saves (temporarily) the dialog parameters on acceptation
	void saveSettings();

	MainWindow* m_win;
};

#endif
