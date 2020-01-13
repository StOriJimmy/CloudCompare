#ifndef BDR_PLANESEG_DLG_HEADER
#define BDR_PLANESEG_DLG_HEADER

#include "ui_bdrPlaneSegDlg.h"
#include "mainwindow.h"

class bdrPlaneSegDlg : public QDialog, public Ui::BDRPlaneSegDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrPlaneSegDlg(QWidget* parent = 0);
	void setPointClouds(ccHObject::Container point_clouds);

protected slots:
	void onAutoChecked(bool);
	void DeducePara();
	void loadResults();
	void exitSafe();
	void Execute();

protected:
	ccHObject::Container m_point_clouds;
};

#endif
