#ifndef BDR_3D4EM_DLG_HEADER
#define BDR_3D4EM_DLG_HEADER

#include "ui_bdrSettingLoD2Dlg.h"
#include "mainwindow.h"

class bdrSettingLoD2Dlg : public QDialog, public Ui::bdrSettingLoD2Dlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrSettingLoD2Dlg(QWidget* parent = 0);

	int GroundHeightMode();
	double UserDefinedGroundHeight();
	double ground_height;

protected slots:

	void browseConfigureFilename();
	//! Saves (temporarily) the dialog parameters on acceptation
	void saveSettings();


};

#endif
