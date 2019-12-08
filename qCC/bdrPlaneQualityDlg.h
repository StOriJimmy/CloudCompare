#ifndef BDR_PLANEQUALITY_DLG_HEADER
#define BDR_PLANEQUALITY_DLG_HEADER

#include "ui_bdrPlaneQualityDlg.h"
#include "mainwindow.h"

namespace Ui
{
	class bdrPlaneQualityDlg;
}

class bdrPlaneQualityDlg : public QDialog
{
	Q_OBJECT

public:
	explicit bdrPlaneQualityDlg(QWidget* parent = 0);
	~bdrPlaneQualityDlg() {}

private:
	Ui::bdrPlaneQualityDlg	*m_UI;

protected slots:
	void exitSafe();
	void onCalculateFlatness();
	void onFilterPlane();
	void onShowHistrogram();

public:
	void setPlanes(ccHObject::Container planes) { m_planes = planes; }

protected:
	ccHObject::Container m_planes;
};

#endif
