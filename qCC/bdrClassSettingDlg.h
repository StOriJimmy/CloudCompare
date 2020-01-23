#ifndef BDR_CLASS_SETTING_DLG_HEADER
#define BDR_CLASS_SETTING_DLG_HEADER

#include "ui_bdrClassSettingDlg.h"

namespace Ui
{
	class bdrClassSettingDlg;
}

class bdrClassSettingDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrClassSettingDlg(QWidget* parent);
	//! Destructor
	~bdrClassSettingDlg() override;

protected slots:

private: //members
	Ui::bdrClassSettingDlg	*m_UI;
};

#endif //BDR_TRACE_FOOTPRINT_HEADER
