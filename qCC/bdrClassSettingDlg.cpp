#include "bdrClassSettingDlg.h"

bdrClassSettingDlg::bdrClassSettingDlg(QWidget* parent)
	: m_UI(new Ui::bdrClassSettingDlg)
{
	m_UI->setupUi(this);

}

bdrClassSettingDlg::~bdrClassSettingDlg()
{

	delete m_UI;
}
