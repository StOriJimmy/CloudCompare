#include "bdrClassDisplayDlg.h"

bdrClassDisplayDlg::bdrClassDisplayDlg(QWidget* parent)
	: m_UI( new Ui::bdrClassDisplayDlg)
{
	m_UI->setupUi(this);

}

bdrClassDisplayDlg::~bdrClassDisplayDlg()
{

	delete m_UI;
}
