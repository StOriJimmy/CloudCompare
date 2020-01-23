#include "bdrLine3DppDlg.h"

//local
#include "mainwindow.h"

#include <ccOctree.h>

bdrLine3DppDlg::bdrLine3DppDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::BDRLine3DppDlg()
{
	setupUi(this);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(GenerateLines()));

	if (MainWindow::TheInstance()) {
		m_win = MainWindow::TheInstance();
	}
}

void bdrLine3DppDlg::GenerateLines()
{
	saveSettings();

#ifndef USE_STOCKER
	if (m_win) {
		m_win->dispToConsole("[Line3DPP] No stocker lib used!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
	return;
#endif // USE_STOCKER

	if (m_win) {
//		m_win->addToDB(plane);
	}
}

void bdrLine3DppDlg::saveSettings()
{

}