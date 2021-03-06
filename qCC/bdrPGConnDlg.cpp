#include "bdrPGConnDlg.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>
#include <QMessageBox>

bdrPGConnDlg::bdrPGConnDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrPGConnDlg)
{
	m_UI->setupUi(this);

	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(AcceptAndExit()));
	connect(m_UI->testConnectToolButton, &QPushButton::clicked, this, &bdrPGConnDlg::doActionTestConnection);
}

void bdrPGConnDlg::AcceptAndExit()
{

}

bool bdrPGConnDlg::doActionTestConnection()
{
	QMessageBox::information(this, "nothing to do", "under construction...");
	return true;
}