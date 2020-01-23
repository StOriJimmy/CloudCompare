#include "bdrSettingLoD2Dlg.h"

//local
#include "mainwindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrSettingLoD2Dlg::bdrSettingLoD2Dlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::bdrSettingLoD2Dlg()
{
	setupUi(this);

	connect(ConfigureFilePathToolButton, &QAbstractButton::clicked, this, &bdrSettingLoD2Dlg::browseConfigureFilename);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings()));

	roofStructureGroupBox->setVisible(false);
}

int bdrSettingLoD2Dlg::GroundHeightMode()
{
	if (GroundHeightLowestRadioButton->isChecked()) {
		return 0;
	}
	else if (GroundHeightContourRadioButton->isChecked()) {
		return 1;
	}
	else if (GroundHeightUserRadioButton->isChecked()) {
		return 2;
	}
	return 0;
}

double bdrSettingLoD2Dlg::UserDefinedGroundHeight()
{
	return GroundHeightUserDoubleSpinBox->value();
}

void bdrSettingLoD2Dlg::browseConfigureFilename()
{
	QString Filename =
		QFileDialog::getOpenFileName(this,
			"Open Configuration file",
			ConfigureFilePathLineEdit->text(),
			"Configuration (*.ini)");

	if (!Filename.isEmpty())
		ConfigureFilePathLineEdit->setText(Filename);
}

void bdrSettingLoD2Dlg::saveSettings()
{
	ConfigureFilePathLineEdit;
}