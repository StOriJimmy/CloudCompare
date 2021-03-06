#include "bdrSettingGrdFilterDlg.h"

bdrSettingGrdFilterDlg::bdrSettingGrdFilterDlg(QWidget* parent)
	: m_UI(new Ui::bdrSettingGrdFilterDlg)
{
	m_UI->setupUi(this);

}

bdrSettingGrdFilterDlg::~bdrSettingGrdFilterDlg()
{

	delete m_UI;
}

QStringList bdrSettingGrdFilterDlg::getParameters()
{
	QStringList paras; 
	QString method, para;
	if (m_UI->MethodSGFRadioButton->isChecked()) {
		method.append("SGFilter 3");
		para.append(QString::number(m_UI->SGFElevThresholdDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->SGFCellSizeDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->SGFElevScaleDoubleSpinBox->value())).append(" ");
	}
	else if (m_UI->MethodMorphRadioButton->isChecked())	{
		method.append("morph 7");
		para.append(QString::number(m_UI->MorphCellSizeDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->MorphMinElevDiffDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->MorphMaxElevDiffDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->MorphMaxWindowDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->MorphSearchBaseDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->MorphSlopeScaleDoubleSpinBox->value())).append(" ");
		para.append(QString::number(m_UI->MorphRegionGrowComboBox->currentIndex())).append(" ");
	}
	paras.append(method);
	paras.append(para);
	return paras;
}
