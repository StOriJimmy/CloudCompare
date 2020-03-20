#include "bdrPlaneQualityDlg.h"
#include "stocker_parser.h"
#include "ccHistogramWindow.h"

#include <QFileDialog>
#include <QToolButton>
#include <QPushButton>

bdrPlaneQualityDlg::bdrPlaneQualityDlg(QWidget* parent)
	: QDialog(parent)
	, m_UI(new Ui::bdrPlaneQualityDlg)
{
	m_UI->setupUi(this);
	setWindowFlags(windowFlags()&~Qt::WindowCloseButtonHint);

	connect(m_UI->buttonBox,					SIGNAL(accepted()), this, SLOT(exitSafe()));
	connect(m_UI->buttonBox,					SIGNAL(rejected()), this, SLOT(exitSafe()));
	connect(m_UI->calculateToolButton,			&QAbstractButton::clicked, this, &bdrPlaneQualityDlg::onCalculate);
	connect(m_UI->filtertoolButton,				&QAbstractButton::clicked, this, &bdrPlaneQualityDlg::onFilterPlane);
	connect(m_UI->histogramToolButton,			&QAbstractButton::clicked, this, &bdrPlaneQualityDlg::onShowHistrogram);
	connect(m_UI->facadeFilterToolButton,		&QAbstractButton::clicked, this, &bdrPlaneQualityDlg::onFilterFacade);
	

// 	connect(m_UI->buttonBox, SIGNAL(accepted()), this, SLOT(AcceptAndExit()));
// 	connect(m_UI->openFileToolButton, &QAbstractButton::clicked, this, &bdrPosImportDlg::doActionOpenFile);
}

void bdrPlaneQualityDlg::exitSafe() 
{
	m_planes.clear();
}

void bdrPlaneQualityDlg::onCalculate()
{
	int flatMode = 0;
	if (m_UI->flatAreaRadioButton->isChecked()) {
		flatMode = 0;
	}
	else if (m_UI->flatPeriRadioButton->isChecked()) {
		flatMode = 1;
	}
	CalculatePlaneQuality(m_planes, flatMode);
}

void bdrPlaneQualityDlg::onFilterPlane()
{
	double threshold = m_UI->thresholdDoubleSpinBox->value();

	ProgStartNorm("filter planes by quality...", m_planes.size());
	for (ccHObject* plane : m_planes) {
		if (!plane->hasMetaData(QString("Flatness"))) { ProgStepBreak continue; }

		bool ok = false;
		double flatness = plane->getMetaData(QString("Flatness")).toDouble(&ok);
		if (!ok) { ProgStepBreak continue; }
		
		bool sel = false;
		if (m_UI->greaterRadioButton->isChecked()) {
			sel = flatness > threshold;
		}
		else if (m_UI->smallerRadioButton->isChecked()) {
			sel = flatness < threshold;
		}
		else {
			sel = fabs(flatness - threshold) < FLT_EPSILON;
		}
		ccHObject* plane_cloud = GetPlaneCloud(plane);
		if (plane_cloud) {
			plane_cloud->setEnabled(sel);
		}
		else {
			plane->setEnabled(sel);
		}
		ProgStepBreak
	}
	ProgEnd
}

size_t g_dist_bin_count = 20;

void bdrPlaneQualityDlg::onShowHistrogram()
{
	QString qualityType;
	if (m_UI->qualityFlatnessRadioButton->isChecked()) {
		qualityType = QString("Flatness");
	}
	else if (m_UI->qualityAreaRadioButton->isChecked()) {
		qualityType = QString("Area");
	}
	else if (m_UI->qualityRMSRadioButton->isChecked()) {
		qualityType = QString("RMS");
	}

	std::vector<double> all_conf;
	for (ccHObject* plane : m_planes) {
		if (!plane->hasMetaData(qualityType)) { continue; }

		bool ok = false;
		double flatness = plane->getMetaData(qualityType).toDouble(&ok);
		if (!ok) continue;
		all_conf.push_back(flatness);
	}
	if (all_conf.empty()) return;
	sort(all_conf.begin(), all_conf.end());
	double min_conf = all_conf.front(); 
	double max_conf = all_conf.back();
	double range_conf = max_conf - min_conf;
	double step_conf = range_conf / static_cast<double>(g_dist_bin_count);

	if (step_conf < FLT_EPSILON) step_conf = 1;
	std::vector<unsigned> histo;
	histo.resize(g_dist_bin_count, 0);
	for (auto f : all_conf)	{
		size_t bin = static_cast<size_t>(floor((f - min_conf) / step_conf));
		++histo[std::min(bin, g_dist_bin_count - 1)];
	}

	ccHistogramWindowDlg* hDlg = new ccHistogramWindowDlg(this);
	hDlg->setAttribute(Qt::WA_DeleteOnClose, true);
	hDlg->setWindowTitle("Flatness Histogram");
	ccHistogramWindow* histogram = hDlg->window();
	{
		histogram->setTitle("Flatness histogram");
		histogram->fromBinArray(histo, min_conf, max_conf);
		histogram->setAxisLabels("Flatness", "Count");
		histogram->refresh();
	}

	hDlg->show();
}

void bdrPlaneQualityDlg::onFilterFacade()
{
	bool rooftop = m_UI->rooftopCheckBox->isChecked();
	bool facade = m_UI->facadeCheckBox->isChecked();
	CCVector3 normal = CCVector3(
		m_UI->facadeNxDoubleSpinBox->value(),
		m_UI->facadeNyDoubleSpinBox->value(),
		m_UI->facadeNzDoubleSpinBox->value());

	normal.normalize();
	if (fabs(normal.norm()) < 1e-6) {
		return;
	}
	for (ccHObject* plane : m_planes) {
		ccHObject* plane_to_check = GetPlaneCloud(plane);
		if (!plane_to_check) plane_to_check = plane;

		if (rooftop && facade) {
			plane_to_check->setEnabled(true);
			continue;
		}
		if (!rooftop && !facade) {
			plane_to_check->setEnabled(false);
			continue;
		}
		ccPlane* planeObj = ccHObjectCaster::ToPlane(plane);
		if (planeObj) {
			bool vertical = planeObj->isVerticalToDirection(normal, m_UI->facadeAngleDoubleSpinBox->value());
			plane_to_check->setEnabled((vertical && facade) || (!vertical && !facade));
		}
	}
}
