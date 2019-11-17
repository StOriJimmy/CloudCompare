#include "bdrPlaneSegDlg.h"

//local
#include "mainwindow.h"
#include <QMessageBox>

#include <ppl.h>
#include <concurrent_vector.h>

#include <ccOctree.h>
#ifdef PCATPS_SUPPORT
#include "PC_ATPS.h"
#endif
#include "stocker_parser.h"

bdrPlaneSegDlg::bdrPlaneSegDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::BDRPlaneSegDlg()
{
	setupUi(this);
	setWindowFlags(windowFlags()&~Qt::WindowCloseButtonHint);
	connect(autoParaCheckBox, &QAbstractButton::toggled, this, &bdrPlaneSegDlg::onAutoChecked);
	connect(autoParaToolButton, &QAbstractButton::clicked, this, &bdrPlaneSegDlg::DeducePara);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(Execute()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(exitSafe()));
}
void bdrPlaneSegDlg::onAutoChecked(bool state)
{
	APTSCurvatureSpinBox->setDisabled(state);
	APTSNFASpinBox->setDisabled(state);
	APTSNormalSpinBox->setDisabled(state);
	maxNormDevAngleSpinBox->setDisabled(state);
	probaDoubleSpinBox->setDisabled(state);
	GrowingRadiusDoubleSpinBox->setDisabled(state);	
}

void bdrPlaneSegDlg::DeducePara()
{
	if (m_point_clouds.empty()) {
		return;
	}
	if (PlaneSegATPSRadioButton->isChecked()) {

		ATPS::ATPS_Plane atps_plane;
		ProgStart("Please wair...Deduce parameters automatically...")
		try
		{
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_point_clouds.front());
			if (!cloud) { QMessageBox::critical(this, "Error", "error point cloud"); ProgEnd return; }
			Concurrency::concurrent_vector<ATPS::SVPoint3d> points;
			points.reserve(cloud->size());
			Concurrency::parallel_for(unsigned(0), cloud->size(), [&](unsigned i) {
				const CCVector3* pt = cloud->getPoint(i);
				points.push_back({ pt->x, pt->y, pt->z });
			});
			QCoreApplication::processEvents();
			atps_plane.get_res({ points.begin(), points.end() });
		}
		catch (const std::exception& e) {
			QMessageBox::critical(this, "Cannot deduce params", QString::fromStdString(e.what()));
			ProgEnd
			return;
		}
		
		ProgEnd

		supportPointsSpinBox->setValue(atps_plane.get_kappa());
		APTSCurvatureSpinBox->setValue(atps_plane.get_delta());
		DistanceEpsilonDoubleSpinBox->setValue(atps_plane.get_tau());
		ClusterEpsilonDoubleSpinBox->setValue(atps_plane.get_gamma());
		APTSNFASpinBox->setValue(atps_plane.get_epsilon());
		APTSNormalSpinBox->setValue(atps_plane.get_theta());
	}
	else {
		ccPointCloud* pc = ccHObjectCaster::ToPointCloud(m_point_clouds.front());
		if (pc)	{
			CCVector3 bbmin, bbmax;	pc->getBoundingBox(bbmin, bbmax);
			CCVector3 diff = bbmax - bbmin;
			double scale = std::max(std::max(diff[0], diff[1]), diff[2]);
			DistanceEpsilonDoubleSpinBox->setValue(.005f * scale);
			ClusterEpsilonDoubleSpinBox->setValue(.01f * scale);
		}
	}
}

void bdrPlaneSegDlg::setPointClouds(ccHObject::Container point_clouds)
{
	pcNumberLineEdit->setText(QString::number(point_clouds.size()));
	m_point_clouds = point_clouds;
}

void bdrPlaneSegDlg::exitSafe()
{
	m_point_clouds.clear();
}

void bdrPlaneSegDlg::Execute()
{
	m_point_clouds.clear();
}
