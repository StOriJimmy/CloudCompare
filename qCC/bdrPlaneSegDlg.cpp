#include "bdrPlaneSegDlg.h"

#include <QFileInfo>

//local
#include "mainwindow.h"
#include <QMessageBox>
#include "ccItemSelectionDlg.h"

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
	connect(loadResultsToolButton, &QAbstractButton::clicked, this, &bdrPlaneSegDlg::loadResults);
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
			int selectedIndex = 0;
			if (m_point_clouds.size() > 1) {
				selectedIndex = ccItemSelectionDlg::SelectEntity(m_point_clouds, selectedIndex, this, "please select the point cloud to calculate average spacing");
				if (selectedIndex < 0)
					return;
				assert(selectedIndex >= 0 && static_cast<size_t>(selectedIndex) < m_point_clouds.size());
			}
			ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(m_point_clouds[selectedIndex]);
			if (!cloud) { QMessageBox::critical(this, "Error", "error point cloud"); ProgEnd return; }
			Concurrency::concurrent_vector<ATPS::SVPoint3d> points;
			points.reserve(cloud->size());
			Concurrency::parallel_for(unsigned(0), cloud->size(), [&](unsigned i) {
				const CCVector3* pt = cloud->getPoint(i);
				points.push_back({ pt->x, pt->y, pt->z });
			});
			QCoreApplication::processEvents();
			double res = atps_plane.get_res({ points.begin(), points.end() });
			std::cout << "average spacing for cloud " << cloud->getName().toStdString() << " is: " << res << std::endl;
			atps_plane.set_parameters(res);
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

void bdrPlaneSegDlg::loadResults()
{
	MainWindow* win = MainWindow::TheInstance();
	if (!win) return;

	ProgStartNorm("loading plane segmentation results...", m_point_clouds.size())
	for (ccHObject* pc : m_point_clouds) {
		ccPointCloud* pcObj = ccHObjectCaster::ToPointCloud(pc); if (!pcObj) continue;

		ccPointCloud* todo_point = nullptr;
		BDBaseHObject * baseObj = GetRootBDBase(pcObj);
		if (baseObj) {
			todo_point = baseObj->GetTodoPoint(GetBaseName(pcObj->getName()));
			if (todo_point) todo_point->clear();
		}
		else {
			try	{
				todo_point = new ccPointCloud("unassigned");
			}
			catch (...) {
				if (todo_point) delete todo_point;
				todo_point = nullptr;
			}
			if (todo_point) {
				todo_point->setGlobalScale(pcObj->getGlobalScale());
				todo_point->setGlobalShift(pcObj->getGlobalShift());
				todo_point->setDisplay(pcObj->getDisplay());
				todo_point->showColors(true);
				if (pcObj->getParent())
					pcObj->getParent()->addChild(todo_point);

				win->addToDB(todo_point, pc->getDBSourceType(), false, false);
			}
		}

		StPrimGroup* group = LoadPlaneParaAsPrimtiveGroup(pcObj, todo_point);
		if (group) {
			win->addToDB(group, pc->getDBSourceType(), false, false);
		}
		if (todo_point && todo_point->size() == 0) {
			win->removeFromDB(todo_point);
		}
		ProgStepBreak
	}
	ProgEnd
	QDialog::reject();
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
