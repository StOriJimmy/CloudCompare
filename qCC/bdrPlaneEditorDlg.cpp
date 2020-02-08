#include "bdrPlaneEditorDlg.h"

//local
#include "mainwindow.h"

//common
#include <ccPickingHub.h>

//qCC_db
#include <ccPlane.h>
#include <ccFacet.h>
#include "StBlock.h"
#include <ccNormalVectors.h>

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QDoubleValidator>
#include <QMenu>

#include "stocker_parser.h"

//semi-persistent parameters
static double s_dip = 0;
static double s_dipDir = 0;
static double s_width = 10.0;
static double s_height = 10.0;
static bool s_upward = true;
static CCVector3d s_center(0, 0, 0);

inline ccPlane* getMainPlaneFromInterface(ccPlanarEntityInterface* plane) {
	ccHObject* planeEnt = plane->getPlane();
	ccPlane* plane_ = nullptr;
	if (planeEnt->isA(CC_TYPES::PLANE)) {
		plane_ = ccHObjectCaster::ToPlane(planeEnt);
	}
	else if (planeEnt->isA(CC_TYPES::ST_BLOCK)) {
		plane_ = ccHObjectCaster::ToStBlock(planeEnt)->getMainPlane();
	}
	return plane_;
}

bdrPlaneEditorDlg::bdrPlaneEditorDlg(ccPickingHub* pickingHub, QWidget* parent)
	: QDialog(parent)
	, Ui::BDRPlaneEditorDlg()
	, m_pickingWin(0)
	, m_associatedPlane(0)
	, m_pickingHub(pickingHub)
	, m_display_state(DISPLAY_PLANE)
{
	assert(pickingHub);

	setModal(false);
	setupUi(this);

	//restore semi-persistent parameters
	wDoubleSpinBox->setValue(s_width);
	hDoubleSpinBox->setValue(s_height);
	cxAxisDoubleSpinBox->setValue(s_center.x);
	cyAxisDoubleSpinBox->setValue(s_center.y);
	czAxisDoubleSpinBox->setValue(s_center.z);

	connect(pickCenterToolButton,	SIGNAL(toggled(bool)),			this, SLOT(pickPointAsCenter(bool)));

	connect(cxAxisDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onCenterChanged(double)));
	connect(cyAxisDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onCenterChanged(double)));
	connect(czAxisDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onCenterChanged(double)));

	connect(normalConfirmToolButton, &QAbstractButton::clicked, this, [this]() {onNormalChanged(0); });
// 	connect(nxDoubleSpinBox,		SIGNAL(valueChanged(double)),	this, SLOT(onNormalChanged(double)));
// 	connect(nyDoubleSpinBox,		SIGNAL(valueChanged(double)),	this, SLOT(onNormalChanged(double)));
// 	connect(nzDoubleSpinBox,		SIGNAL(valueChanged(double)),	this, SLOT(onNormalChanged(double)));

	connect(wDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onDimensionChanged(double)));
	connect(hDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onDimensionChanged(double)));

	connect(previewCheckBox, &QAbstractButton::clicked, this, &bdrPlaneEditorDlg::preview);
	connect(restoreToolButton, &QAbstractButton::clicked, this, &bdrPlaneEditorDlg::restore);
	connect(applyToolButton, &QAbstractButton::clicked, this, &bdrPlaneEditorDlg::saveParamsAndAccept);

	{
		QMenu* displayMenu = new QMenu(displayToolButton);
		displayMenu->addAction(actionDisplayHide);
		displayMenu->addAction(actionDisplayShow);
		displayMenu->addAction(actionDisplayEditor);
		displayToolButton->setMenu(displayMenu);
		
 		connect(actionDisplayHide, &QAction::triggered, this, [this]() {setDisplayState(DISPLAY_NONE); });
 		connect(actionDisplayShow, &QAction::triggered, this, [this]() {setDisplayState(DISPLAY_PLANE); });
 		connect(actionDisplayEditor, &QAction::triggered, this, [this]() {setDisplayState(DISPLAY_EDITOR); });
	}
	
	//auto disable picking mode on quit
	connect(this, &QDialog::finished, [&]()
	{
		if (pickCenterToolButton->isChecked()) pickCenterToolButton->setChecked(false); }
	);
}

bdrPlaneEditorDlg::~bdrPlaneEditorDlg()
{
	assert(!pickCenterToolButton->isChecked());
}

void bdrPlaneEditorDlg::updateParams()
{
	if (m_associatedPlane) {
		updatePlane(m_associatedPlane);
		if (MainWindow::TheInstance())
			MainWindow::TheInstance()->updatePropertiesView();

		m_associatedPlane->getPlane()->redrawDisplay();
	}
}

void bdrPlaneEditorDlg::saveParamsAndAccept()
{
	//save semi-persistent parameters
// 	if (!m_associatedPlane)
// 	{
// 		s_width = wDoubleSpinBox->value();
// 		s_height = hDoubleSpinBox->value();
// 		s_center.x = cxAxisDoubleSpinBox->value();
// 		s_center.y = cyAxisDoubleSpinBox->value();
// 		s_center.z = czAxisDoubleSpinBox->value();
// 	}

	//edition mode
	if (m_associatedPlane)
	{
		updateParams();
		if (!m_planePara.already_in_db) {
			m_planePara.already_in_db = true;
		}
	}
	else //creation
	{
		MainWindow* win = MainWindow::TheInstance();
		if (!win) { return; }
		ccPlane* plane = new ccPlane();
		updatePlane(plane);
		if (m_pickingWin)
		{
			plane->setDisplay(m_pickingWin);
		}
		else {
			plane->setDisplay(win->getActiveGLWindow());
		}
		
		win->addToDB(plane, win->getCurrentDB());
	}
}

void bdrPlaneEditorDlg::setDisplayState(DisplayState state)
{
	m_display_state = state;

	switch (state)
	{
	case bdrPlaneEditorDlg::DISPLAY_NONE:
		displayToolButton->setIcon(actionDisplayHide->icon());
		if (m_associatedPlane && m_associatedPlane->getPlane()) {
			m_associatedPlane->getPlane()->setVisible(false);
			m_associatedPlane->showNormalVector(false);
		}
		break;
	case bdrPlaneEditorDlg::DISPLAY_PLANE:
		displayToolButton->setIcon(actionDisplayShow->icon());
		if (m_associatedPlane && m_associatedPlane->getPlane()) {
			m_associatedPlane->getPlane()->setVisible(true);
			m_associatedPlane->showNormalVector(false);
		}
		break;
	case bdrPlaneEditorDlg::DISPLAY_EDITOR:
		displayToolButton->setIcon(actionDisplayEditor->icon());
		if (m_associatedPlane && m_associatedPlane->getPlane()) {
			m_associatedPlane->getPlane()->setVisible(true);
			m_associatedPlane->showNormalVector(true);
		}
		break;
	default:
		break;
	}
	if (m_associatedPlane && m_associatedPlane->getPlane()) {
		m_associatedPlane->getPlane()->redrawDisplay();
	}
}

void bdrPlaneEditorDlg::onCenterChanged(double)
{
	if (previewCheckBox->isChecked()) {
		updateParams();
	}
}

void bdrPlaneEditorDlg::onNormalChanged(double)
{
	CCVector3 Nd = getNormal();
	Nd.normalize();
	if (Nd.norm() < 1e-6) {
		return;
	}
	setNormal(Nd);

	if (previewCheckBox->isChecked()) {
		updateParams();
	}
}

void bdrPlaneEditorDlg::onDimensionChanged(double)
{
	if (previewCheckBox->isChecked()) {
		updateParams();
	}
}

void bdrPlaneEditorDlg::preview()
{
	if (previewCheckBox->isChecked()) {
		updateParams();
	}
	else {
		restore();
	}
}

void bdrPlaneEditorDlg::restore()
{
	if (!m_associatedPlane) {
		return;
	}
	if (m_associatedPlane && !m_planePara.already_in_db) {
		MainWindow::TheInstance()->removeFromDB(m_associatedPlane->getPlane());
		return;
	}

	CCVector3 Nd = m_planePara.normal;
	CCVector3 Cd = m_planePara.center;
	PointCoordinateType width = m_planePara.size.x;
	PointCoordinateType height = m_planePara.size.y;

	double dip = 0;
	double dipDir = 0;

	CCVector3 N = m_associatedPlane->getNormal();
	CCVector3 C = m_associatedPlane->getCenter();

	//shall we transform (translate and / or rotate) the plane?
	ccGLMatrix trans;
	bool needToApplyTrans = false;
	bool needToApplyRot = false;

	needToApplyRot = (fabs(N.dot(Nd) - PC_ONE) > std::numeric_limits<PointCoordinateType>::epsilon());
	needToApplyTrans = needToApplyRot || ((C - Cd).norm2d() != 0);

	if (needToApplyTrans)
	{
		trans.setTranslation(-C);
		needToApplyTrans = true;
	}
	if (needToApplyRot)
	{
		ccGLMatrix rotation;
		//special case: plane parallel to XY
		if (fabs(N.z) > PC_ONE - std::numeric_limits<PointCoordinateType>::epsilon())
		{
			ccGLMatrix rotX; rotX.initFromParameters(-dip * CC_DEG_TO_RAD, CCVector3(1, 0, 0), CCVector3(0, 0, 0)); //plunge
			ccGLMatrix rotZ; rotZ.initFromParameters(dipDir * CC_DEG_TO_RAD, CCVector3(0, 0, -1), CCVector3(0, 0, 0));
			rotation = rotZ * rotX;
		}
		else //general case
		{
			rotation = ccGLMatrix::FromToRotation(N, Nd);
		}
		trans = rotation * trans;
	}
	if (needToApplyTrans)
	{
		trans.setTranslation(trans.getTranslationAsVec3D() + Cd);
	}
	if (needToApplyRot || needToApplyTrans)
	{
		m_associatedPlane->getPlane()->applyGLTransformation_recursive(&trans);

		ccLog::Print("[Plane edit] Applied transformation matrix:");
		ccLog::Print(trans.toString(12, ' ')); //full precision
	}

	if (m_associatedPlane->getPlane()->isA(CC_TYPES::PLANE)) {
		ccPlane* plane = static_cast<ccPlane*>(m_associatedPlane);
		if (plane->getXWidth() != width
			|| plane->getYWidth() != height) {
			plane->setXWidth(width, false);
			plane->setYWidth(height, true);
		}
	}

	setNormal(m_planePara.normal);
	
	wDoubleSpinBox->setValue(m_planePara.size.x);
	hDoubleSpinBox->setValue(m_planePara.size.y);
	
	setCenter(m_planePara.center);

	if (MainWindow::TheInstance())
		MainWindow::TheInstance()->updatePropertiesView();
	m_associatedPlane->getPlane()->redrawDisplay();
}

void bdrPlaneEditorDlg::cancle()
{
	//restore();
	disconnectPlane();
	
	//deleteLater();
}

void bdrPlaneEditorDlg::pickPointAsCenter(bool state)
{
	if (!m_pickingHub)
	{
		return;
	}
	if (state)
	{
		if (!m_pickingHub->addListener(this, true))
		{
			ccLog::Error("Can't start the picking process (another tool is using it)");
			state = false;
		}
	}
	else
	{
		m_pickingHub->removeListener(this);
	}

	pickCenterToolButton->blockSignals(true);
	pickCenterToolButton->setChecked(state);
	pickCenterToolButton->blockSignals(false);
}

void bdrPlaneEditorDlg::onItemPicked(const PickedItem& pi)
{
	if (!m_pickingHub) {
		return;
	}

	if (!pi.entity)
	{
		return;
	}

	m_pickingWin = m_pickingHub->activeWindow();

	setCenter(pi.P3D);

	pickCenterToolButton->setChecked(false);

	if (previewCheckBox->isChecked()) {
		updateParams();
	}

	if (!m_associatedPlane) {
		if (pi.entity->isKindOf(CC_TYPES::MESH)) {
			ccGenericMesh* block = ccHObjectCaster::ToGenericMesh(pi.entity);
			ccHObject* footprint = block->getParent();
			CCVector3 Na, Nb, Nc;
			block->getTriangleNormals(pi.itemIndex, Na, Nb, Nc);
			CCVector3 N = (Na + Nb + Nc) / 3;
			N.normalize();

			setNormal(N);
			onNormalChanged(0);
			
			MainWindow* main_window = MainWindow::TheInstance();
			if (!main_window) { return; }
			//! create a plane for footprint //FIXME: Maybe we should create a new footprint for this new canvas
			ccPlane* new_plane = new ccPlane;
			updatePlane(new_plane);
			StBlock* new_block = new StBlock(new_plane, nullptr, nullptr);
			int block_number = footprint ? (GetMaxNumberExcludeChildPrefix(footprint, BDDB_BLOCK_PREFIX) + 1) : 0;
			new_block->setName(BDDB_BLOCK_PREFIX + QString::number(block_number));
			new_block->setDisplay_recursive(block->getDisplay());
			new_plane->setDisplay_recursive(block->getDisplay());
			if (footprint) { footprint->addChild(new_block); }

			
			main_window->addToDB(new_block, block->getDBSourceType());
			main_window->unselectAllInDB();
			main_window->setSelectedInDB(new_block, true);
			
			initWithPlane(new_block);
			m_planePara.already_in_db = false; // this is created inside this dialog, if restore, delete it!
		}
	}
}

void bdrPlaneEditorDlg::setNormal(CCVector3 n)
{
	nxDoubleSpinBox->blockSignals(true);
	nyDoubleSpinBox->blockSignals(true);
	nzDoubleSpinBox->blockSignals(true);

	nxDoubleSpinBox->setValue(n.x);
	nyDoubleSpinBox->setValue(n.y);
	nzDoubleSpinBox->setValue(n.z);

	nxDoubleSpinBox->blockSignals(false);
	nyDoubleSpinBox->blockSignals(false);
	nzDoubleSpinBox->blockSignals(false);
}

CCVector3 bdrPlaneEditorDlg::getNormal() const
{
	return { 
		static_cast<PointCoordinateType>(nxDoubleSpinBox->value()),
		static_cast<PointCoordinateType>(nyDoubleSpinBox->value()),
		static_cast<PointCoordinateType>(nzDoubleSpinBox->value()) };
}

void bdrPlaneEditorDlg::setCenter(CCVector3 c)
{
	cxAxisDoubleSpinBox->blockSignals(true);
	cyAxisDoubleSpinBox->blockSignals(true);
	czAxisDoubleSpinBox->blockSignals(true);

	cxAxisDoubleSpinBox->setValue(c.x);
	cyAxisDoubleSpinBox->setValue(c.y);
	czAxisDoubleSpinBox->setValue(c.z);

	cxAxisDoubleSpinBox->blockSignals(false);
	cyAxisDoubleSpinBox->blockSignals(false);
	czAxisDoubleSpinBox->blockSignals(false);
}

CCVector3 bdrPlaneEditorDlg::getCenter() const
{
	return { 
		static_cast<PointCoordinateType>(cxAxisDoubleSpinBox->value()),
		static_cast<PointCoordinateType>(cyAxisDoubleSpinBox->value()),
		static_cast<PointCoordinateType>(czAxisDoubleSpinBox->value()) };
}

void bdrPlaneEditorDlg::initWithPlane(ccPlanarEntityInterface* plane)
{
	if (!plane) { assert(false); return; }

	if (m_associatedPlane != plane) {
		if (m_associatedPlane) {
			disconnectPlane();
		}
		m_associatedPlane = plane;
		m_associatedPlane->normalEditState(true);
		connect(m_associatedPlane, SIGNAL(planarEntityChanged()), this, SLOT(updateUI()));
		m_planePara.already_in_db = true;
	}
	
	CCVector3 N = plane->getNormal();

	//init the dialog

	setNormal(N);

	//onNormalChanged(0);

	//PointCoordinateType dip = 0, dipDir = 0;
	//ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);

	//dipDoubleSpinBox->setValue(dip);
	//dipDirDoubleSpinBox->setValue(dipDir);
	//upwardCheckBox->setChecked(N.z >= 0);
		
	ccPlane* plane_ = getMainPlaneFromInterface(plane);
	if (plane_) {
		dimFrame->setVisible(true);
		wDoubleSpinBox->setValue(plane_->getXWidth());
		hDoubleSpinBox->setValue(plane_->getYWidth());
		m_planePara.size = CCVector2(plane_->getXWidth(), plane_->getYWidth());
	}
	else {
		dimFrame->setVisible(false);
	}	
	
	CCVector3 C = plane->getCenter();
	setCenter(C);

	m_planePara.normal = N;
	m_planePara.center = C;
	//m_planePara.size = CCVector2(plane->getXWidth(), plane->getYWidth());

	setDisplayState(m_display_state);
}

void bdrPlaneEditorDlg::disconnectPlane()
{
	if (m_associatedPlane) {
		m_associatedPlane->normalEditState(false);	// restore old plane edit state
		disconnect(m_associatedPlane, SIGNAL(planarEntityChanged), this, SLOT(updateUI));
		m_associatedPlane = nullptr;
	}
	m_planePara.reset();
}

void bdrPlaneEditorDlg::updateUI()
{
	if (m_associatedPlane) {
		initWithPlane(m_associatedPlane);

		if (MainWindow::TheInstance())
			MainWindow::TheInstance()->updatePropertiesView();
	}
}

void bdrPlaneEditorDlg::closeEvent(QCloseEvent *)
{
	disconnectPlane();
}

void bdrPlaneEditorDlg::updatePlane(ccPlanarEntityInterface* plane)
{
	if (!plane)
	{
		assert(false);
		return;
	}
	
	PointCoordinateType width  = static_cast<PointCoordinateType>(wDoubleSpinBox->value());
	PointCoordinateType height = static_cast<PointCoordinateType>(hDoubleSpinBox->value());
	CCVector3 Nd = getNormal();
	CCVector3 Cd = getCenter();
	
	CCVector3 N = plane->getNormal();
	CCVector3 C = plane->getCenter();

	//shall we transform (translate and / or rotate) the plane?
	ccGLMatrix trans;
	bool needToApplyTrans = false;
	bool needToApplyRot = false;

	needToApplyRot = (fabs(N.dot(Nd) - PC_ONE) > 1e-6/*std::numeric_limits<PointCoordinateType>::epsilon()*/);
	needToApplyTrans = needToApplyRot || ((C - Cd).norm2d() != 0);

	if (needToApplyTrans)
	{
		trans.setTranslation(-C);
	}
	if (needToApplyRot)
	{
		ccGLMatrix rotation;
		//special case: plane parallel to XY
		if (fabs(N.z) > PC_ONE - std::numeric_limits<PointCoordinateType>::epsilon())
		{
			PointCoordinateType dip, dipDir;
			ccNormalVectors::ConvertNormalToDipAndDipDir(Nd, dip, dipDir);
			ccGLMatrix rotX; rotX.initFromParameters(-dip * CC_DEG_TO_RAD, CCVector3(1, 0, 0), CCVector3(0, 0, 0)); //plunge
			ccGLMatrix rotZ; rotZ.initFromParameters(dipDir * CC_DEG_TO_RAD, CCVector3(0, 0, -1), CCVector3(0, 0, 0));
			rotation = rotZ * rotX;
		}
		else //general case
		{
			rotation = ccGLMatrix::FromToRotation(N, Nd);
		}
		trans = rotation * trans;
	}
	if (needToApplyTrans)
	{
		trans.setTranslation(trans.getTranslationAsVec3D() + Cd);
	}
	
	if (needToApplyRot || needToApplyTrans)
	{
		//plane->getPlane()->applyGLTransformation_recursive(&trans);
		plane->applyPlanarEntityChange(trans);

		ccLog::Print("[Plane edit] Applied transformation matrix:");
		ccLog::Print(trans.toString(12, ' ')); //full precision
	}

	ccPlane* plane_ = getMainPlaneFromInterface(plane);
	if (plane_) {
		if (plane_->getXWidth() != width
			|| plane_->getYWidth() != height)
		{
			plane_->setXWidth(width, false);
			plane_->setYWidth(height, true);
		}
	}
}
