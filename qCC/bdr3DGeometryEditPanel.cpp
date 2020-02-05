#include "bdr3DGeometryEditPanel.h"

//Local
#include "mainwindow.h"
#include "ccItemSelectionDlg.h"

//CCLib
#include <ManualSegmentationTools.h>
#include <SquareMatrix.h>

//qCC_db
#include <ccLog.h>
#include <ccPolyline.h>
#include <ccGenericPointCloud.h>
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccHObjectCaster.h>
#include <cc2DViewportObject.h>
#include <ccScalarField.h>
#include <ccColorScalesManager.h>
#include "ccPickingHub.h"
#include "bdrPlaneEditorDlg.h"

#include "StBlock.h"
#include "ccBox.h"
#include "ccSphere.h"
#include "ccCone.h"
#include "ccCylinder.h"
#include "ccTorus.h"
#include "ccDish.h"
#include "ccPlane.h"

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>

#include "stocker_parser.h"

//System
#include <assert.h>

bdr3DGeometryEditPanel::bdr3DGeometryEditPanel(QWidget* parent, ccPickingHub* pickingHub)
	: ccOverlayDialog(parent, Qt::Tool | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint)
	, m_UI(new Ui::bdr3DGeometryEditPanel )
	, m_pickingHub(pickingHub)
	, m_somethingHasChanged(false)
	, m_state(0)
	, m_segmentationPoly(0)
	, m_polyVertices(0)
	, m_rectangularSelection(false)
	, m_deleteHiddenParts(false)
	, m_destination(nullptr)
	, m_current_editor(GEO_END)
	, m_refPlanePanel(nullptr)
	, m_toolPlanePanel(nullptr)
	, m_refPlane(nullptr)
{
	m_UI->setupUi(this);

	m_UI->blockToolButton->installEventFilter(this);
	m_UI->boxToolButton->installEventFilter(this);
	m_UI->sphereToolButton->installEventFilter(this);
	m_UI->coneToolButton->installEventFilter(this);
	m_UI->cylinderToolButton->installEventFilter(this);
	m_UI->parapetToolButton->installEventFilter(this);
	m_UI->toroidCylinderToolButton->installEventFilter(this);
	m_UI->torusToolButton->installEventFilter(this);
	m_UI->dishToolButton->installEventFilter(this);
	m_UI->planeToolButton->installEventFilter(this);
	m_UI->polygonToolButton->installEventFilter(this);
	m_UI->polylineToolButton->installEventFilter(this);
	m_UI->wallToolButton->installEventFilter(this);

	connect(m_UI->editToolButton,				&QToolButton::toggled, this, &bdr3DGeometryEditPanel::startEditingMode);
	connect(m_UI->confirmToolButton,			&QToolButton::clicked, this, &bdr3DGeometryEditPanel::confirmCreate);
	connect(m_UI->blockToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doBlock);
	connect(m_UI->boxToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doBox);
	connect(m_UI->sphereToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doSphere);
	connect(m_UI->coneToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doCone);
	connect(m_UI->cylinderToolButton,			&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doCylinder);
	connect(m_UI->parapetToolButton,			&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doParapet);
	connect(m_UI->toroidCylinderToolButton,		&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doToroidCylinder); 
	connect(m_UI->torusToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doTorus);
	connect(m_UI->dishToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doDish);
	connect(m_UI->planeToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doPlane);
	connect(m_UI->polylineToolButton,			&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doPolyline);
	connect(m_UI->wallToolButton,				&QToolButton::clicked, this, &bdr3DGeometryEditPanel::doWall);
	
	connect(m_UI->razButton,					&QToolButton::clicked, this, &bdr3DGeometryEditPanel::reset);
	connect(m_UI->exitButton,					&QToolButton::clicked, this, &bdr3DGeometryEditPanel::exit);

	connect(m_UI->freeMeshToolButton,			&QToolButton::clicked, this, &bdr3DGeometryEditPanel::makeFreeMesh);

	connect(m_UI->currentModelComboBox,			SIGNAL(currentIndexChanged(QString)), this, SLOT(onCurrentModelChanged(QString)));

	connect(m_UI->planeBasedViewToolButton, &QToolButton::clicked, this, &bdr3DGeometryEditPanel::planeBasedView);
	connect(m_UI->objectBasedViewToolButton, &QToolButton::clicked, this, &bdr3DGeometryEditPanel::objectBasedView);

	m_UI->geometryTabWidget->tabBar()->hide();
	
 	//add shortcuts
 	addOverridenShortcut(Qt::Key_Space);  //space bar for the "pause" button
 	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
 	addOverridenShortcut(Qt::Key_Return); //return key for the "apply" button
 	addOverridenShortcut(Qt::Key_Delete); //delete key for the "apply and delete" button
 	addOverridenShortcut(Qt::Key_Tab);    //tab key to switch between EDITING mode
 	addOverridenShortcut(Qt::Key_C);      // CONFIRM
 	addOverridenShortcut(Qt::Key_I);      // IMAGE VIEW
	addOverridenShortcut(Qt::Key_E);	  // EIDITING
 	connect(this,								&ccOverlayDialog::shortcutTriggered, this, &bdr3DGeometryEditPanel::onShortcutTriggered);

	echoUIchange();

	createPlaneEditInterface();

//  QMenu* selectionModeMenu = new QMenu(this);
//  selectionModeMenu->addAction(m_UI->action2DSelection);
//  selectionModeMenu->addAction(m_UI->action3DSelection);
// 	m_UI->selectionModeButton->setDefaultAction(m_UI->action3DSelection);
// 	m_UI->selectionModeButton->setMenu(selectionModeMenu);
 
// 	QMenu* importExportMenu = new QMenu(this);
// 	importExportMenu->addAction(actionUseExistingPolyline);
// 	importExportMenu->addAction(actionExportSegmentationPolyline);
// 	loadSaveToolButton->setMenu(importExportMenu);
// 

	m_UI->editGroupBox->setVisible(false);

 	m_polyVertices = new ccPointCloud("vertices");
 	m_segmentationPoly = new ccPolyline(m_polyVertices);
 	m_segmentationPoly->setForeground(true);
 	m_segmentationPoly->setColor(ccColor::green);
 	m_segmentationPoly->showColors(true);
 	m_segmentationPoly->set2DMode(true);
 	allowExecutePolyline(false);

	m_selection_mode = SELECT_3D;
}

bdr3DGeometryEditPanel::~bdr3DGeometryEditPanel()
{
	if (m_segmentationPoly)
		delete m_segmentationPoly;
	m_segmentationPoly = 0;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = 0;
}

void bdr3DGeometryEditPanel::echoUIchange()
{
//block
	connect(m_UI->blockTopDoubleSpinBox,	SIGNAL(valueChanged(double)), this, SLOT(echoBlockTopHeight(double)));
	connect(m_UI->blockBottomDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(echoBlockBottomHeight(double)));
}

void bdr3DGeometryEditPanel::createPlaneEditInterface()
{
	m_refPlanePanel = new bdrPlaneEditorDlg(m_pickingHub, this);
	m_refPlanePanel->confirmGroupBox->setVisible(false);
	m_refPlanePanel->setDisplayState(bdrPlaneEditorDlg::DISPLAY_PLANE);
	m_UI->refPlaneHorizontalLayout->addWidget(m_refPlanePanel);

	m_refPlane = new ccPlane("master");
	m_refPlanePanel->updatePlane(m_refPlane);
	m_refPlane->setColor(ccColor::darkGrey);
	m_refPlane->showColors(true);
	m_refPlane->notifyColorUpdate();
	m_refPlane->enableStippling(true);
	m_refPlanePanel->initWithPlane(m_refPlane);

	m_toolPlanePanel = new bdrPlaneEditorDlg(m_pickingHub, this);
	m_toolPlanePanel->setDisplayState(bdrPlaneEditorDlg::DISPLAY_EDITOR);
	m_UI->toolPlaneHorizontalLayout->addWidget(m_toolPlanePanel);
}

bool bdr3DGeometryEditPanel::eventFilter(QObject * obj, QEvent * e)
{
	if (e->type() == QEvent::Close) {
		//exit();
	}
	else if (e->type() == QEvent::HoverEnter) {
		if (obj == m_UI->blockToolButton)				{ m_UI->geometryTabWidget->setCurrentIndex(GEO_BLOCK); }
		else if (obj == m_UI->boxToolButton)			{ m_UI->geometryTabWidget->setCurrentIndex(GEO_BOX); }
		else if (obj == m_UI->sphereToolButton)			{ m_UI->geometryTabWidget->setCurrentIndex(GEO_SPHERE); }
		else if (obj == m_UI->coneToolButton)			{ m_UI->geometryTabWidget->setCurrentIndex(GEO_CONE); }
		else if (obj == m_UI->cylinderToolButton)		{ m_UI->geometryTabWidget->setCurrentIndex(GEO_CYLINDER); }
		else if (obj == m_UI->parapetToolButton)		{ m_UI->geometryTabWidget->setCurrentIndex(GEO_PARAPET); }
		else if (obj == m_UI->toroidCylinderToolButton) { m_UI->geometryTabWidget->setCurrentIndex(GEO_TORCYLINDER); }
		else if (obj == m_UI->torusToolButton)			{ m_UI->geometryTabWidget->setCurrentIndex(GEO_TORUS); }
		else if (obj == m_UI->dishToolButton)			{ m_UI->geometryTabWidget->setCurrentIndex(GEO_DISH); }
		else if (obj == m_UI->planeToolButton)			{ m_UI->geometryTabWidget->setCurrentIndex(GEO_PLANE); }
		else if (obj == m_UI->polygonToolButton)		{ m_UI->geometryTabWidget->setCurrentIndex(GEO_POLYGON); }
		else if (obj == m_UI->polylineToolButton)		{ m_UI->geometryTabWidget->setCurrentIndex(GEO_POLYLINE); }
		else if (obj == m_UI->wallToolButton)			{ m_UI->geometryTabWidget->setCurrentIndex(GEO_WALL); }
	}
	else if (e->type() == QEvent::Leave) {
		if (m_current_editor < GEO_END) {
			m_UI->geometryTabWidget->setCurrentIndex(m_current_editor);
		}
	}
	return ccOverlayDialog::eventFilter(obj, e);
}

QToolButton* bdr3DGeometryEditPanel::getGeoToolBottun(GEOMETRY3D g)
{
	switch (g)
	{
	case GEO_BLOCK:
		return m_UI->blockToolButton;
	case GEO_BOX:
		return m_UI->boxToolButton;
	case GEO_SPHERE:
		return m_UI->sphereToolButton;
	case GEO_CONE:
		return m_UI->coneToolButton;
	case GEO_CYLINDER:
		return m_UI->cylinderToolButton;
	case GEO_PARAPET:
		return m_UI->parapetToolButton;
	case GEO_TORCYLINDER:
		return m_UI->toroidCylinderToolButton;
	case GEO_TORUS:
		return m_UI->torusToolButton;
	case GEO_DISH:
		return m_UI->dishToolButton;
	case GEO_PLANE:
		return m_UI->planeToolButton;
	case GEO_POLYGON:
		return m_UI->polygonToolButton;
	case GEO_POLYLINE:
		return m_UI->polylineToolButton;
	case GEO_WALL:			
		return m_UI->wallToolButton;
	default:
		break;
	}
	return nullptr;
}

void bdr3DGeometryEditPanel::allowExecutePolyline(bool state)
{
 	if (state) {
		m_UI->confirmToolButton->setEnabled(true);
 	}
 	else {
		m_UI->confirmToolButton->setEnabled(false);
 	}
}

void bdr3DGeometryEditPanel::allowStateChange(bool state)
{

}

void bdr3DGeometryEditPanel::updateWithActive(ccHObject * obj)
{
	m_UI->geometryTabWidget->blockSignals(true);
	if (obj->isA(CC_TYPES::ST_BLOCK)) {
		StBlock* block = ccHObjectCaster::ToStBlock(obj);
		if (block) {
			m_UI->blockTopDoubleSpinBox->setValue(block->getTopHeight());
			m_UI->blockBottomDoubleSpinBox->setValue(block->getBottomHeight());
			m_UI->geometryTabWidget->setCurrentIndex(GEO_BLOCK);
		}
	}
	else if (obj->isA(CC_TYPES::BOX)) {
		ccBox* box = ccHObjectCaster::ToBox(obj);
		if (box) {
			CCVector3 dim = box->getDimensions();
			m_UI->boxDxDoubleSpinBox->setValue(dim.x);
			m_UI->boxDyDoubleSpinBox->setValue(dim.y);
			m_UI->boxDzDoubleSpinBox->setValue(dim.z);

			m_UI->geometryTabWidget->setCurrentIndex(GEO_BOX);
		}
	}
	else if (obj->isA(CC_TYPES::SPHERE)) {
		ccSphere* sphere = ccHObjectCaster::ToSphere(obj);
		if (sphere)	{
			m_UI->sphereRadiusDoubleSpinBox->setValue(sphere->getRadius());
			m_UI->sphereDimensionSpinBox->setValue(sphere->getDrawingPrecision());
			
			m_UI->geometryTabWidget->setCurrentIndex(GEO_SPHERE);
		}
	}
	else if (obj->isA(CC_TYPES::CONE)) {
		ccCone* cone = ccHObjectCaster::ToCone(obj);
		if (cone) {
			m_UI->coneTopRadiusDoubleSpinBox->setValue(cone->getTopRadius());
			m_UI->coneBottomRadiusDoubleSpinBox->setValue(cone->getBottomRadius());
			m_UI->coneHeightDoubleSpinBox->setValue(cone->getHeight());
			PointCoordinateType snoutx, snouty;
			cone->getSnout(snoutx, snouty);
			m_UI->coneXOffsetDoubleSpinBox->setValue(snoutx);
			m_UI->coneYOffsetDoubleSpinBox->setValue(snouty);
			if (cone->hasMetaData(GEO_ROUND_FLAG) && cone->getMetaData(GEO_ROUND_FLAG).toBool()) {
				m_UI->conePrecisionRadioButton->setChecked(true);
				m_UI->conePrecisionSpinBox->setValue(cone->getDrawingPrecision());
			}
			else {
				m_UI->coneEdgeDimRadioButton->setChecked(true);
				m_UI->coneEdgeDimSpinBox->setValue(cone->getDrawingPrecision());
			}
			m_UI->geometryTabWidget->setCurrentIndex(GEO_CONE);
		}
	}
	else if (obj->isA(CC_TYPES::CYLINDER)) {
		ccCylinder* cylinder = ccHObjectCaster::ToCylinder(obj);
		if (cylinder) {
			m_UI->cylinderRadiusDoubleSpinBox->setValue(cylinder->getTopRadius());
			m_UI->cylinderHeightDoubleSpinBox->setValue(cylinder->getHeight());
			if (cylinder->hasMetaData(GEO_ROUND_FLAG) && cylinder->getMetaData(GEO_ROUND_FLAG).toBool()) {
				m_UI->cylinderPrecisionRadioButton->setChecked(true);
				m_UI->cylinderPrecisionSpinBox->setValue(cylinder->getDrawingPrecision());
			}
			else {
				m_UI->cylinderEdgeDimRadioButton->setChecked(true);
				m_UI->cylinderEdgeDimSpinBox->setValue(cylinder->getDrawingPrecision());
			}
			m_UI->geometryTabWidget->setCurrentIndex(GEO_CYLINDER);
		}
	}
	else if (obj->isA(CC_TYPES::TORUS)) {
		ccTorus* torus = ccHObjectCaster::ToTorus(obj);
		if (torus) {
			if (torus->isCylinder()) {
				m_UI->toroidCylinderInsideRadiusDoubleSpinBox->setValue(torus->getInsideRadius());
				m_UI->toroidCylinderOutsideRadiusDoubleSpinBox->setValue(torus->getOutsideRadius());
				m_UI->toroidCylinderAngleDoubleSpinBox->setValue(torus->getAngle());
				m_UI->toroidCylinderRectSectionHeightDoubleSpinBox->setValue(torus->getCylinderHeight());
				m_UI->geometryTabWidget->setCurrentIndex(GEO_TORCYLINDER);
			}
			else {
				m_UI->torusInsideRadiusDoubleSpinBox->setValue(torus->getInsideRadius());
				m_UI->torusOutsideRadiusDoubleSpinBox->setValue(torus->getOutsideRadius());
				m_UI->torusAngleDoubleSpinBox->setValue(torus->getAngle());
				m_UI->geometryTabWidget->setCurrentIndex(GEO_TORUS);
			}
		}
	}
	else if (obj->isA(CC_TYPES::DISH)) {
		ccDish* dish = ccHObjectCaster::ToDish(obj);
		if (dish) {
			PointCoordinateType r1, r2;
			dish->getRadius(r1, r2);
			m_UI->dishRadiusDoubleSpinBox->setValue(r1);
			m_UI->dishRadius2DoubleSpinBox->setValue(r2);
			m_UI->dishHeightDoubleSpinBox->setValue(dish->getHeight());
			m_UI->geometryTabWidget->setCurrentIndex(GEO_DISH);
		}
	}
	else if (obj->isA(CC_TYPES::PLANE)) {
		ccPlane* plane = ccHObjectCaster::ToPlane(obj);
		if (plane && plane != m_refPlane) {
			m_toolPlanePanel->initWithPlane(plane);
			m_UI->geometryTabWidget->setCurrentIndex(GEO_PLANE);
		}
	}
	else if (obj->isA(CC_TYPES::POLY_LINE)) {

		m_UI->geometryTabWidget->setCurrentIndex(GEO_POLYLINE);
	}
	m_UI->geometryTabWidget->blockSignals(false);
}

void bdr3DGeometryEditPanel::onShortcutTriggered(int key)
{
 	switch(key)
	{
	case Qt::Key_Space:
		m_UI->editToolButton->toggle();
		return;

	case Qt::Key_I:
		MainWindow::TheInstance()->showBestImage(false);
		return;

	case Qt::Key_O:
		//outButton->click();
		return;

	case Qt::Key_E:
		m_UI->editToolButton->toggle();
		return;

	case Qt::Key_Return:
		//validButton->click();
		return;
	case Qt::Key_Delete:
		//validAndDeleteButton->click();
		return;
	case Qt::Key_Escape:
		return;

	case Qt::Key_Tab:
		
		return;

	default:
		//nothing to do
		break;
	}
}

bool bdr3DGeometryEditPanel::linkWith(ccGLWindow* win)
{
	assert(m_segmentationPoly);

	ccGLWindow* oldWin = m_associatedWin;

	if (!ccOverlayDialog::linkWith(win))
	{
		return false;
	}

	if (oldWin)
	{
		oldWin->disconnect(this);
		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(0);
		}
		if (m_refPlane) {
			m_refPlane->setDisplay(0);
		}
	}
	
	if (m_associatedWin)
	{
		connect(m_associatedWin, &ccGLWindow::leftButtonClicked,	this, &bdr3DGeometryEditPanel::addPointToPolyline);
		connect(m_associatedWin, &ccGLWindow::rightButtonClicked,	this, &bdr3DGeometryEditPanel::closePolyLine);
		connect(m_associatedWin, &ccGLWindow::mouseMoved,			this, &bdr3DGeometryEditPanel::updatePolyLine);
		connect(m_associatedWin, &ccGLWindow::buttonReleased,		this, &bdr3DGeometryEditPanel::closeRectangle);
		connect(m_associatedWin, &ccGLWindow::entitySelectionChanged, this, &bdr3DGeometryEditPanel::echoSelectChange);

		if (m_segmentationPoly)	{
			m_segmentationPoly->setDisplay(m_associatedWin);
		}
		if (m_refPlane)	{
			m_refPlane->setDisplay(m_associatedWin);
		}
	}

	return true;
}

bool bdr3DGeometryEditPanel::start()
{
	assert(m_polyVertices && m_segmentationPoly && m_refPlane);

	if (!m_associatedWin)
	{
		return false;
	}
	int i(0);
	for (ccHObject* entity : m_ModelObjs) {
		if (entity->isA(CC_TYPES::ST_BUILDING)) {
			m_UI->currentModelComboBox->addItem(entity->getName());
			m_UI->currentModelComboBox->setItemIcon(i, QIcon(QStringLiteral(":/CC/Stocker/images/stocker/building.png")));
		}
		else
			continue;
		++i;
	}
	m_UI->currentModelComboBox->setCurrentIndex(-1);

	m_segmentationPoly->clear();
	m_polyVertices->clear();
	allowExecutePolyline(false);

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->addToOwnDB(m_segmentationPoly);
	m_associatedWin->addToOwnDB(m_refPlane);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);

	if (!m_ModelObjs.empty()) m_UI->currentModelComboBox->setCurrentIndex(0);

	MainWindow*win = MainWindow::TheInstance();
	if (win->getRoot(CC_TYPES::DB_BUILDING)->getChildrenNumber() == 0) {
		m_associatedWin->zoomGlobal();
	}
	
	startEditingMode(false);

	m_somethingHasChanged = false;

	reset();

	return ccOverlayDialog::start();
}

void bdr3DGeometryEditPanel::removeAllEntities(bool unallocateVisibilityArrays)
{
	for (QSet<ccHObject*>::const_iterator p = m_ModelObjs.constBegin(); p != m_ModelObjs.constEnd(); ++p)
	{
		
	}
	
	m_ModelObjs.clear();
	m_UI->currentModelComboBox->clear();
}

void bdr3DGeometryEditPanel::setActiveItem(std::vector<ccHObject*> active)
{
	if (m_state & RUNNING) {
		return;
	}
	m_actives = active;

	if (m_actives.size() == 1 && m_actives.front()) {
		updateWithActive(m_actives.front());
	}
}

void bdr3DGeometryEditPanel::setModelObjects(std::vector<ccHObject*> builds)
{
	if (!m_ModelObjs.empty()) {
		removeAllEntities(true);
	}
	m_ModelObjs.clear();

	for (ccHObject* entity : builds) {
		bool name_exist = false;
		for (ccHObject* e : m_ModelObjs) {
			if (e->getName() == entity->getName()) {
				name_exist = true;
				break;
			}
		}
		if (name_exist) {
			continue;
		}
		
		if (entity->isA(CC_TYPES::ST_BUILDING))	{
			
		}
		else {
			continue;
		}

		m_ModelObjs.insert(entity);
	}
}

void bdr3DGeometryEditPanel::stop(bool accepted)
{
	assert(m_segmentationPoly);

	if (m_associatedWin) {
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setPerspectiveState(true, true);
		m_associatedWin->lockRotationAxis(false, CCVector3d(0, 0, 1));
		m_associatedWin->setUnclosable(false);
		m_associatedWin->removeFromOwnDB(m_segmentationPoly);
		if (m_refPlane) m_associatedWin->removeFromOwnDB(m_refPlane);
	}

	ccOverlayDialog::stop(accepted);
}

void bdr3DGeometryEditPanel::reset()
{
	if (m_somethingHasChanged)
	{
		for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
		{
			
		}

		if (m_associatedWin)
			m_associatedWin->redraw(false);
		m_somethingHasChanged = false;
	}

 	m_UI->razButton->setEnabled(false);
// 	validButton->setEnabled(false);
// 	validAndDeleteButton->setEnabled(false);
// 	loadSaveToolButton->setDefaultAction(actionUseExistingPolyline);
}

void bdr3DGeometryEditPanel::exit()
{
	if (m_somethingHasChanged) {
		// TODO: ask for reset
		if (0) {
			reset();
			m_deleteHiddenParts = false;
			stop(false);
			return;
		}
	}
	stop(true);
}

void bdr3DGeometryEditPanel::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
{
	//process not started yet?
	if ((m_state & RUNNING) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);

	unsigned vertCount = m_polyVertices->size();

	//new point (expressed relatively to the screen center)
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
				static_cast<PointCoordinateType>(pos2D.y()),
				0);

	if (m_state & RECTANGLE)
	{
		//we need 4 points for the rectangle!
		if (vertCount != 4)
			m_polyVertices->resize(4);

		const CCVector3* A = m_polyVertices->getPointPersistentPtr(0);
		CCVector3* B = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(1));
		CCVector3* C = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(2));
		CCVector3* D = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(3));
		*B = CCVector3(A->x,P.y,0);
		*C = P;
		*D = CCVector3(P.x,A->y,0);

		if (vertCount != 4)
		{
			m_segmentationPoly->clear();
			if (!m_segmentationPoly->addPointIndex(0,4))
			{
				ccLog::Error("Out of memory!");
				allowExecutePolyline(false);
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
	}
	else if (m_state & POLYLINE)
	{
		if (vertCount < 2)
			return;
		//we replace last point by the current one
		CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount-1));
		*lastP = P;
	}

	m_associatedWin->redraw(true, false);
}

void bdr3DGeometryEditPanel::echoSelectChange(ccHObject* obj)
{
	if (m_state & RUNNING) {
		return;
	}
	if (!(QApplication::keyboardModifiers() & Qt::ControlModifier)) {
		m_actives.clear();
	}
	m_actives.push_back(obj);
	setActiveItem(m_actives);
}

void bdr3DGeometryEditPanel::addPointToPolyline(int x, int y)
{
	if (m_selection_mode != SELECT_2D) {
		return;
	}

	if ((m_state & STARTED) == 0)
	{
		return;
	}
	if (!m_associatedWin)
	{
		assert(false);
		return;
	}

	assert(m_polyVertices);
	assert(m_segmentationPoly);
	unsigned vertCount = m_polyVertices->size();

	//particular case: we close the rectangular selection by a 2nd click
	if (m_rectangularSelection && vertCount == 4 && (m_state & RUNNING))
		return;

	//new point
	QPointF pos2D = m_associatedWin->toCenteredGLCoordinates(x, y);
	CCVector3 P(static_cast<PointCoordinateType>(pos2D.x()),
				static_cast<PointCoordinateType>(pos2D.y()),
				0);

	//CTRL key pressed at the same time?
	bool ctrlKeyPressed = m_rectangularSelection || ((QApplication::keyboardModifiers() & Qt::ControlModifier) == Qt::ControlModifier);

	//start new polyline?
	if (((m_state & RUNNING) == 0) || vertCount == 0 || ctrlKeyPressed)
	{
		//reset state
		m_state = (ctrlKeyPressed ? RECTANGLE : POLYLINE);
		m_state |= (STARTED | RUNNING);
		//reset polyline
		m_polyVertices->clear();
		if (!m_polyVertices->reserve(2))
		{
			ccLog::Error("Out of memory!");
			allowExecutePolyline(false);
			return;
		}
		//we add the same point twice (the last point will be used for display only)
		m_polyVertices->addPoint(P);
		m_polyVertices->addPoint(P);
		m_segmentationPoly->clear();
		if (!m_segmentationPoly->addPointIndex(0, 2))
		{
			ccLog::Error("Out of memory!");
			allowExecutePolyline(false);
			return;
		}
	}
	else //next points in "polyline mode" only
	{
		//we were already in 'polyline' mode?
		if (m_state & POLYLINE)
		{
			if (!m_polyVertices->reserve(vertCount+1))
			{
				ccLog::Error("Out of memory!");
				allowExecutePolyline(false);
				return;
			}

			//we replace last point by the current one
			CCVector3* lastP = const_cast<CCVector3*>(m_polyVertices->getPointPersistentPtr(vertCount-1));
			*lastP = P;
			//and add a new (equivalent) one
			m_polyVertices->addPoint(P);
			if (!m_segmentationPoly->addPointIndex(vertCount))
			{
				ccLog::Error("Out of memory!");
				return;
			}
			m_segmentationPoly->setClosed(true);
		}
		else //we must change mode
		{
			assert(false); //we shouldn't fall here?!
			m_state &= (~RUNNING);
			addPointToPolyline(x,y);
			return;
		}
	}

	m_associatedWin->redraw(true, false);
}

void bdr3DGeometryEditPanel::closeRectangle()
{
	//only for rectangle selection in RUNNING mode
	if ((m_state & RECTANGLE) == 0 || (m_state & RUNNING) == 0)
		return;

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		//first point only? we keep the real time update mechanism
		if (m_rectangularSelection)
			return;
		m_segmentationPoly->clear();
		m_polyVertices->clear();
		allowExecutePolyline(false);
	}
	else
	{
		allowExecutePolyline(true);
	}

	//stop
	m_state &= (~RUNNING);

	if (m_associatedWin)
		m_associatedWin->redraw(true, false);
}

void bdr3DGeometryEditPanel::closePolyLine(int, int)
{
	//only for polyline in RUNNING mode
	if ((m_state & POLYLINE) == 0 || (m_state & RUNNING) == 0)
		return;

	if ((QApplication::keyboardModifiers() & Qt::ShiftModifier)) {
		return;
	}

	assert(m_segmentationPoly);
	unsigned vertCount = m_segmentationPoly->size();
	if (vertCount < 4)
	{
		m_segmentationPoly->clear();
		m_polyVertices->clear();
	}
	else
	{
		if (m_current_editor == GEO_BLOCK) {
			//remove last point!
			m_segmentationPoly->resize(vertCount - 1); //can't fail --> smaller
			m_segmentationPoly->setClosed(true);
		}
		else if (m_current_editor == GEO_PARAPET) {
			m_segmentationPoly->setClosed(false);
		}
		
		allowExecutePolyline(true);
	}

	//stop
	m_state &= (~RUNNING);
 	
 	if (m_associatedWin)
 	{
 		m_associatedWin->redraw(true, false);
 	}
}

void bdr3DGeometryEditPanel::startEditingMode(bool state)
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
		return;

	if (!state/*=activate start mode*/) {
		m_state = PAUSED;
		
		if (m_polyVertices->size() != 0)
		{
			m_segmentationPoly->clear();
			m_polyVertices->clear();
		}
		allowExecutePolyline(false);
		allowStateChange(true);
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		MainWindow::TheInstance()->dispToStatus(QString("paused, press space to continue editing"));

		m_UI->editToolButton->setIcon(QIcon(QStringLiteral(":/CC/Stocker/images/stocker/play.png")));
	}
	else {
		m_state = STARTED;
		allowStateChange(false);

		m_associatedWin->setInteractionMode(ccGLWindow::INTERACT_SHIFT_PAN | ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		if (m_selection_mode == SELECT_2D) {
			MainWindow::TheInstance()->dispToStatus(QString("editing (2D), left click to add contour points, right click to close"));
		}
		else if (m_selection_mode == SELECT_3D) {
			MainWindow::TheInstance()->dispToStatus(QString("editing (3D), left click to add contour points, right click to close"));
		}
		m_UI->editToolButton->setIcon(QIcon(QStringLiteral(":/CC/Stocker/images/stocker/pause.png")));
	}

	//update mini-GUI
	m_UI->editToolButton->blockSignals(true);
	m_UI->editToolButton->setChecked(state);
	m_UI->editToolButton->blockSignals(false);

	m_associatedWin->redraw(!state);
}

void bdr3DGeometryEditPanel::confirmCreate()
{
	ccHObject* activeModel = nullptr;
	for (ccHObject* o : m_ModelObjs) {
		if (o->getName() == m_UI->currentModelComboBox->currentText()) {
			activeModel = o;
		}
	}
	if (!activeModel) {
		pauseAll();
		ccLog::Error("no model to create the primitive");
	}

	//viewing parameters
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	std::vector<CCVector3> polygon3D;
	{
		double refPlaneZ = 0;
		CCVector3d Q2D;
		camera.project(m_refPlane->getCenter(), Q2D, false);
		refPlaneZ = Q2D.z;

		polygon3D = m_segmentationPoly->getPoints(false);


	}

	// project the 2d polygon to the ref plane
	if (m_current_editor == GEO_BLOCK) {
		
		ccPlane* mainPlane = new ccPlane();
		StBlock* block = new StBlock(mainPlane, 1, CCVector3(0, 0, 1), 0, CCVector3(0, 0, -1));

		activeModel->addChild(block);
		MainWindow::TheInstance()->addToDB_Build(block);
	}
}

void bdr3DGeometryEditPanel::pauseAll()
{
	startEditingMode(false);
	m_state = PAUSED;
	QToolButton* ct = getGeoToolBottun(m_current_editor);
	if (ct) ct->setChecked(false);
	m_current_editor = GEO_END;
	m_associatedWin->setPerspectiveState(true, true);
}

void bdr3DGeometryEditPanel::startGeoTool(GEOMETRY3D g)
{
	bool same = (m_current_editor == g);
	if (m_current_editor < GEO_END) {
		pauseAll();
	}
	if (!same) {
		m_current_editor = g;
		m_UI->createGroupBox->setEnabled(true);

		if (m_current_editor == GEO_BLOCK || m_current_editor == GEO_PARAPET) {
			if (m_refPlane) {
				m_associatedWin->setPerspectiveState(false, true);
				CCVector3d normal = CCVector3d::fromArray(m_refPlane->getNormal().u);
				CCVector3d vertDir = CCVector3d::fromArray(m_refPlane->getTransformation().getColumnAsVec3D(1).u);
				m_associatedWin->setCustomView(-normal, vertDir);
			}
			m_selection_mode = SELECT_2D;
		}
		else
			m_selection_mode = SELECT_3D;
	}
}

void bdr3DGeometryEditPanel::doBlock()
{
	startGeoTool(GEO_BLOCK);
}

void bdr3DGeometryEditPanel::doBox()
{
	startGeoTool(GEO_BOX);
}

void bdr3DGeometryEditPanel::doSphere()
{
	startGeoTool(GEO_SPHERE);
}

void bdr3DGeometryEditPanel::doCone()
{
	startGeoTool(GEO_CONE);
}

void bdr3DGeometryEditPanel::doCylinder()
{
	startGeoTool(GEO_CYLINDER);
}

void bdr3DGeometryEditPanel::doParapet()
{
	startGeoTool(GEO_PARAPET);
}

void bdr3DGeometryEditPanel::doToroidCylinder()
{
	startGeoTool(GEO_TORCYLINDER);
}

void bdr3DGeometryEditPanel::doTorus()
{
	startGeoTool(GEO_TORUS);
}

void bdr3DGeometryEditPanel::doDish()
{
	startGeoTool(GEO_DISH);
}

void bdr3DGeometryEditPanel::doPlane()
{
	startGeoTool(GEO_PLANE);
}

void bdr3DGeometryEditPanel::doPolyline()
{
	startGeoTool(GEO_BOX);
}

void bdr3DGeometryEditPanel::doWall()
{
	startGeoTool(GEO_WALL);
}

void bdr3DGeometryEditPanel::startEdit()
{
	if (m_actives.size() == 1) {
		ccHObject* active = m_actives.front();
		if (active)	{

		}
	}
	else {

	}
}

void bdr3DGeometryEditPanel::makeFreeMesh()
{
}

void bdr3DGeometryEditPanel::planeBasedView()
{
	if (m_refPlane) {
		m_associatedWin->setPerspectiveState(false, true);
		CCVector3d normal = CCVector3d::fromArray(m_refPlane->getNormal().u);
		CCVector3d vertDir = CCVector3d::fromArray(m_refPlane->getTransformation().getColumnAsVec3D(1).u);
		m_associatedWin->setCustomView(-normal, vertDir);
	}
}

void bdr3DGeometryEditPanel::objectBasedView()
{
	m_associatedWin->setPerspectiveState(true, true);
}

void bdr3DGeometryEditPanel::onCurrentModelChanged(QString name)
{
	if (!m_refPlane || !m_refPlane->isDisplayedIn(m_associatedWin)) {
		return;
	}
	ccBBox box;
	for (ccHObject* obj : m_ModelObjs) {
		if (obj->getName() == name) {
			box = obj->getBB_recursive(false);
			break;
		}
	}
	if (!box.isValid()) {
		for (ccHObject* obj : m_ModelObjs) {
			box += obj->getBB_recursive(false);
		}
	}

	if (box.isValid()) {
		ccGLMatrix trans;
		CCVector3 C = (box.P(0) + box.P(3)) / 2;
		CCVector3 Cd = m_refPlane->getCenter();
		trans.setTranslation(-C);
		ccGLMatrix rotation;
		//special case: plane parallel to XY
		CCVector3 N = m_refPlane->getNormal();
		CCVector3 Nd(0, 0, 1);

		if ((N-Nd).norm2d() > std::numeric_limits<PointCoordinateType>::epsilon()) {
			if (fabs(N.z) > PC_ONE - std::numeric_limits<PointCoordinateType>::epsilon()) {
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

		trans.setTranslation(trans.getTranslationAsVec3D() + Cd);

		m_refPlane->applyGLTransformation_recursive(&trans);
		m_refPlane->setXWidth((box.P(0) - box.P(1)).norm(), false);
		m_refPlane->setYWidth((box.P(0) - box.P(2)).norm(), true);
		m_refPlanePanel->initWithPlane(m_refPlane);
	}
}

//////////////////////////////////////////////////////////////////////////

void bdr3DGeometryEditPanel::echoBlockTopHeight(double v)
{
	//if (m_current_editor >= GEO_END) return;
	for (ccHObject* active : m_actives) {
		StBlock* block = ccHObjectCaster::ToStBlock(active);
		if (block) block->setTopHeight(v);
	}
	m_associatedWin->redraw();
}

void bdr3DGeometryEditPanel::echoBlockBottomHeight(double v)
{
	//if (m_current_editor >= GEO_END) return;
	for (ccHObject* active : m_actives) {
		StBlock* block = ccHObjectCaster::ToStBlock(active);
		if (block) block->setBottomHeight(v);
	}
	m_associatedWin->redraw();
}

//////////////////////////////////////////////////////////////////////////