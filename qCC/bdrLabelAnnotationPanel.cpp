#include "bdrLabelAnnotationPanel.h"

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

//qCC_gl
#include <ccGLWindow.h>

//Qt
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>

#include "stocker_parser.h"

//System
#include <assert.h>

bdrLabelAnnotationPanel::bdrLabelAnnotationPanel(QWidget* parent)
	: ccOverlayDialog(parent, Qt::Tool | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowStaysOnTopHint)
	, m_UI(new Ui::bdrLabelAnnotationPanel )
	, m_somethingHasChanged(false)
	, m_state(0)
	, m_segmentationPoly(0)
	, m_polyVertices(0)
	, m_rectangularSelection(false)
	, m_deleteHiddenParts(false)
	, m_destination(nullptr)
	, m_segment_mode(SEGMENT_LABELING)
{
	m_UI->setupUi(this);

	connect(m_UI->settingToolButton,			&QToolButton::clicked, this, &bdrLabelAnnotationPanel::settings);
	connect(m_UI->pauseButton,					&QToolButton::toggled, this, &bdrLabelAnnotationPanel::pauseLabelingMode);
	connect(m_UI->filterToolButton,				&QToolButton::clicked, this, &bdrLabelAnnotationPanel::filterClassification);
	connect(m_UI->setLabelToolButton,			&QToolButton::clicked, this, &bdrLabelAnnotationPanel::setLabel);
	connect(m_UI->createEntityToolButton,		&QToolButton::clicked, this, &bdrLabelAnnotationPanel::createEntity);
	connect(m_UI->razButton,					&QToolButton::clicked, this, &bdrLabelAnnotationPanel::reset);
	connect(m_UI->exitButton,					&QToolButton::clicked, this, &bdrLabelAnnotationPanel::exit);

 	//selection modes
 	connect(m_UI->action2DSelection,			&QAction::triggered,	this, &bdrLabelAnnotationPanel::doSet2DSelection);
 	connect(m_UI->action3DSelection,			&QAction::triggered,	this, &bdrLabelAnnotationPanel::doSet3DSelection);
 	//import/export options
// 	connect(actionUseExistingPolyline,			&QAction::triggered,	this,	&bdrLabelAnnotationPanel::doActionUseExistingPolyline);
// 	connect(actionExportSegmentationPolyline,	&QAction::triggered,	this,	&bdrLabelAnnotationPanel::doExportSegmentationPolyline);

	connect(m_UI->exportPolygonToolButton,		&QToolButton::clicked,	this, &bdrLabelAnnotationPanel::doExportSegmentationPolyline);

	for (size_t i = 0; i < LAS_LABEL::LABEL_END; ++i) {
		m_UI->typeComboBox->addItem(LAS_LABEL::g_strLabelName[i]);
		QPixmap px(18, 18);
		px.fill(QColor(LAS_LABEL::g_classification_color[i * 3], LAS_LABEL::g_classification_color[i * 3 + 1], LAS_LABEL::g_classification_color[i * 3 + 2]));
		m_UI->typeComboBox->setItemIcon(i, QIcon(px));
	}
	m_UI->typeComboBox->setCurrentIndex(LAS_LABEL::Building);	
	connect(m_UI->typeComboBox,					SIGNAL(currentIndexChanged(int)), this, SLOT(onLasLabelChanged(int)));
	connect(m_UI->typeComboBox,					SIGNAL(activated),		this, SLOT(onLasLabelActivated));
	m_UI->typeComboBox->setFocusPolicy(Qt::NoFocus);
 
 	//add shortcuts
 	addOverridenShortcut(Qt::Key_Space);  //space bar for the "pause" button
 	addOverridenShortcut(Qt::Key_Escape); //escape key for the "cancel" button
 	addOverridenShortcut(Qt::Key_Return); //return key for the "apply" button
 	addOverridenShortcut(Qt::Key_Delete); //delete key for the "apply and delete" button
 	addOverridenShortcut(Qt::Key_Tab);    //tab key to switch between rectangular and polygonal selection modes
 	addOverridenShortcut(Qt::Key_E);      //'C' 
 	addOverridenShortcut(Qt::Key_I);      //'S' 
 	connect(this,								&ccOverlayDialog::shortcutTriggered, this, &bdrLabelAnnotationPanel::onShortcutTriggered);

 	QMenu* selectionModeMenu = new QMenu(this);
 	selectionModeMenu->addAction(m_UI->action2DSelection);
 	selectionModeMenu->addAction(m_UI->action3DSelection);
	m_UI->selectionModeButton->setDefaultAction(m_UI->action3DSelection);
	m_UI->selectionModeButton->setMenu(selectionModeMenu);
 
// 	QMenu* importExportMenu = new QMenu(this);
// 	importExportMenu->addAction(actionUseExistingPolyline);
// 	importExportMenu->addAction(actionExportSegmentationPolyline);
// 	loadSaveToolButton->setMenu(importExportMenu);
// 
 	m_polyVertices = new ccPointCloud("vertices");
 	m_segmentationPoly = new ccPolyline(m_polyVertices);
 	m_segmentationPoly->setForeground(true);
 	m_segmentationPoly->setColor(ccColor::doderBlue);
 	m_segmentationPoly->showColors(true);
 	m_segmentationPoly->set2DMode(true);
 	allowExecutePolyline(false);

	m_selection_mode = SELECT_3D;
}

void bdrLabelAnnotationPanel::allowExecutePolyline(bool state)
{
	//m_UI->createEntityToolButton->setEnabled(state);
	//m_UI->setLabelToolButton->setEnabled(state);

 	if (state) {
 	}
 	else {
		
 	}
}

void bdrLabelAnnotationPanel::allowStateChange(bool state)
{
	//m_UI->selectionModeButton->setEnabled(state);
	//m_UI->typeComboBox->setEnabled(state);
	//m_UI->filterToolButton->setEnabled(state);
}

bdrLabelAnnotationPanel::~bdrLabelAnnotationPanel()
{
	if (m_segmentationPoly)
		delete m_segmentationPoly;
	m_segmentationPoly = 0;

	if (m_polyVertices)
		delete m_polyVertices;
	m_polyVertices = 0;
}

void bdrLabelAnnotationPanel::onShortcutTriggered(int key)
{
 	switch(key)
	{
	case Qt::Key_Space:
		m_UI->pauseButton->toggle();
		return;

	case Qt::Key_I:
		MainWindow::TheInstance()->showBestImage(false);
		//inButton->click();
		return;

	case Qt::Key_E:
		m_UI->exportPolygonToolButton->click();
		//outButton->click();
		return;

	case Qt::Key_Return:
		//validButton->click();
		return;
	case Qt::Key_Delete:
		//validAndDeleteButton->click();
		return;
	case Qt::Key_Escape:
		m_UI->exitButton->click();
		return;

	case Qt::Key_Tab:
		if (m_selection_mode == SELECT_2D)
			doSet3DSelection();
		else if (m_selection_mode == SELECT_3D)
			doSet2DSelection();
		return;

	default:
		//nothing to do
		break;
	}
}

bool bdrLabelAnnotationPanel::linkWith(ccGLWindow* win)
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
	}
	
	if (m_associatedWin)
	{
		connect(m_associatedWin, &ccGLWindow::leftButtonClicked,	this, &bdrLabelAnnotationPanel::addPointToPolyline);
		connect(m_associatedWin, &ccGLWindow::rightButtonClicked,	this, &bdrLabelAnnotationPanel::closePolyLine);
		connect(m_associatedWin, &ccGLWindow::mouseMoved,			this, &bdrLabelAnnotationPanel::updatePolyLine);
		connect(m_associatedWin, &ccGLWindow::buttonReleased,		this, &bdrLabelAnnotationPanel::closeRectangle);

		if (m_segmentationPoly)
		{
			m_segmentationPoly->setDisplay(m_associatedWin);
		}
	}

	return true;
}

bool bdrLabelAnnotationPanel::start()
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
	{
		return false;
	}

	m_segmentationPoly->clear();
	m_polyVertices->clear();
	allowExecutePolyline(false);

	//the user must not close this window!
	m_associatedWin->setUnclosable(true);
	m_associatedWin->addToOwnDB(m_segmentationPoly);
	m_associatedWin->setPickingMode(ccGLWindow::NO_PICKING);
	if (m_selection_mode == SELECT_3D)
	{
		m_associatedWin->setPerspectiveState(true, true);
		m_associatedWin->lockRotationAxis(false, CCVector3d(0, 0, 1));

		m_UI->selectionModeButton->setDefaultAction(m_UI->action3DSelection);
	}
	else if (m_selection_mode == SELECT_2D) {
		m_associatedWin->setPerspectiveState(false, true);
		m_associatedWin->lockRotationAxis(true, CCVector3d(0, 0, 1));
		m_associatedWin->setView(CC_TOP_VIEW);
		m_UI->selectionModeButton->setDefaultAction(m_UI->action2DSelection);
	}
	pauseLabelingMode(false);

	m_somethingHasChanged = false;

	reset();

	return ccOverlayDialog::start();
}

void bdrLabelAnnotationPanel::removeAllEntities(bool unallocateVisibilityArrays)
{
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		if (m_segment_mode == SEGMENT_LABELING || m_segment_mode == SEGMENT_BUILD_EIDT) {
			ccPointCloud* pc = ccHObjectCaster::ToPointCloud(*p);
			if (pc) {
				if (pc->hasColors()) {
					pc->showColors(true);
					pc->showSF(false);
				}
				else {
					pc->showColors(false);
					pc->showSF(true);
				}
			}
			(*p)->setLocked(false);
		}
		else if (unallocateVisibilityArrays) {
			ccHObjectCaster::ToGenericPointCloud(*p)->unallocateVisibilityArray();
		}
	}
	
	m_toSegment.clear();
}

void bdrLabelAnnotationPanel::stop(bool accepted)
{
	assert(m_segmentationPoly);

	if (m_associatedWin) {
		m_associatedWin->setInteractionMode(ccGLWindow::TRANSFORM_CAMERA());
		m_associatedWin->setPickingMode(ccGLWindow::DEFAULT_PICKING);
		m_associatedWin->setPerspectiveState(true, true);
		m_associatedWin->lockRotationAxis(false, CCVector3d(0, 0, 1));
		m_associatedWin->setUnclosable(false);
		m_associatedWin->removeFromOwnDB(m_segmentationPoly);
	}

	ccOverlayDialog::stop(accepted);
}

void bdrLabelAnnotationPanel::reset()
{
	if (m_somethingHasChanged)
	{
		for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
		{
			if (m_segment_mode <= SEGMENT_PLANE_SPLIT) {
				ccHObjectCaster::ToGenericPointCloud(*p)->resetVisibilityArray();
			}
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

bool bdrLabelAnnotationPanel::addEntity(ccHObject* entity)
{
	//FIXME
	/*if (entity->isLocked())
		ccLog::Warning(QString("Can't use entity [%1] cause it's locked!").arg(entity->getName()));
	else */
	if (!entity->isDisplayedIn(m_associatedWin))
	{
		ccLog::Warning(QString("[Graphical Segmentation Tool] Entity [%1] is not visible in the active 3D view!").arg(entity->getName()));
	}

	bool result = false;
	if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity);
		//detect if this cloud is in fact a vertex set for at least one mesh
		{
			//either the cloud is the child of its parent mesh
			if (cloud->getParent() && cloud->getParent()->isKindOf(CC_TYPES::MESH) && ccHObjectCaster::ToGenericMesh(cloud->getParent())->getAssociatedCloud() == cloud)
			{
				//ccLog::Warning(QString("[Graphical Segmentation Tool] Can't segment mesh vertices '%1' directly! Select its parent mesh instead!").arg(entity->getName()));
				return false;
			}
			//or the parent of its child mesh!
			ccHObject::Container meshes;
			if (cloud->filterChildren(meshes,false,CC_TYPES::MESH) != 0)
			{
				for (unsigned i=0; i<meshes.size(); ++i)
					if (ccHObjectCaster::ToGenericMesh(meshes[i])->getAssociatedCloud() == cloud)
					{
						//ccLog::Warning(QString("[Graphical Segmentation Tool] Can't segment mesh vertices '%1' directly! Select its child mesh instead!").arg(entity->getName()));
						return false;
					}
			}
		}

		if (m_segment_mode == SEGMENT_LABELING || m_segment_mode == SEGMENT_BUILD_EIDT) {
			//cloud->hasScalarFields()
			//cloud->geScalarValueColor()
			
			ccPointCloud* cloudObj = ccHObjectCaster::ToPointCloud(cloud);
			if (cloudObj) {
				int class_index = cloudObj->getScalarFieldIndexByName("Classification");
				if (class_index < 0) {
					class_index = cloudObj->addScalarField("Classification");
					if (class_index < 0) return false;
				}
				
				cloudObj->setCurrentScalarField(class_index);
				cloudObj->setCurrentDisplayedScalarField(class_index);

				// default in ccPointCloud
// 				ccScalarField* sf = static_cast<ccScalarField*>(cloudObj->getCurrentInScalarField());
// 				
// 				if (!sf || sf != cloudObj->getCurrentDisplayedScalarField()) {
// 					return false;
// 				}
// 				sf->setMax(LAS_LABEL::LABEL_END - 1);
// 				sf->setMin(0);				
// 				sf->computeMinAndMax(false, false);
// 				sf->setColorScale(ccColorScalesManager::GetDefaultScale(ccColorScalesManager::CLASSIFICATION));

				cloud->showColors(false);
				cloud->showSF(true);
			}
		}
		else {
			cloud->resetVisibilityArray();
		}
		cloud->setLocked(true);
		m_toSegment.insert(cloud);

		//automatically add cloud's children
		for (unsigned i=0; i<entity->getChildrenNumber(); ++i)
			result |= addEntity(entity->getChild(i));
	}
	else if (0)//(entity->isKindOf(CC_TYPES::MESH))
	{
		if (entity->isKindOf(CC_TYPES::PRIMITIVE))
		{
			ccLog::Warning("[bdrLabelAnnotationPanel] Can't segment primitives yet! Sorry...");
			return false;
		}
		if (entity->isKindOf(CC_TYPES::SUB_MESH))
		{
			ccLog::Warning("[bdrLabelAnnotationPanel] Can't segment sub-meshes! Select the parent mesh...");
			return false;
		}
		else
		{
			ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(entity);

			//first, we must check that there's no mesh and at least one of its sub-mesh mixed in the current selection!
			for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
			{
				if ((*p)->isKindOf(CC_TYPES::MESH))
				{
					ccGenericMesh* otherMesh = ccHObjectCaster::ToGenericMesh(*p);
					if (otherMesh->getAssociatedCloud() == mesh->getAssociatedCloud())
					{
						if ((otherMesh->isA(CC_TYPES::SUB_MESH) && mesh->isA(CC_TYPES::MESH))
							|| (otherMesh->isA(CC_TYPES::MESH) && mesh->isA(CC_TYPES::SUB_MESH)))
						{
							ccLog::Warning("[Graphical Segmentation Tool] Can't mix sub-meshes with their parent mesh!");
							return false;
						}
					}
				}
			}

			mesh->getAssociatedCloud()->resetVisibilityArray();
			m_toSegment.insert(mesh);
			result = true;
		}
	}
	else if (entity->isA(CC_TYPES::HIERARCHY_OBJECT))
	{
		//automatically add entity's children
		for (unsigned i=0;i<entity->getChildrenNumber();++i)
			result |= addEntity(entity->getChild(i));
	}
	
	return result;
}

unsigned bdrLabelAnnotationPanel::getNumberOfValidEntities() const
{
	return static_cast<unsigned>(m_toSegment.size());
}

void bdrLabelAnnotationPanel::updatePolyLine(int x, int y, Qt::MouseButtons buttons)
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

void bdrLabelAnnotationPanel::settings()
{
}

void bdrLabelAnnotationPanel::addPointToPolyline(int x, int y)
{
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

void bdrLabelAnnotationPanel::closeRectangle()
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

void bdrLabelAnnotationPanel::closePolyLine(int, int)
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
		//remove last point!
		m_segmentationPoly->resize(vertCount-1); //can't fail --> smaller
		m_segmentationPoly->setClosed(true);
		allowExecutePolyline(true);
	}

	//stop
	m_state &= (~RUNNING);
 	
 	if (m_associatedWin)
 	{
 		m_associatedWin->redraw(true, false);
 	}
}

void bdrLabelAnnotationPanel::segmentIn()
{
	segment(true);
}

void bdrLabelAnnotationPanel::segmentOut()
{
	segment(false);
}

void bdrLabelAnnotationPanel::segment(bool keepPointsInside)
{
	if (!m_associatedWin)
		return;

	if (!m_segmentationPoly)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	if (!m_segmentationPoly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return;
	}

	//viewing parameters
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	//for each selected entity
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(*p);
		if (!cloud) continue;
		assert(cloud);

		ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
		assert(!visibilityArray.empty());

		unsigned cloudSize = cloud->size();

		//we project each point and we check if it falls inside the segmentation polyline
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(cloudSize); ++i)
		{
			
			if (visibilityArray[i] == POINT_VISIBLE)
			{
				const CCVector3* P3D = cloud->getPoint(i);

				CCVector3d Q2D;
				bool pointInFrustrum = camera.project(*P3D, Q2D, true);

				CCVector2 P2D(	static_cast<PointCoordinateType>(Q2D.x-half_w),
								static_cast<PointCoordinateType>(Q2D.y-half_h) );
				
				bool pointInside = pointInFrustrum && CCLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);

				visibilityArray[i] = (keepPointsInside != pointInside ? POINT_HIDDEN : POINT_VISIBLE);

			}
		}
	}

	m_somethingHasChanged = true;
 	m_UI->razButton->setEnabled(true);
	pauseLabelingMode(true);
}

void bdrLabelAnnotationPanel::pauseLabelingMode(bool state)
{
	assert(m_polyVertices && m_segmentationPoly);

	if (!m_associatedWin)
		return;

	if (state/*=activate pause mode*/) {
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
		MainWindow::TheInstance()->dispToStatus(QString("paused, press space to continue labeling"));
		m_UI->pauseButton->setIcon(QIcon(QStringLiteral(":/CC/Stocker/images/stocker/play.png")));
	}
	else {
		m_state = STARTED;
		allowStateChange(false);
		m_associatedWin->setInteractionMode(ccGLWindow::INTERACT_SHIFT_PAN | ccGLWindow::INTERACT_SEND_ALL_SIGNALS);
		if (m_selection_mode == SELECT_2D) {
			MainWindow::TheInstance()->dispToStatus(QString("labeling (2D), left click to add contour points, right click to close"));
		}
		else if (m_selection_mode == SELECT_3D) {
			MainWindow::TheInstance()->dispToStatus(QString("labeling (3D), left click to add contour points, right click to close"));
		}
		m_UI->pauseButton->setIcon(QIcon(QStringLiteral(":/CC/Stocker/images/stocker/pause.png")));
	}

	//update mini-GUI
	m_UI->pauseButton->blockSignals(true);
	m_UI->pauseButton->setChecked(state);
	m_UI->pauseButton->blockSignals(false);

	m_associatedWin->redraw(!state);
}

void bdrLabelAnnotationPanel::filterClassification()
{
}

void bdrLabelAnnotationPanel::setLabel()
{
	if (!m_associatedWin)
		return;

	if (!m_segmentationPoly)
	{
		ccLog::Error("No polyline defined!");
		return;
	}

	if (!m_segmentationPoly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return;
	}

	//viewing parameters
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	int label_index = m_UI->typeComboBox->currentIndex();
	
	//for each selected entity
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(*p);
		if (!cloud) continue;
		assert(cloud);

// 		ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
// 		assert(!visibilityArray.empty());

		unsigned cloudSize = cloud->size();

		//we project each point and we check if it falls inside the segmentation polyline
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(cloudSize); ++i)
		{

			//if (visibilityArray[i] == POINT_VISIBLE)
			{
				const CCVector3* P3D = cloud->getPoint(i);

				CCVector3d Q2D;
				bool pointInFrustrum = camera.project(*P3D, Q2D, true);

				CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
					static_cast<PointCoordinateType>(Q2D.y - half_h));

				bool pointInside = pointInFrustrum && CCLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);

				//visibilityArray[i] = (keepPointsInside != pointInside ? POINT_HIDDEN : POINT_VISIBLE);

				if (pointInside) {
					cloud->setPointScalarValue(i, label_index);
				}

			}
		}
		cloud->notifyGeometryUpdate();
		cloud->prepareDisplayForRefresh();
	}

	m_somethingHasChanged = true;
	m_UI->razButton->setEnabled(true);
	pauseLabelingMode(true);
}

void bdrLabelAnnotationPanel::createEntity()
{
	if (!m_associatedWin)
		return;
	MainWindow* mainWin = MainWindow::TheInstance(); if (!mainWin) { return; }

	std::vector<ccPolyline*> segmentPolygons;

	bool mode2d = false;
	if (m_segmentationPoly && m_segmentationPoly->isClosed()) {
		segmentPolygons.push_back(m_segmentationPoly);
		mode2d = true;
	}
	else {
		for (ccHObject* ent : mainWin->getSelectedEntities()) {
			ccPolyline* poly = ccHObjectCaster::ToPolyline(ent);
			if (poly) {
				segmentPolygons.push_back(poly);
			}
		}
	}
	if (segmentPolygons.empty()) {
		ccLog::Error("Define and/or close the segmentation polygon or choose existing polygons");
		return;
	}

	const int min_points = 15;

	//viewing parameters
	ccGLCameraParameters camera;
	m_associatedWin->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	//for each selected entity
	for (QSet<ccHObject*>::const_iterator p = m_toSegment.constBegin(); p != m_toSegment.constEnd(); ++p)
	{
		ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(*p);
		assert(cloud);

//		ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();

		unsigned cloudSize = cloud->size();
		ccHObject* destination = nullptr;
		if (m_destination) destination = m_destination;
		else {
			destination = GetRootBDBase(*p);
		}
		if (!destination) {
			destination = cloud->getParent();
		}
		if (!destination) continue;
		if (isBuildingProject(destination))	{
			m_changed_baseobj.insert(destination);
		}

		for (ccPolyline* poly : segmentPolygons) {

			//! create a new point cloud
			ccPointCloud* new_ent = new ccPointCloud();
			new_ent->setGlobalScale(cloud->getGlobalScale());
			new_ent->setGlobalShift(cloud->getGlobalShift());

			bool use_color = true;
			for (int i = 0; i < static_cast<int>(cloudSize); ++i)
			{
				const CCVector3* P3D = cloud->getPoint(i);

				CCVector3d Q2D;
				bool pointInFrustrum = camera.project(*P3D, Q2D, true);

				if (!pointInFrustrum)	{
					continue;
				}

				CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
					static_cast<PointCoordinateType>(Q2D.y - half_h));
				
				bool pointInside(false);
				if (mode2d)	{
					pointInside = CCLib::ManualSegmentationTools::isPointInsidePoly(P2D, poly);
				}
				else {
					std::vector<CCVector2> poly_vertices;
					for (CCVector3 p : poly->getPoints(false)) {
						CCVector3d pq2d;
						camera.project(p, pq2d);
						poly_vertices.push_back(CCVector2(pq2d.x - half_w, pq2d.y - half_h));
					}
					pointInside = CCLib::ManualSegmentationTools::isPointInsidePoly(P2D, poly_vertices);
				}

				if (!pointInside) { continue; }
				new_ent->addPoint(*P3D);

				if (use_color) {
					use_color = cloud->hasColors() && (new_ent->hasColors() || (!new_ent->hasColors() && new_ent->reserveTheRGBTable()));
				}
				if (use_color)
					new_ent->addRGBColor(cloud->getPointColor(i));
			}
			if (new_ent->size() < min_points) {
				delete new_ent;
				new_ent = nullptr;
				continue;
			}

			int number = 0;
			switch (m_UI->typeComboBox->currentIndex())
			{
			case LAS_LABEL::Building:
			{
				number = GetMaxNumberExcludeChildPrefix(destination, BDDB_BUILDING_PREFIX, CC_TYPES::ST_BUILDING) + 1;
				new_ent->setName(BuildingNameByNumber(number));
				break;
			}
			default:
				break;
			}

			new_ent->setPointSize(2);

			int seg_index = new_ent->addScalarField("Segmentation");
			if (seg_index >= 0) {
				CCLib::ScalarField* sf = new_ent->getScalarField(seg_index);
				if (sf) {
					sf->fill(number);
				}

				new_ent->showColors(false);
				new_ent->showSF(true);
				new_ent->setCurrentScalarField(seg_index);
				new_ent->setCurrentDisplayedScalarField(seg_index);
			}
			else {
				new_ent->showColors(true);
				new_ent->showSF(false);
			}

			switch (m_UI->typeComboBox->currentIndex())
			{
			case LAS_LABEL::Building:
			{
				ccHObject* new_building_folder = new ccHObject(new_ent->getName() + BDDB_ORIGIN_CLOUD_SUFFIX);
				new_building_folder->addChild(new_ent);
				StBuilding* new_building_obj = new StBuilding(new_ent->getName());
				new_building_obj->addChild(new_building_folder);
				new_building_obj->setDisplay_recursive(destination->getDisplay());
				new_building_obj->setDBSourceType_recursive(destination->getDBSourceType());

				destination->addChild(new_building_obj);
				MainWindow::TheInstance()->addToDB(new_building_obj, destination->getDBSourceType());

				break;
			}
			default:
				ccLog::Error("not supported yet");
				break;
			}

			m_somethingHasChanged = true;
		}		
	}

	m_UI->razButton->setEnabled(true);
	pauseLabelingMode(true);
}

void bdrLabelAnnotationPanel::exit()
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

void bdrLabelAnnotationPanel::doSet2DSelection()
{
	if (m_selection_mode == SELECT_2D) return;	
	m_selection_mode = SELECT_2D;

	if (m_state != PAUSED)
	{
		pauseLabelingMode(true);
		pauseLabelingMode(false);
	}

	m_associatedWin->setPerspectiveState(false, true);
	m_associatedWin->lockRotationAxis(true, CCVector3d(0, 0, 1));
	m_associatedWin->setView(CC_TOP_VIEW);
	m_UI->selectionModeButton->setDefaultAction(m_UI->action2DSelection);
}

void bdrLabelAnnotationPanel::doSet3DSelection()
{
	if (m_selection_mode == SELECT_3D) return;
	m_selection_mode = SELECT_3D;
		 
	if (m_state != PAUSED)
	{
		pauseLabelingMode(true);
		pauseLabelingMode(false);
	}

	m_associatedWin->setPerspectiveState(true, true);
	m_associatedWin->lockRotationAxis(false, CCVector3d(0, 0, 1));

	m_UI->selectionModeButton->setDefaultAction(m_UI->action3DSelection);
}

void bdrLabelAnnotationPanel::onLasLabelChanged(int)
{
	//int i = m_UI->typeComboBox->currentIndex();
}

void bdrLabelAnnotationPanel::onLasLabelActivated()
{
	//m_UI->pauseButton->setFocus();
}

void bdrLabelAnnotationPanel::setSegmentMode(SegmentMode mode)
{
	m_segment_mode = mode;

// 	switch (mode)
// 	{
// 	case bdrLabelAnnotationPanel::SEGMENT_GENERAL:
// 		inButton->setVisible(true);
// 		outButton->setVisible(true);
// 		validAndDeleteButton->setToolTip("Confirm and delete hidden points");
// 		validButton->setToolTip("Confirm segmentation");
// 		createBuildingToolButton->setVisible(false);
// 		break;
// 	case bdrLabelAnnotationPanel::SEGMENT_PLANE_CREATE:
// 	case bdrLabelAnnotationPanel::SEGMENT_PLANE_SPLIT:
// 		inButton->setVisible(false);
// 		outButton->setVisible(false);
// 		validAndDeleteButton->setToolTip("Confirm and delete points inside the polygon");
// 		validButton->setToolTip("Create a new plane by points inside the polygon");
// 		createBuildingToolButton->setVisible(false);
// 		break;
// 	case bdrLabelAnnotationPanel::SEGMENT_LABELING:
// 		inButton->setVisible(false);
// 		outButton->setVisible(false);
// 		break;
// 	case bdrLabelAnnotationPanel::SEGMENT_BUILD_EIDT:
// 		inButton->setVisible(false);
// 		outButton->setVisible(false);
// 		createBuildingToolButton->setVisible(true);
// 		validAndDeleteButton->setVisible(false);
// 		break;
// 	default:
// 		break;
// 	}
}

void bdrLabelAnnotationPanel::doExportSegmentationPolyline()
{
	MainWindow* mainWindow = MainWindow::TheInstance();
	if (mainWindow && m_segmentationPoly)
	{
		bool mode2D = false;
#ifdef ALLOW_2D_OR_3D_EXPORT
		QMessageBox messageBox(0);
		messageBox.setWindowTitle("Choose export type");
		messageBox.setText("Export polyline in:\n - 2D (with coordinates relative to the screen)\n - 3D (with coordinates relative to the segmented entities)");
		QPushButton* button2D = new QPushButton("2D");
		QPushButton* button3D = new QPushButton("3D");
		messageBox.addButton(button2D, QMessageBox::AcceptRole);
		messageBox.addButton(button3D, QMessageBox::AcceptRole);
		messageBox.addButton(QMessageBox::Cancel);
		messageBox.setDefaultButton(button3D);
		messageBox.exec();
		if (messageBox.clickedButton() == messageBox.button(QMessageBox::Cancel))
		{
			//process cancelled by user
			return;
		}
		mode2D = (messageBox.clickedButton() == button2D);
#endif

		ccPolyline* poly = new ccPolyline(*m_segmentationPoly);

		//if the polyline is 2D and we export the polyline in 3D, we must project its vertices
		if (!mode2D)
		{
			//get current display parameters
			ccGLCameraParameters camera;
			m_associatedWin->getGLCameraParameters(camera);
			const double half_w = camera.viewport[2] / 2.0;
			const double half_h = camera.viewport[3] / 2.0;

			//project the 2D polyline in 3D
			CCLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
			ccPointCloud* verticesPC = dynamic_cast<ccPointCloud*>(vertices);
			if (verticesPC)
			{
				for (unsigned i = 0; i < vertices->size(); ++i)
				{
					CCVector3* Pscreen = const_cast<CCVector3*>(verticesPC->getPoint(i));
					CCVector3d Pd(half_w + Pscreen->x, half_h + Pscreen->y, 0/*Pscreen->z*/);
					CCVector3d Q3D;
					camera.unproject(Pd, Q3D);
					*Pscreen = CCVector3::fromArray(Q3D.u);
				}
				verticesPC->invalidateBoundingBox();
			}
			else
			{
				delete poly;
				poly = nullptr;
				return;
			}

			//export Global Shift & Scale info (if any)
			bool hasGlobalShift = false;
			CCVector3d globalShift(0, 0, 0);
			double globalScale = 1.0;
			{
				for (QSet<ccHObject*>::const_iterator it = m_toSegment.constBegin(); it != m_toSegment.constEnd(); ++it)
				{
					ccShiftedObject* shifted = ccHObjectCaster::ToShifted(*it);
					bool isShifted = (shifted && shifted->isShifted());
					if (isShifted)
					{
						globalShift = shifted->getGlobalShift();
						globalScale = shifted->getGlobalScale();
						hasGlobalShift = true;
						break;
					}
				}
			}

			if (hasGlobalShift && m_toSegment.size() != 1)
			{
				hasGlobalShift = (QMessageBox::question(MainWindow::TheInstance(), "Apply Global Shift", "At least one of the segmented entity has been shifted. Apply the same shift to the polyline?", QMessageBox::Yes, QMessageBox::No) == QMessageBox::Yes);
			}

			if (hasGlobalShift)
			{
				poly->setGlobalShift(globalShift);
				poly->setGlobalScale(globalScale);
			}
		}

		QString polyName = QString("Polygon");
		poly->setName(polyName);
		poly->setEnabled(false); //we don't want it to appear while the segmentation mode is enabled! (anyway it's 2D only...)
		poly->set2DMode(mode2D);
		poly->setColor(ccColor::yellow); //we use a different color so as to differentiate them from the active polyline!

		//save associated viewport
		cc2DViewportObject* viewportObject = new cc2DViewportObject(polyName + QString(" viewport"));
		viewportObject->setParameters(m_associatedWin->getViewportParameters());
		viewportObject->setDisplay(m_associatedWin);
		poly->addChild(viewportObject);

		mainWindow->addToDB(poly, CC_TYPES::DB_BUILDING, false, false, false);
	}
}