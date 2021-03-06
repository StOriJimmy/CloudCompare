//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_MAIN_WINDOW_HEADER
#define CC_MAIN_WINDOW_HEADER

//Qt
#include <QMainWindow>

//Local
#include "ccEntityAction.h"
#include "ccMainAppInterface.h"
#include "ccPickingListener.h"

//CCLib
#include <AutoSegmentationTools.h>

class QAction;
class QMdiArea;
class QMdiSubWindow;
class QToolBar;
class QToolButton;
class QPushButton;
class QProgressBar;
class QLabel;
class QSpinBox;

class cc3DMouseManager;
class ccCameraParamEditDlg;
class ccClippingBoxTool;
class ccComparisonDlg;
class ccDBRoot;
class ccDrawableObject;
class ccGamepadManager;
class ccGLWindow;
class ccGraphicalSegmentationTool;
class ccGraphicalTransformationTool;
class ccHObject;
class ccOverlayDialog;
class ccPluginUIManager;
class ccPointListPickingDlg;
class ccPointPairRegistrationDlg;
class ccPointPropertiesDlg;
class ccPrimitiveFactoryDlg;
class ccRecentFiles;
class ccSectionExtractionTool;
class ccStdPluginInterface;
class ccTracePolylineTool;

struct dbTreeSelectionInfo;

class bdrLine3DppDlg;
class bdrDeductionDlg;
class bdrPolyFitDlg;
class bdrSettingLoD2Dlg;
class bdrPlaneSegDlg;
class bdrFacetFilterDlg;
class bdr2Point5DimEditor;
class bdrImageEditorPanel;
class bdrPlaneEditorDlg;
class bdrPlaneQualityDlg;

class bdrSettingBDSegDlg;
class bdrSettingGrdFilterDlg;

class bdrProjectDlg;
class bdrLabelAnnotationPanel;
class bdr3DGeometryEditPanel;

class PolyFitObj;

class StDBMainRoot;
class StDBBuildingRoot;
class StDBImageRoot;
class DataBaseHObject;
class BDBaseHObject;
class BDImageBaseHObject;

namespace Ui {
	class MainWindow;
} 

//! Main window
class MainWindow : public QMainWindow, public ccMainAppInterface, public ccPickingListener 
{
	Q_OBJECT

protected:
	//! Default constructor
	MainWindow();

	//! Default desctructor
	~MainWindow() override;
	
public:
	//! Returns the unique instance of this object
	static MainWindow* TheInstance();

	//! Static shortcut to MainWindow::getActiveGLWindow
	static ccGLWindow* GetActiveGLWindow();

	//! Returns a given GL sub-window (determined by its title)
	/** \param title window title
	**/
	static ccGLWindow* GetGLWindow(const QString& title);

	//! Returns all GL sub-windows
	/** \param[in,out] glWindows vector to store all sub-windows
	**/
	static void GetGLWindows(std::vector<ccGLWindow*>& glWindows);

	//! Static shortcut to MainWindow::refreshAll
	static void RefreshAllGLWindow(bool only2D = false);

	//! Static shortcut to MainWindow::updateUI
	static void UpdateUI();

	//! Deletes current main window instance
	static void DestroyInstance();

	//! Returns active GL sub-window (if any)
	ccGLWindow* getActiveGLWindow() override;
	
	//! Returns MDI area subwindow corresponding to a given 3D view
	QMdiSubWindow* getMDISubWindow(ccGLWindow* win);

	//! Returns a given views
	ccGLWindow* getGLWindow(int index) const;

	//! Returns the number of 3D views
	int getGLWindowCount() const;

	CC_TYPES::DB_SOURCE getCurrentDB() override;

	/// load mode -1 for normal, 0 for really fast (no shift), 1 for general fast (auto shift)
	std::vector<ccHObject*> loadFiles(const QStringList& filenames, int loadMode = -1, QString fileFilter = QString());

	//! Tries to load several files (and then pushes them into main DB)
	/** \param filenames list of all filenames
		\param fileFilter selected file filter (i.e. type)
		\param destWin destination window (0 = active one)
	**/	
	std::vector<ccHObject*> addToDB( const QStringList& filenames, 
						  CC_TYPES::DB_SOURCE dest,
						  QString fileFilter = QString(),
						  ccGLWindow* destWin = nullptr);
	
	//inherited from ccMainAppInterface
	void addToDB( ccHObject* obj,
				  CC_TYPES::DB_SOURCE dest,
				  bool updateZoom = false,
				  bool autoExpandDBTree = true,
				  bool checkDimensions = false,
				  bool autoRedraw = true ) override;

	void addToDB(ccHObject* obj,
		bool updateZoom = false,
		bool autoExpandDBTree = true,
		bool checkDimensions = false,
		bool autoRedraw = true) override;

	std::vector<ccHObject*> addToDB_Main(const QStringList& filenames, QString fileFilter = QString(), ccGLWindow* destWin = nullptr) {	
		return addToDB(filenames, CC_TYPES::DB_MAINDB, fileFilter, destWin); 
	}
	std::vector<ccHObject*> addToDB_Build(const QStringList& filenames, QString fileFilter = QString(), ccGLWindow* destWin = nullptr) {
		return addToDB(filenames, CC_TYPES::DB_BUILDING, fileFilter, destWin);
	}
	std::vector<ccHObject*> addToDB_Image(const QStringList& filenames, QString fileFilter = QString(), ccGLWindow* destWin = nullptr) {
		return addToDB(filenames, CC_TYPES::DB_IMAGE, fileFilter, destWin);
	}

	void addToDB_Main(ccHObject* obj, bool updateZoom = false, bool autoExpandDBTree = true, bool checkDimensions = false, bool autoRedraw = true) { 
		addToDB(obj, CC_TYPES::DB_MAINDB, updateZoom, autoExpandDBTree, checkDimensions, autoRedraw); 
	}
	void addToDB_Build(ccHObject* obj, bool updateZoom = false, bool autoExpandDBTree = true, bool checkDimensions = false, bool autoRedraw = true) {
		addToDB(obj, CC_TYPES::DB_BUILDING, updateZoom, autoExpandDBTree, checkDimensions, autoRedraw);
	}
	void addToDB_Image(ccHObject* obj, bool updateZoom = false, bool autoExpandDBTree = true, bool checkDimensions = false, bool autoRedraw = true) {
		addToDB(obj, CC_TYPES::DB_IMAGE, updateZoom, autoExpandDBTree, checkDimensions, autoRedraw);
	}

	void addToDatabase(QStringList files, ccHObject* import_pool, bool remove_exist = true, bool auto_sort = true);
	ccHObject::Container addPointsToDatabase(QStringList files, ccHObject* import_pool, bool remove_exist = true, bool auto_sort = true);

	ccHObject::Container addFilesToDatabase(QStringList files, ccHObject * import_pool, bool remove_exist, bool auto_sort);
	
	void registerOverlayDialog(ccOverlayDialog* dlg, Qt::Corner pos) override;
	void unregisterOverlayDialog(ccOverlayDialog* dlg) override;
	void updateOverlayDialogsPlacement() override;
	void removeFromDB(ccHObject* obj, bool autoDelete = true) override;
	void setSelectedInDB(ccHObject* obj, bool selected) override;	
	void dispToConsole(QString message, ConsoleMessageLevel level = STD_CONSOLE_MESSAGE) override;
	void dispToStatus(QString message, int time = 0);
	void forceConsoleDisplay() override;
	ccHObject* dbRootObject(CC_TYPES::DB_SOURCE rt) override;
	ccHObject* dbRootObject() override;
	inline  QMainWindow* getMainWindow() override { return this; }
	inline  const ccHObject::Container& getSelectedEntities() const override { return m_selectedEntities; }
	void createGLWindow(ccGLWindow*& window, QWidget*& widget) const override;
	void destroyGLWindow(ccGLWindow*) const override;
	ccUniqueIDGenerator::Shared getUniqueIDGenerator() override;
	ccColorScalesManager* getColorScalesManager() override;
	void spawnHistogramDialog(	const std::vector<unsigned>& histoValues,
								double minVal, double maxVal,
								QString title, QString xAxisLabel) override;
	ccPickingHub* pickingHub() override { return m_pickingHub; }
	ccHObjectContext removeObjectTemporarilyFromDBTree(ccHObject* obj) override;
	void putObjectBackIntoDBTree(ccHObject* obj, const ccHObjectContext& context) override;
	
	//! Inherited from ccPickingListener
	void onItemPicked(const PickedItem& pi) override;

	void unselectAllInDB();

	void switchDatabase(CC_TYPES::DB_SOURCE src);
	
	//! Returns real 'dbRoot' object
	virtual ccDBRoot* db(CC_TYPES::DB_SOURCE tp);
	virtual ccDBRoot* db(ccHObject* obj) { return db(obj->getDBSourceType()); }
	virtual StDBMainRoot* db_main() { return m_ccRoot; }
	virtual StDBBuildingRoot* db_building() { return m_buildingRoot; }
	virtual StDBImageRoot* db_image() { return m_imageRoot; }
	ccHObject* getRoot(CC_TYPES::DB_SOURCE tp);

	//! Adds the "Edit Plane" action to the given menu.
	/**
	 * This is the only MainWindow UI action used externally (by ccDBRoot).
	**/
	void  addEditPlaneAction( QMenu &menu ) const;
	
	//! Sets up the UI (menus and toolbars) based on loaded plugins
	void initPlugins();

	//! Updates the 'Properties' view
	void updatePropertiesView();

	//! XYLIU images

	ccHObject* getCameraGroup(QString name);
	void setStatusImageCoord(const CCVector3d & P, bool b3d);

	ccHObject::Container getMainDatabases(bool check_enable);

	DataBaseHObject* getCurrentMainDatabase(bool check_enable);
	DataBaseHObject* getCurrentMainDatabase();

	void showBestImage(bool use_area = true);
	void showImage(ccHObject* imCamera);

	QProgressBar* getProgressBar() { return m_progressBar; }
	void progressStart(QString name, int size);
	void progressStep();
	void progressStop();

private slots:
	//! Creates a new 3D GL sub-window
	ccGLWindow* new3DView( bool allowEntitySelection );

	//! Zooms in (current 3D view)
	void zoomIn();
	//! Zooms out (current 3D view)
	void zoomOut();

	//! Displays 'help' dialog
	void doActionShowHelpDialog();
	//! Displays file open dialog
	void doActionLoadFile();
	//! Displays file save dialog
	void doActionSaveFile();
	//! Displays the Global Shift settings dialog
	void doActionGlobalShiftSeetings();
	//! Toggles the 'show Qt warnings in Console' option
	void doEnableQtWarnings(bool);

	//! Updates entities display target when a gl sub-window is deleted
	/** \param glWindow the window that is going to be delete
	**/
	void prepareWindowDeletion(QObject* glWindow);

	//! Slot called when the exclusive fullscreen mode is toggled on a window
	void onExclusiveFullScreenToggled(bool);

	//inherited from ccMainAppInterface
	void freezeUI(bool state) override;
	void redrawAll(bool only2D = false) override;
	void refreshAll(bool only2D = false) override;
	void enableAll() override;
	void disableAll() override;
	void disableAllBut(ccGLWindow* win) override;
	void updateUI() override;
	
	virtual void toggleActiveWindowStereoVision(bool);
	void toggleActiveWindowCenteredPerspective() override;
	void toggleActiveWindowCustomLight() override;
	void toggleActiveWindowSunLight() override;
	void toggleActiveWindowViewerBasedPerspective() override;
	void zoomOnSelectedEntities() override;
	void setGlobalZoom() override;
	
	void increasePointSize() override;
	void decreasePointSize() override;
	
	void toggleLockRotationAxis();
	void doActionEnableBubbleViewMode();
	void setPivotAlwaysOn();
	void setPivotRotationOnly();
	void setPivotOff();
	void toggleActiveWindowAutoPickRotCenter(bool);
	void toggleActiveWindowShowCursorCoords(bool);
	void toggleActiveWindowPointViewEditMode(bool);

	//! Handles new label
	void handleNewLabel(ccHObject*);

	void setActiveSubWindow(QWidget* window);
	void showDisplayOptions();
	void showSelectedEntitiesHistogram();
	void testFrameRate();
	void toggleFullScreen(bool state);
	void toggleVisualDebugTraces();
	void toggleExclusiveFullScreen(bool state);
	void update3DViewsMenu();
	void updateMenus();
	void on3DViewActivated(QMdiSubWindow*);
	void updateUIWithSelection();
	void updateViewStateWithSelection();
	void addToDBAuto(const QStringList& filenames);

	void echoMouseWheelRotate(float);
	void echoCameraDisplaced(float ddx, float ddy);
	void echoBaseViewMatRotation(const ccGLMatrixd& rotMat);
	void echoCameraPosChanged(const CCVector3d&);
	void echoPivotPointChanged(const CCVector3d&);
	void echoPixelSizeChanged(float);
	void echoMouseMoved3D(const CCVector3d & P, bool b3d);
	void echoMouseMoved2D(int x, int y, double depth);
	void echopointSnapBufferChanged(int buffer);
	void echoImageCursorPos(const CCVector3d & P, bool b3d);

	void pointSnapBufferChanged(int buffer);

	void doActionRenderToFile();

	//menu action
	void doActionSetUniqueColor();
	void doActionColorize();
	void doActionRGBToGreyScale();
	void doActionSetColor(bool colorize);
	void doActionSetColorGradient();
	void doActionInterpolateColors();
	void doActionChangeColorLevels();
	void doActionEnhanceRGBWithIntensities();

	void doActionDisplayGlobalCoord();

	void doActionSFGaussianFilter();
	void doActionSFBilateralFilter();
	void doActionSFConvertToRGB();
	void doActionSFConvertToRandomRGB();
	void doActionRenameSF();
	void doActionOpenColorScalesManager();
	void doActionAddIdField();
	void doActionSetSFAsCoord();
	void doActionInterpolateScalarFields();

	void doComputeGeometricFeature();
	void doActionSFGradient();
	void doRemoveDuplicatePoints();
	void doSphericalNeighbourhoodExtractionTest(); //DGM TODO: remove after test
	void doCylindricalNeighbourhoodExtractionTest(); //DGM TODO: remove after test
	void doActionFitPlane();
	void doActionFitSphere();
	void doActionFitFacet();
	void doActionFitQuadric();
	void doShowPrimitiveFactory();

	void doActionComputeNormals();
	void doActionInvertNormals();
	void doActionConvertNormalsToHSV();
	void doActionConvertNormalsToDipDir();
	void doActionComputeOctree();
	void doActionComputeKdTree();
	void doActionApplyTransformation();
	void doActionMerge();
	void doActionRegister();
	void doAction4pcsRegister(); //Aurelien BEY le 13/11/2008
	void doActionSubsample(); //Aurelien BEY le 4/12/2008
	void doActionStatisticalTest();
	void doActionSamplePointsOnMesh();
	void doActionSamplePointsOnPolyline();
	void doActionConvertTextureToColor();
	void doActionLabelConnectedComponents();
	void doActionComputeStatParams();
	void doActionFilterByValue();
	
	// Picking opeations
	void enablePickingOperation(ccGLWindow* win, QString message);
	void cancelPreviousPickingOperation(bool aborted);

	// For rotation center picking
	void doPickRotationCenter();
	// For leveling
	void doLevel();
	
	void doActionCreatePlane();
	void doActionEditPlane();
	void doActionFlipPlane();
	void doActionComparePlanes();

	void doActionDeleteScanGrids();
	void doActionSmoothMeshSF();
	void doActionEnhanceMeshSF();
	void doActionAddConstantSF();
	void doActionScalarFieldArithmetic();
	void doActionScalarFieldFromColor();
	void doActionOrientNormalsFM();
	void doActionOrientNormalsMST();
	void doActionResampleWithOctree();
	void doActionComputeMeshAA();
	void doActionComputeMeshLS();
	void doActionMeshScanGrids();
	void doActionComputeDistanceMap();
	void doActionComputeDistToBestFitQuadric3D();
	void doActionMeasureMeshSurface();
	void doActionMeasureMeshVolume();
	void doActionFlagMeshVertices();
	void doActionSmoothMeshLaplacian();
	void doActionSubdivideMesh();
	void doActionFlipMeshTriangles();
	void doActionComputeCPS();
	void doActionShowWaveDialog();
	void doActionCompressFWFData();
	void doActionKMeans();
	void doActionFrontPropagation();
	void doActionApplyScale();
	void doActionEditGlobalShiftAndScale();
	void doActionMatchBBCenters();
	void doActionMatchScales();
	void doActionSORFilter();
	void doActionFilterNoise();
	void doActionUnroll();
	void doActionCreateGBLSensor();
	void doActionCreateCameraSensor();
	void doActionModifySensor();
	void doActionProjectUncertainty();
	void doActionCheckPointsInsideFrustum();
	void doActionComputeDistancesFromSensor();
	void doActionComputeScatteringAngles();
	void doActionSetViewFromSensor();
	void doActionShowDepthBuffer();
	void doActionExportDepthBuffer();
	void doActionComputePointsVisibility();
	void doActionRasterize();
	void doCompute2HalfDimVolume();
	void doConvertPolylinesToMesh();
	void doMeshTwoPolylines();
	void doActionExportCoordToSF();
	void doActionExportNormalToSF();
	void doComputeBestFitBB();
	void doActionCrop();

	//! Clones currently selected entities
	void doActionClone();

	void doActionEditCamera();
	void doActionAdjustZoom();
	void doActionSaveViewportAsCamera();
	void doActionResetGUIElementsPos();

	//Shaders & plugins
	void doActionLoadShader();
	void doActionDeleteShader();

	void doActionFindBiggestInnerRectangle();

	//Clipping box
	void activateClippingBoxMode();
	void deactivateClippingBoxMode(bool);

	//Graphical transformation
	void activateTranslateRotateMode();
	void deactivateTranslateRotateMode(bool);

	//Graphical segmentation
	void activateSegmentationMode();
	void deactivateSegmentationMode(bool);

	//Polyline tracing
	void activateTracePolylineMode();
	void deactivateTracePolylineMode(bool);

	//Section extraction
	void activateSectionExtractionMode();
	void deactivateSectionExtractionMode(bool);

	//Entities comparison
	void doActionCloudCloudDist();
	void doActionCloudMeshDist();
	void doActionCloudPrimitiveDist();
	void deactivateComparisonMode(int);
	void doActionCloudModelDist();

	//Point picking mechanism
	void activatePointPickingMode();
	void deactivatePointPickingMode(bool);

	//Point list picking mechanism
	void activatePointListPickingMode();
	void deactivatePointListPickingMode(bool);

	//Point-pair registration mechanism
	void activateRegisterPointPairTool();
	void deactivateRegisterPointPairTool(bool);

	//Current active scalar field
	void doActionToggleActiveSFColorScale();
	void doActionShowActiveSFPrevious();
	void doActionShowActiveSFNext();

	//! Removes all entities currently loaded in the DB tree
	void closeAll();

	//! Batch export some info from a set of selected clouds
	void doActionExportCloudInfo();
	//! Batch export some info from a set of selected planes
	void doActionExportPlaneInfo();

	//! Generates a matrix with the best (registration) RMS for all possible couple among the selected entities
	void doActionComputeBestICPRmsMatrix();

	//! Creates a cloud with the (bounding-box) centers of all selected entities
	void doActionCreateCloudFromEntCenters();

	inline void doActionMoveBBCenterToOrigin()    { doActionFastRegistration(MoveBBCenterToOrigin); }
	inline void doActionMoveBBMinCornerToOrigin() { doActionFastRegistration(MoveBBMinCornerToOrigin); }
	inline void doActionMoveBBMaxCornerToOrigin() { doActionFastRegistration(MoveBBMaxCornerToOrigin); }

	bool updateBuildingList(BDBaseHObject* baseObj, bool from_file);

	ccHObject * LoadBDReconProject_Shell(QString Filename);

	ccHObject * LoadBDReconProject(QString Filename);

	bool saveImageJuctions(BDImageBaseHObject * imagePrj);

	//////////////////////////////////////////////////////////////////////////
	//! Building Reconstruction
	/// Load Project
	void doActionBDProjectCreate();
	void doActionBDProjectLoad();
	void doActionBDProjectSave();

	void doActionBDRLoadModels();
	void doActionBDRExportModels();

	/// image project
	void doActionBDImagesLoad();
	void doActionBDImagesToggle3DView();

	void doActionBDPrimitives();
	void deactiveBDPrimitives(bool state);
	/// Plane Segmentation
	void doActionBDPlaneSegmentation();
	void doActionBDPrimPlaneQuality();

	void doActionBDRetrieve();
	void doActionBDRetrievePlanePoints();
	/// Create Image Lines
	void doActionBDImageLines();

	//! Planar Primitives
	/// Intersections - get intersection lines from planes
	void doActionBDPrimIntersections();
	/// Assign Sharp Lines - assign sharp lines for each plane
	void doActionBDPrimAssignSharpLines();
	/// plane segmentation from sharp lines
	void doActionBDPrimPlaneFromSharp();
	/// Boundary - generate plane boundary
	void doActionBDPrimBoundary();
	/// Outline - generate plane outlines
	void doActionBDPrimOutline();
	/// Plane Frame - generate plane frames by optimization
	void doActionBDPrimPlaneFrame();
	/// Merge Selected Planes
	void doActionBDPrimMergePlane();
	/// split selected plane
	void doActionBDPrimSplitPlane();			///< ccGraphicalSegmentationTool
	/// Create Ground Plane
	void doActionBDPrimCreateGround();
	/// Shrink plane to outline - remove points outside the alpha shape
	void doActionBDPrimShrinkPlane();

	void doActionBDPrimPointProjection();

	void doActionBDPlaneFromPoints();			///< ccGraphicalSegmentationTool
	void doActionBDPlaneFromPolygon();			///< ccTracePolylineTool
	/// Plane Deduction
	void doActionBDPlaneDeduction();
	/// Make plane from line(s)
	void doActionBDPlaneCreate();
	/// PolyFit
	void doActionBDPolyFit();

	void doActionBDPolyFitHypothesis();

	void doActionBDPolyFitConfidence();

	void doActionBDPolyFitSelection();

	void doActionBDPolyFitFacetFilter();

	void doActionBDPolyFitSettings();

	void doActionBDFootPrintAuto();
	void doActionBDFootPrintManual();			///< ccSectionExtractionTool
	void doActionBDFootPrintPack();
	void doActionBDFootPrintGetPlane();
	void doActionBDMeshToBlock();

	void doActionBDLoD1Generation();

	/// 3d4em
	void doActionBDLoD2Generation();

	void doActionSettingsLoD2();

	void doActionBDTextureMapping();

	void doActionBDConstrainedMesh();
	/// display
	void doActionBDDisplayPlaneOn();
	void doActionBDDisplayPlaneOff();
	void doActionBDDisplayPointOn();
	void doActionBDDisplayPointOff();
	void doActionDisplayWireframe();
	void doActionDisplayFace();
	void doActionDisplayNormalPerFace();
	void doActionDisplayNormalPerVertex();

	/// image
	void doActionShowBestImage();
	void doActionShowSelectedImage();

	void doActionChangeTabTree(int index);

	void updateDBSelection(CC_TYPES::DB_SOURCE type);

	void toggleImageOverlay();

	void clearImagePanel();

	void doActionProjectToImage();

	void doActionSelectWorkingPlane();

	void doActionTogglePlaneEditState();

	void doActionEditSelectedItem();

	void doActionCreateDatabase();
	void doActionOpenDatabase();
	void doActionSaveDatabase();	
	void doActionImportData();
	void doActionImportFolder();
	void doActionEditDatabase();
	void doActionCreateBuildingProject();
	void doActionLoadSubstance();

	void doActionImageLiDARRegistration();
	void doActionRegistrationEditor();

	void doActionGroundFilteringBatch();
	void doActionClassificationBatch();
	void doActionBuildingSegmentationBatch();

	void doActionPointClassEditor();
	void deactivatePointClassEditor(bool);
	void doActionBuildingSegmentEditor();	
	void deactivateBuildingSegmentEditor(bool);

	void doAactionSettingsGroundFiltering();
	void doActionSettingsClassification();
	void doActionSettingsBuildingSeg();
	
	void doActionScheduleProjectID();
	void doActionScheduleGCServer();
	void doActionScheduleGCNode();

	void doActionClearEmptyItems();

private:
	//! Shortcut: asks the user to select one cloud
	/** \param defaultCloudEntity a cloud to select by default (optional)
		\param inviteMessage invite message (default is something like 'Please select an entity:') (optional)
		\return the selected cloud (or null if the user cancelled the operation)
	**/
	ccPointCloud* askUserToSelectACloud(ccHObject* defaultCloudEntity = nullptr, QString inviteMessage = QString());

	enum FastRegistrationMode
	{
		MoveBBCenterToOrigin,
		MoveBBMinCornerToOrigin,
		MoveBBMaxCornerToOrigin
	};

	void doActionFastRegistration(FastRegistrationMode mode);

	void toggleSelectedEntitiesProperty( ccEntityAction::TOGGLE_PROPERTY property );
	void clearSelectedEntitiesProperty( ccEntityAction::CLEAR_PROPERTY property );
	
	void setView( CC_VIEW_ORIENTATION view ) override;
	
	//! Apply transformation to the selected entities
	void applyTransformation(const ccGLMatrixd& transMat);

	//! Creates point clouds from multiple 'components'
	void createComponentsClouds(ccGenericPointCloud* cloud,
								CCLib::ReferenceCloudContainer& components,
								unsigned minPointPerComponent,
								bool randomColors,
								bool selectComponents,
								bool sortBysize = true);

	//! Saves position and state of all GUI elements
	void saveGUIElementsPos();

	void setOrthoView(ccGLWindow* win);
	void setCenteredPerspectiveView(ccGLWindow* win, bool autoRedraw = true);
	void setViewerPerspectiveView(ccGLWindow* win);
	
	void showEvent(QShowEvent* event) override;
	void closeEvent(QCloseEvent* event) override;
	void moveEvent(QMoveEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;
	bool eventFilter(QObject *obj, QEvent *event) override;
	void keyPressEvent(QKeyEvent *event) override;
	
	//! Makes the window including an entity zoom on it (helper)
	void zoomOn(ccHObject* object);

	//! Active SF action fork
	/** - action=0 : toggle SF color scale
		- action=1 : activate previous SF
		- action=2 : activate next SF
		\param action action id
	**/
	void doApplyActiveSFAction(int action);

	//! Mesh computation fork
	/** \param type triangulation type
	**/
	void doActionComputeMesh(CC_TRIANGULATION_TYPES type);

	//! Computes the orientation of an entity
	/** Either fit a plane or a 'facet' (2D polygon)
	**/
	void doComputePlaneOrientation(bool fitFacet);

	//! Sets up any input devices (3D mouse, gamepad) and adds their menus
	void setupInputDevices();
	//! Stops input and destroys any input device handling
	void destroyInputDevices();

	//! Connects all QT actions to slots
	void connectActions();

	//! Enables menu entires based on the current selection
	void enableUIItems(dbTreeSelectionInfo& selInfo);

	//! Updates the view mode pop-menu based for a given window (or an absence of!)
	virtual void updateViewModePopUpMenu(ccGLWindow* win);

	//! Updates the pivot visibility pop-menu based for a given window (or an absence of!)
	virtual void updatePivotVisibilityPopUpMenu(ccGLWindow* win);

	//! Checks whether stereo mode can be stopped (if necessary) or not
	bool checkStereoMode(ccGLWindow* win);

	Ui::MainWindow	*m_UI;
	
	//DB & DB Tree
	StDBMainRoot* m_ccRoot;
	//Building DB Tree
	StDBBuildingRoot* m_buildingRoot;
	//Image DB Tree
	StDBImageRoot* m_imageRoot;

	//! Currently selected entities;
	ccHObject::Container m_selectedEntities;

	//! UI frozen state (see freezeUI)
	bool m_uiFrozen;

	//! Recent files menu
	ccRecentFiles* m_recentFiles;
	
	//! 3D mouse
	cc3DMouseManager* m_3DMouseManager;

	//! Gamepad handler
	ccGamepadManager* m_gamepadManager;

	//! View mode pop-up menu button
	QToolButton* m_viewModePopupButton;

	//! Pivot visibility pop-up menu button
	QToolButton* m_pivotVisibilityPopupButton;

	QToolButton* m_lidarproFilterPopupButton;
	QToolButton* m_lidarproClassifyPopupButton;
	QToolButton* m_lidarproBdsegmentPopupButton;

	//! Flag: first time the window is made visible
	bool m_FirstShow;

	//! Point picking hub
	ccPickingHub* m_pickingHub;

	/******************************/
	/***      STATUS BAR        ***/
	/******************************/
	QLabel* m_progressLabel;
	QProgressBar* m_progressBar;
	QPushButton* m_progressButton;

	QSpinBox* m_status_pointSnapBufferSpinBox;
	QLabel* m_status_depth;
	QToolButton* m_status_show_coord3D;
	QToolButton* m_status_show_global;
	QLabel* m_status_coord3D;
	QLabel* m_status_coord2D;
	

	/******************************/
	/***        MDI AREA        ***/
	/******************************/

	QMdiArea* m_mdiArea;

	//! CloudCompare MDI area overlay dialogs
	struct ccMDIDialogs
	{
		ccOverlayDialog* dialog;
		Qt::Corner position;

		//! Constructor with dialog and position
		ccMDIDialogs(ccOverlayDialog* dlg, Qt::Corner pos)
			: dialog(dlg)
			, position(pos)
		{}
	};

	//! Repositions an MDI dialog at its right position
	void repositionOverlayDialog(ccMDIDialogs& mdiDlg);

	//! Registered MDI area 'overlay' dialogs
	std::vector<ccMDIDialogs> m_mdiDialogs;

	/*** dialogs ***/
	//! Camera params dialog
	ccCameraParamEditDlg* m_cpeDlg;
	//! Graphical segmentation dialog
	ccGraphicalSegmentationTool* m_gsTool;
	//! Polyline tracing tool
	ccTracePolylineTool * m_tplTool;
	//! Section extraction dialog
	ccSectionExtractionTool* m_seTool;
	//! Graphical transformation dialog
	ccGraphicalTransformationTool* m_transTool;
	//! Clipping box dialog
	ccClippingBoxTool* m_clipTool;
	//! Cloud comparison dialog
	ccComparisonDlg* m_compDlg;
	//! Point properties mode dialog
	ccPointPropertiesDlg* m_ppDlg;
	//! Point list picking
	ccPointListPickingDlg* m_plpDlg;
	//! Point-pair registration
	ccPointPairRegistrationDlg* m_pprDlg;
	//! Primitive factory dialog
	ccPrimitiveFactoryDlg* m_pfDlg;


	/*** plugins ***/
	//! Manages plugins - menus, toolbars, and the about dialog
	ccPluginUIManager	*m_pluginUIManager;

	//////////////////////////////////////////////////////////////////////////
	// XYLIU
	ccHObject* askUserToSelect(CC_CLASS_ENUM type, ccHObject* defaultCloudEntity = 0, QString inviteMessage = QString(), ccHObject* root = nullptr);

	void doActionToggleDrawBBox();

	void CreateImageEditor();

	void CreateEditorPanel();

	void Link3DAnd2DWindow();

	//! Building Reconstruction dialogs
	bdrPlaneSegDlg* m_pbdrPSDlg;
	bdrLine3DppDlg* m_pbdrl3dDlg;
	bdrDeductionDlg* m_pbdrddtDlg;
	bdrPolyFitDlg* m_pbdrpfDlg;
	bdrFacetFilterDlg* m_pbdrffDlg;
	bdrSettingLoD2Dlg* m_pbdrSettingLoD2Dlg;
	bdr2Point5DimEditor* m_pbdrImshow;
	bdrImageEditorPanel* m_pbdrImagePanel;
	bdrPlaneEditorDlg* m_pbdrPlaneEditDlg;
	bdrPlaneQualityDlg* m_pbdrPlaneQDlg;

	bdrSettingBDSegDlg* m_pbdrSettingBDSegDlg;
	bdrSettingGrdFilterDlg* m_pbdrSettingGrdFilterDlg;

	bdrProjectDlg* m_pbdrPrjDlg;
	bdrLabelAnnotationPanel* m_pbdrLAPanel;
	bdr3DGeometryEditPanel* m_pbdrGeoPanel;

	PolyFitObj* polyfit_obj;
	int m_GCSvr_prj_id;
};

#include "ccProgressDialog.h"
#define ProgStart(title) \
		ccProgressDialog progDlg(false, this);\
		progDlg.setAutoClose(false);\
		if (progDlg.textCanBeEdited()) {\
			progDlg.setMethodTitle(title);\
			progDlg.setInfo("Processing, please wait...");}\
			progDlg.start();

#define ProgStartNorm(title, number) \
		ccProgressDialog progDlg(true, this);\
		progDlg.setAutoClose(false);\
		if (progDlg.textCanBeEdited()) {\
			progDlg.setMethodTitle(title);\
			char infos[256]; sprintf(infos, "Processing %d items...please wait", number);\
			progDlg.setInfo(infos);}\
		CCLib::NormalizedProgress nprogress(&progDlg, number);\
		progDlg.start();
#define ProgStartNorm_(title, number) \
		ccProgressDialog progDlg(true, MainWindow::TheInstance());\
		progDlg.setAutoClose(false);\
		if (progDlg.textCanBeEdited()) {\
			progDlg.setMethodTitle(title);\
			char infos[256]; sprintf(infos, "Processing %d items...please wait", number);\
			progDlg.setInfo(infos);}\
		CCLib::NormalizedProgress nprogress(&progDlg, number);\
		progDlg.start();
#define ProgStepReturn(x) if (!nprogress.oneStep()) {progDlg.stop(); return x;}
#define ProgStepBreak if (!nprogress.oneStep()) {progDlg.stop(); break;}
#define ProgEnd progDlg.update(100.0f); progDlg.stop();

#endif
