#ifndef BDR_3D_GEOMETRY_EDIT_PANEL_HEADER
#define BDR_3D_GEOMETRY_EDIT_PANEL_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>

//Qt
#include <QSet>

//GUI
#include <ui_bdr3DGeometryEditPanel.h>

class ccPolyline;
class ccPointCloud;
class ccGLWindow;
class ccPlane;
class QToolButton;
class ccPickingHub;
class bdrPlaneEditorDlg;
class BDBaseHObject;

#define GEO_ROUND_FLAG QStringLiteral("ROUND")

enum GEOMETRY3D
{
	GEO_BLOCK,
	GEO_BOX,
	GEO_SPHERE,
	GEO_CONE,
	GEO_CYLINDER,
	GEO_PARAPET,
	GEO_TORCYLINDER,
	GEO_TORUS,
	GEO_DISH,
	GEO_PLANE,
	GEO_POLYGON,
	GEO_POLYLINE,
	GEO_WALL,
	GEO_END,
};

enum CSG_OPERATION { CSG_UNION, CSG_INTERSECT, CSG_DIFF, CSG_SYM_DIFF };

namespace Ui
{
	class bdr3DGeometryEditPanel;
}

class bdr3DGeometryEditPanel : public ccOverlayDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdr3DGeometryEditPanel(QWidget* parent, ccPickingHub* pickingHub = nullptr);
	//! Destructor
	~bdr3DGeometryEditPanel() override;

	//! Get a pointer to the polyline that has been segmented
	ccPolyline *getPolyLine() {return m_segmentationPoly;}

	//! Returns whether hidden parts should be delete after segmentation
	bool deleteHiddenParts() const { return m_deleteHiddenParts; }
	
	//inherited from ccOverlayDialog
	bool linkWith(ccGLWindow* win) override;
	bool start() override;
	void stop(bool accepted) override;

	//! Remove entities from the 'to be segmented' pool
	/** \warning 'unallocateVisibilityArray' will be called on all point clouds
		prior to be removed from the pool.
	**/
	void removeAllEntities(bool unallocateVisibilityArrays);

	void clearChangedBaseObj() { m_changed_baseobj.clear(); }
	QSet<ccHObject*> getChangedBaseObj() { return m_changed_baseobj; }
	
	void setDestinationGroup(ccHObject* obj) { m_destination = obj; }

	void setActiveItem(std::vector<ccHObject*> active);

	void setModelObjects(std::vector<ccHObject*> builds);

	//! Returns the active 'to be segmented' set
	QSet<ccHObject*>& modelObjects() { return m_ModelObjs; }
	//! Returns the active 'to be segmented' set (const version)
	const QSet<ccHObject*>& modelObjects() const { return m_ModelObjs; }

private:
	Ui::bdr3DGeometryEditPanel	*m_UI;

	void echoUIchange();

	void createPlaneEditInterface();

protected slots:

	void addPointToPolyline(int x, int y);
	void closePolyLine(int x=0, int y=0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void closeRectangle();
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);
	void echoSelectChange(ccHObject* obj);

	void startEditingMode(bool);
	void confirmCreate();
	void pauseAll();
	
	void doBlock();
	void doBox();
	void doSphere();
	void doCone();
	void doCylinder();
	void doParapet();
	void doToroidCylinder();
	void doTorus();
	void doDish();
	void doPlane();
	void doPolyline();
	void doWall();

	void startEdit();
	void makeFreeMesh();

	void doCSGoperation(CSG_OPERATION operation);

	void planeBasedView();
	void objectBasedView();

	void onCurrentModelChanged(QString name);

	void reset();	// reset current editing state
	void exit();

	//! To capture overridden shortcuts (pause button, etc.)
	void onShortcutTriggered(int);

	//////////////////////////////////////////////////////////////////////////
	void echoBlockTopHeight(double v);
	void echoBlockBottomHeight(double v);
	//////////////////////////////////////////////////////////////////////////

protected:

	//inherited from QObject
	bool eventFilter(QObject *obj, QEvent *e) override;

	QToolButton* getGeoToolBottun(GEOMETRY3D g);

	void startGeoTool(GEOMETRY3D g);

	//! Whether to allow or not to exort the current segmentation polyline
	void allowExecutePolyline(bool state);

	void allowStateChange(bool state);

	void updateWithActive(ccHObject* obj);

	ccHObject* getActiveModel();

	//! Whether something has changed or not (for proper 'cancel')
	bool m_somethingHasChanged;

	//! Process states
	enum ProcessStates
	{
		POLYLINE		= 1,
		POLYGON			= 2,
		RECTANGLE		= 4,
		
		//EDITING		= 4,
		//...			= 8,
		//...			= 16,
		PAUSED			= 32,
		STARTED			= 64,
		RUNNING			= 128,
	};

	//! Current process state
	unsigned m_state;

	//! Segmentation polyline
	ccPolyline* m_segmentationPoly;
	//! Segmentation polyline vertices
	ccPointCloud* m_polyVertices;

	//! Selection mode
	enum SelectionMode
	{
		SELECT_3D,
		SELECT_2D,
	};
	SelectionMode m_selection_mode;
	bool m_rectangularSelection;

	//! Whether to delete hidden parts after segmentation
	bool m_deleteHiddenParts;
		
	ccHObject* m_destination;

	QSet<ccHObject*> m_changed_baseobj;

	QSet<ccHObject*> m_ModelObjs;

	std::vector<ccHObject*> m_actives;

	GEOMETRY3D m_current_editor;

	ccPlane* m_refPlane;

	ccPickingHub* m_pickingHub;

	bdrPlaneEditorDlg* m_refPlanePanel;
	bdrPlaneEditorDlg* m_toolPlanePanel;

};

#endif //BDR_3D_GEOMETRY_EDIT_PANEL_HEADER
