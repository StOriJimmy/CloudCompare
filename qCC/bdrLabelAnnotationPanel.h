#ifndef BDR_LABEL_ANNOTATION_PANEL_HEADER
#define BDR_LABEL_ANNOTATION_PANEL_HEADER

//Local
#include <ccOverlayDialog.h>

//qCC_db
#include <ccHObject.h>

//Qt
#include <QSet>

//GUI
#include <ui_bdrLabelAnnotationPanel.h>

class ccPolyline;
class ccPointCloud;
class ccGLWindow;

namespace Ui
{
	class bdrLabelAnnotationPanel;
}

class bdrLabelAnnotationPanel : public ccOverlayDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrLabelAnnotationPanel(QWidget* parent);
	//! Destructor
	~bdrLabelAnnotationPanel() override;

	//! Adds an entity (and/or its children) to the 'to be segmented' pool
	/** Warning: some entities may be rejected if they are
		locked, or can't be segmented this way.
		\return whether entity has been added to the pool or not
	**/
	bool addEntity(ccHObject* anObject);
	
	//! Returns the number of entites currently in the the 'to be segmented' pool
	unsigned getNumberOfValidEntities() const;

	//! Get a pointer to the polyline that has been segmented
	ccPolyline *getPolyLine() {return m_segmentationPoly;}

	//! Returns the active 'to be segmented' set
	QSet<ccHObject*>& entities() { return m_toSegment; }
	//! Returns the active 'to be segmented' set (const version)
	const QSet<ccHObject*>& entities() const { return m_toSegment; }

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

	enum SegmentMode
	{
		SEGMENT_GENERAL,
		SEGMENT_PLANE_CREATE,
		SEGMENT_PLANE_SPLIT,
		SEGMENT_LABELING,
		SEGMENT_BUILD_EIDT,
	};
	void setSegmentMode(SegmentMode mode);
	int getSegmentMode() {	return m_segment_mode; }

	void setDestinationGroup(ccHObject* obj) { m_destination = obj; }
private:
	Ui::bdrLabelAnnotationPanel	*m_UI;

protected slots:

	void segmentIn();
	void segmentOut();
	void segment(bool);
	
	void addPointToPolyline(int x, int y);
	void closePolyLine(int x=0, int y=0); //arguments for compatibility with ccGlWindow::rightButtonClicked signal
	void closeRectangle();
	void updatePolyLine(int x, int y, Qt::MouseButtons buttons);

	void settings();

	void pauseLabelingMode(bool);
	void filterClassification();

	void setLabel();
	void createEntity();

	void reset();
	void exit();

	void doSet2DSelection();
	void doSet3DSelection();

	void onLasLabelChanged(int);

	void onLasLabelActivated();

	//! To capture overridden shortcuts (pause button, etc.)
	void onShortcutTriggered(int);

protected:

	//! Whether to allow or not to exort the current segmentation polyline
	void allowExecutePolyline(bool state);

	void allowStateChange(bool state);

	//! Set of entities to be segmented
	QSet<ccHObject*> m_toSegment;

	//! Whether something has changed or not (for proper 'cancel')
	bool m_somethingHasChanged;

	//! Process states
	enum ProcessStates
	{
		POLYLINE		= 1,
		RECTANGLE		= 2,
		//...			= 4,
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
		
	SegmentMode m_segment_mode;

	ccHObject* m_destination;

	QSet<ccHObject*> m_changed_baseobj;
};

#endif //BDR_LABEL_ANNOTATION_PANEL_HEADER
