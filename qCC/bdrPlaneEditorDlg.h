#ifndef BDR_PLANE_EDITOR_DLG_HEADER
#define BDR_PLANE_EDITOR_DLG_HEADER

//Local
#include <ui_bdrPlaneEditorDlg.h>
#include "ccPickingListener.h"

//CCLib
#include <CCGeom.h>

//Qt
#include <QDialog>

class ccGLWindow;
class ccPlane;
class ccHObject;
class ccPickingHub;
class ccPlanarEntityInterface;

//! Dialog to create (or edit the parameters) of a plane
class bdrPlaneEditorDlg : public QDialog, public ccPickingListener, public Ui::BDRPlaneEditorDlg
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrPlaneEditorDlg(ccPickingHub* pickingHub, QWidget* parent);

	//! Destructor
	virtual ~bdrPlaneEditorDlg();

	void updateParams();

	//! Links this dialog with an existing plane
	void initWithPlane(ccPlanarEntityInterface* plane);

	void disconnectPlane();

	//! Updates a plane with the current parameters
	void updatePlane(ccPlanarEntityInterface* plane);

	//! Inherited from ccPickingListener
	virtual void onItemPicked(const PickedItem& pi);
	
	void setNormal(CCVector3 n);
	CCVector3 getNormal() const;
	void setCenter(CCVector3 c);
	CCVector3 getCenter() const;

	enum DisplayState {
		DISPLAY_NONE,
		DISPLAY_PLANE,
		DISPLAY_EDITOR,
	};
	void setDisplayState(DisplayState state);
	DisplayState getDisplayState() const { return m_display_state; }

public slots:

	void pickPointAsCenter(bool);
	void onCenterChanged(double);
	void onNormalChanged(double);
	void onDimensionChanged(double);
	void saveParamsAndAccept();
	void restore();

	void cancle();

protected slots:

	void preview();
	void updateUI();

protected:
	void closeEvent(QCloseEvent *) override;

protected: //members

	//! to store the initial plane parameters
	struct planeParams {
		CCVector3 normal;
		CCVector3 center;
		CCVector2 size;
		bool already_in_db;

		void reset() {
			normal = CCVector3(0, 0, 0);
			center = CCVector3(0, 0, 0);
			size = CCVector2(0, 0);
			already_in_db = false;
		}
	};
	planeParams m_planePara;

	//! Picking window (if any)
	ccGLWindow* m_pickingWin;

	//! Associated plane (if any)
	ccPlanarEntityInterface* m_associatedPlane;

	//! Picking hub
	ccPickingHub* m_pickingHub;

	DisplayState m_display_state;
};

#endif
