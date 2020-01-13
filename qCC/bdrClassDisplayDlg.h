#ifndef BDR_CLASS_DISPLAY_DLG_HEADER
#define BDR_CLASS_DISPLAY_DLG_HEADER

#include "ui_bdrClassDisplayDlg.h"

class ccGenericPointCloud;
class ccPointCloud;
class ccGLWindow;
class ccPlane;
class QToolButton;

namespace Ui
{
	class bdrClassDisplayDlg;
}

class bdrClassDisplayDlg : public QDialog
{
	Q_OBJECT

public:

	//! Default constructor
	explicit bdrClassDisplayDlg(QWidget* parent);
	//! Destructor
	~bdrClassDisplayDlg() override;

protected slots:

private: //members
	Ui::bdrClassDisplayDlg	*m_UI;
};

#endif //BDR_TRACE_FOOTPRINT_HEADER
