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

#ifndef ST_BLOCK_HEADER
#define ST_BLOCK_HEADER

//Local
#include "ccGenericPrimitive.h"
#include "ccFacet.h"
#include "ccPlanarEntityInterface.h"

class ccPlane;

// block
class QCC_DB_LIB_API StBlock : public ccGenericPrimitive, public ccPlanarEntityInterface
{
public:

	//! Default constructor
	/** Input (2D) polyline represents the profile in (X,Y) plane.
		Extrusion is always done ine the 'Z' dimension.
		\param profile 2D profile to extrude
		\param height extrusion thickness
		\param transMat optional 3D transformation (can be set afterwards with ccDrawableObject::setGLTransformation)
		\param name name
	**/


	StBlock(ccPlane * mainPlane, 
		PointCoordinateType top_height, 
		CCVector3 top_normal, 
		PointCoordinateType bottom_height,
		CCVector3 bottom_normal,
		QString name = QString("Prism"));

	// 2.5D
	static StBlock * Create(const std::vector<CCVector3>& top,
		const PointCoordinateType bottom_height,
		QString name = QString("Prism"));

	//! Simplified constructor
	/** For ccHObject factory only!
	**/
	StBlock(QString name = QString("Prism"));

	//! Returns class ID
	virtual CC_CLASS_ENUM getClassID() const override { return CC_TYPES::ST_BLOCK; }

	//inherited from ccGenericPrimitive
	virtual QString getTypeName() const override { return "Prism"; }
	virtual ccGenericPrimitive* clone() const override;
	
	ccPlane* getMainPlane() { return m_mainPlane; }

	ccFacet* getTopFacet();
	void setTopFacet(ccFacet* facet);
	ccFacet* getBottomFacet();
	void setBottomFacet(ccFacet* facet); 

	void setTopHeight(double val);
	double getTopHeight() { return m_top_height; }
	void setBottomHeight(double val);
	double getBottomHeight() { return m_bottom_height; }
	
	void updateFacet(ccFacet* facet);
	void setFacetPoints(ccFacet* facet, std::vector<CCVector3> points, bool computePlane);

	// roof -> bottom -> facades
	bool getWallPolygons(std::vector<std::vector<CCVector3>>& walls);

	bool isHole() { return m_top_height < m_bottom_height; }

protected:

	//inherited from ccDrawable
	virtual void drawMeOnly(CC_DRAW_CONTEXT& context) override;

	//inherited from ccGenericPrimitive
	virtual bool toFile_MeOnly(QFile& out) const override;
	virtual bool fromFile_MeOnly(QFile& in, short dataVersion, int flags) override;
	virtual bool buildUp() override;
	
	//inherited from ccHObject
	virtual void applyGLTransformation(const ccGLMatrix& trans) override;

	void paramFromFacet();
	bool buildFromFacet();

	std::vector<CCVector3> deduceTopPoints();
	std::vector<CCVector3> deduceBottomPoints();

	ccFacet* m_top_facet;
	ccFacet* m_bottom_facet;

protected:
	ccPlane* m_mainPlane; // center, normal, profile
	
	//! top plane center=m_mainPlane.center+m_mainPlane.normal*m_top_height
	CCVector3 getTopCenter(); 
	double m_top_height;	
	CCVector3 m_top_normal;

	//! bottom plane center = m_mainPlane.center + m_mainPlane.normal*m_bottom_height
	CCVector3 getBottomCenter(); 
	double m_bottom_height;
	CCVector3 m_bottom_normal;

public:
	//inherited from ccPlanarEntityInterface //! for planar entity
	ccHObject* getPlane() override { return this; }
	//inherited from ccPlanarEntityInterface
	inline CCVector3 getNormal() const override;
	//inherited from ccPlanarEntityInterface //! Returns the facet center
	CCVector3 getCenter() const override;
	//inherited from ccPlanarEntityInterface
	void applyPlanarEntityChange(ccGLMatrix mat) override;
	void normalEditState(bool edit) override;

	void getEquation(CCVector3& N, PointCoordinateType& constVal) const override;

};

#endif //ST_BLOCK_HEADER
