#include "StBlock.h"

//qCC_db
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccNormalVectors.h"
#include "ccHObjectCaster.h"
#include "ccPlane.h"

//CCLib
#include <Delaunay2dMesh.h>

//system
#include <string.h>
#include <iostream>

#include "vcg/space/intersection3.h"
#include "vcg/space/plane3.h"
#include "vcg/space/line3.h"

//////////////////////////////////////////////////////////////////////////

StBlock::StBlock(ccPlane* mainPlane,
	PointCoordinateType top_height,	CCVector3 top_normal, 
	PointCoordinateType bottom_height, CCVector3 bottom_normal,
	QString name)
	: m_mainPlane(mainPlane)
	, m_top_height(top_height)
	, m_top_normal(top_normal)
	, m_bottom_height(bottom_height)
	, m_bottom_normal(bottom_normal)
	, ccGenericPrimitive(name, nullptr/*&mainPlane->getTransformation()*/)
	, m_top_facet(nullptr)
	, m_bottom_facet(nullptr)
{
	if (m_mainPlane) {
		m_mainPlane->enableStippling(true);
	}
	if (!updateRepresentation()) {
		throw std::runtime_error("internal error");
	}
}

StBlock* StBlock::Create(const std::vector<CCVector3>& top,
	const PointCoordinateType bottom_height, QString name)
{
	bool bottom_is_lower_than_allpoints = true;
	bool bottom_is_heigher_than_allpoints = true;
	double min_height = FLT_MAX;
	for (auto pt : top)	{
		if (bottom_height >= pt.z) {
			bottom_is_lower_than_allpoints = false;
		}
		else {
			bottom_is_heigher_than_allpoints = false;
		}
		if (pt.z < min_height) {
			min_height = pt.z;
		}
	}
	PointCoordinateType bottom_height_valid(bottom_height);
	if (!bottom_is_heigher_than_allpoints && !bottom_is_lower_than_allpoints) {
		bottom_height_valid = min_height;
	}

	// for now the bottom should be lower
	assert(!bottom_is_heigher_than_allpoints);

	std::vector<CCVector3> profile_points;
	for (auto & pt : top) {
		profile_points.push_back(CCVector3(pt.x, pt.y, bottom_height_valid));
	}
	PointCoordinateType eq[4];
	eq[0] = 0; eq[1] = 0; eq[2] = 1; eq[3] = bottom_height_valid;
	ccPlane* mainPlane = ccPlane::Fit(profile_points, eq);
	if (!mainPlane) { return nullptr; }
	ccFacet* top_facet = ccFacet::CreateFromContour(top, "top", true);
	if (!top_facet) { delete mainPlane; return nullptr; }
		
	try	{
		StBlock* block = new StBlock(mainPlane, top_facet->getCenter().z - bottom_height_valid, top_facet->getNormal(),
			0, CCVector3(0, 0, 1), name);
		if (block) {
			delete top_facet;
			return block;
		}
		else throw std::runtime_error("internal error");
	}
	catch (...) {
		if (mainPlane) {
			delete mainPlane;
		}
	}
	
	return nullptr;
}

bool StBlock::buildFromFacet()
{
	if (!m_top_facet || !m_bottom_facet || !m_mainPlane) {
		return false;
	}

	std::vector<CCVector3> profile = m_mainPlane->getProfile();
	ccMesh* mesh = m_top_facet->getPolygon();

	if (!mesh) {
		ccLog::Warning(QString("[ccPlane::buildUp] Profile triangulation failed"));
		return false;
	}

	unsigned count = mesh->getAssociatedCloud()->size();
	unsigned numberOfTriangles = mesh->size();
	bool flip = false;

	if (numberOfTriangles == 0)
		return false;

	//vertices
	unsigned vertCount = 2 * count;
	//faces
	unsigned faceCount = 2 * numberOfTriangles + 2 * count;
	//faces normals
	unsigned faceNormCount = 2 + count;

	if (!init(vertCount, false, faceCount, faceNormCount))
	{
		ccLog::Error("[ccPlane::buildUp] Not enough memory");
		return false;
	}

	ccPointCloud* verts = vertices();
	assert(verts);
	assert(m_triNormals);

	verts->clear();
	m_triNormals->clear();

	bool outside = (m_top_height >= m_bottom_height);

	// top & bottom faces normals
	{
		if (outside) {
			m_triNormals->addElement(ccNormalVectors::GetNormIndex(m_top_facet->getNormal()));
			m_triNormals->addElement(ccNormalVectors::GetNormIndex(m_bottom_facet->getNormal()));
		}
		else {
			m_triNormals->addElement(ccNormalVectors::GetNormIndex(-m_top_facet->getNormal()));
			m_triNormals->addElement(ccNormalVectors::GetNormIndex(-m_bottom_facet->getNormal()));
		}
	}

	// if the facet is clockwise, need reverse
	bool reverse = false;
	{
		ccNormalVectors* normalv = ccNormalVectors::GetUniqueInstance();
		CCVector3 n0 = normalv->getNormal(m_triNormals->getValue(0));

		CCVector3 A, B, C;
		mesh->getTriangleVertices(0, A, B, C);
		CCVector3 N = (B - A).cross(C - A);
		N.normalize();
		if (N.dot(n0) < 0) {
			reverse = true;
		}
	}

	ccPointCloud* top_points = m_top_facet->getContourVertices();
	ccPointCloud* bottom_points = m_bottom_facet->getContourVertices();
	
	//add profile vertices & normals
	for (unsigned i = 0; i < count; ++i)
	{
		verts->addPoint(*top_points->getPoint(i));
		verts->addPoint(*bottom_points->getPoint(i));

// 		const CCVector2& P = CCVector2(profile[i].x, profile[i].y);	// TODO: Project to plane
// 		const CCVector2& PNext = CCVector2(profile[(i + 1) % count].x, profile[(i + 1) % count].y);
// 		CCVector2 N(PNext.y - P.y, -(PNext.x - P.x));
// 		N.normalize();
// 		m_triNormals->addElement(ccNormalVectors::GetNormIndex(CCVector3(N.x, N.y, 0.0).u));	// TODO: SHOULD CONSIDER MAINPLANE
	}

	//add faces
	{
		//side faces
		{
			int first(1), second(2);
			if (reverse) {
				first = 2;
				second = 1;
			}

			for (unsigned ti = 0; ti < numberOfTriangles; ++ti) {
				unsigned int* _triIndexes = mesh->getTriangleVertIndexes(ti)->i;
				
				addTriangle(_triIndexes[0] * 2, _triIndexes[first] * 2, _triIndexes[second] * 2);
				addTriangleNormalIndexes(0, 0, 0);

				addTriangle(_triIndexes[0] * 2 + 1, _triIndexes[second] * 2 + 1, _triIndexes[first] * 2 + 1);
				addTriangleNormalIndexes(1, 1, 1);
			}
		}

		//thickness
		{
			for (unsigned i = 0; i < count; ++i)
			{
				unsigned iNext = ((i + 1) % count);

				int first = i * 2 + 1;
				int second = iNext * 2;
				if (reverse) {
					std::swap(first, second);
				}		

				CCVector3 A, B, C;
				A = *verts->getPoint(i * 2);
				B = *verts->getPoint(first);
				C = *verts->getPoint(second);

				CCVector3 N = (B - A).cross(C - A); N.normalize();
				m_triNormals->addElement(ccNormalVectors::GetNormIndex(N));
				
				addTriangle(i * 2, first, second);
				addTriangleNormalIndexes(2 + i, 2 + i, 2 + i);

				addTriangle(second, first, iNext * 2 + 1);
				addTriangleNormalIndexes(2 + i, 2 + i, 2 + i);
			}
		}
	}
	setVisible(true);
	enableStippling(false);
	showNormals(true);

	return true;
}

void StBlock::updateFacet(ccFacet * facet)
{
	//! update normal and height
	if (facet == m_top_facet || facet == m_bottom_facet) {
		paramFromFacet();
	}
	else {
		return;
	}
	
	//! reset facet
	buildUp();
}

void StBlock::setFacetPoints(ccFacet * facet, std::vector<CCVector3> points, bool computePlane)
{
	if (!(facet == m_top_facet) && !(facet == m_bottom_facet)) {
		return;
	}
	facet->FormByContour(points, true, computePlane ? nullptr : facet->getPlaneEquation());

	//! change the mainPlane
	std::vector<CCVector3> plane_points = m_mainPlane->projectTo3DGlobal(points);
	m_mainPlane->setProfile(plane_points, true);

	if (facet == m_top_facet) {
		std::vector<CCVector3> bottom_points;

		std::vector<CCVector3> profiles = m_mainPlane->getProfile();
		CCVector3 plane_normal = m_mainPlane->getNormal();

		vcg::Plane3d bot_plane;
		CCVector3 bot_center = getBottomCenter();
		CCVector3 bot_normal = m_bottom_facet ? m_bottom_normal : -plane_normal;
		bot_plane.Init({ bot_center.x,bot_center.y,bot_center.z }, { bot_normal.x,bot_normal.y,bot_normal.z });

		for (auto & pt : profiles) {
			vcg::Line3d line;
			line.SetOrigin({ pt.x, pt.y, pt.z });
			line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

			vcg::Point3d facet_pt;
			if (!vcg::IntersectionLinePlane(line, bot_plane, facet_pt)) {
				return;
			}
			bottom_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
		}
		if (!m_bottom_facet) {
			ccFacet* facet = new ccFacet(0, "bottom");
			setBottomFacet(facet);
		}
		m_bottom_facet->FormByContour(bottom_points, true);
		m_bottom_facet->invertNormal();
	}
	else if (facet == m_bottom_facet) {
		std::vector<CCVector3> top_points;

		std::vector<CCVector3> profiles = m_mainPlane->getProfile();
		CCVector3 plane_normal = m_mainPlane->getNormal();

		vcg::Plane3d top_plane;
		CCVector3 top_center = getTopCenter();
		CCVector3 top_normal = m_top_facet ? m_top_normal : plane_normal;
		top_plane.Init({ top_center.x,top_center.y,top_center.z }, { top_normal.x,top_normal.y,top_normal.z });

		for (auto & pt : profiles) {
			vcg::Line3d line;
			line.SetOrigin({ pt.x, pt.y, pt.z });
			line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

			vcg::Point3d facet_pt;
			if (!vcg::IntersectionLinePlane(line, top_plane, facet_pt)) {
				return;
			}
			top_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
		}
		if (!m_top_facet) {
			ccFacet* facet = new ccFacet(0, "top");
			setTopFacet(facet);
		}
		m_top_facet->FormByContour(top_points, true);
	}

	paramFromFacet();
	buildFromFacet();
}

bool StBlock::getWallPolygons(std::vector<std::vector<CCVector3>>& walls)
{	
	//! roof
	std::vector<CCVector3> roof;
	ccPointCloud* top_points = m_top_facet->getContourVertices();
	int count = top_points->size();
	if (count < 3) return false;

	for (int i = 0; i < top_points->size(); ++i) {
		roof.push_back(*top_points->getPoint(i));
	}

	//! bottom
	std::vector<CCVector3> bottom;
	ccPointCloud* bottom_points = m_bottom_facet->getContourVertices();
	if (bottom_points->size() != count) return false;
	for (int i = 0; i < bottom_points->size(); ++i) {
		bottom.push_back(*bottom_points->getPoint(i));
	}
	
	//! facades
	std::vector<std::vector<CCVector3>> facades;
	for (int i = 0; i < count; ++i) {
		std::vector<CCVector3> facade;
		facade.push_back(roof[i]);
		facade.push_back(bottom[i]);
		facade.push_back(bottom[(i + 1) % count]);
		facade.push_back(roof[(i + 1) % count]);
		facades.push_back(facade);
	}

	std::reverse(bottom.begin(), bottom.end());
	walls.push_back(roof);
	walls.push_back(bottom);
	walls.insert(walls.end(), facades.begin(), facades.end());

	return true;
}

bool StBlock::buildUp()
{
	//! firstly, deduce top facet and bottom facet from height and normal
	std::vector<CCVector3> top_points = deduceTopPoints();
	std::vector<CCVector3> bottom_points = deduceBottomPoints();

	if (top_points.size() < 3 || bottom_points.size() < 3) { return false; }

	if (m_top_facet && m_bottom_facet) {
		//! change the facet contours
		m_top_facet->FormByContour(top_points, true);
		m_bottom_facet->FormByContour(bottom_points, true);
	}
	else {
		if (m_top_facet) { delete m_top_facet; m_top_facet = nullptr; }
		if (m_bottom_facet) { delete m_bottom_facet; m_bottom_facet = nullptr; }
		ccFacet* top = ccFacet::CreateFromContour(top_points, "top", true);
		setTopFacet(top);
		ccFacet* bottom = ccFacet::CreateFromContour(bottom_points, "bottom", true);
		setBottomFacet(bottom);
	}
	if (!m_top_facet || !m_bottom_facet) {
		return false;
	}
	if (m_bottom_height <= m_top_height) {
		m_bottom_facet->invertNormal();
	}
	else m_top_facet->invertNormal();
 	
	return buildFromFacet();
}

void StBlock::applyGLTransformation(const ccGLMatrix & trans)
{
	//! main plane
	m_mainPlane->applyGLTransformation_recursive(&trans);

	//! for mesh
	ccGenericPrimitive::applyGLTransformation(trans);

	//! for m_top_normal
	m_top_normal = getTopFacet()->getNormal();

	//! for m_bottom_normal
	m_bottom_normal = getBottomFacet()->getNormal();
}

void StBlock::paramFromFacet()
{
	if (!m_top_facet || !m_bottom_facet || !m_mainPlane) {
		return;
	}
	CCVector3 plane_normal = getNormal(); plane_normal.normalize();
	m_top_height = (m_top_facet->getCenter() - m_mainPlane->getProfileCenter()).dot(plane_normal);
	m_top_normal = m_top_facet->getNormal();
	m_bottom_height = (m_bottom_facet->getCenter() - m_mainPlane->getProfileCenter()).dot(plane_normal);
	m_bottom_normal = m_bottom_facet->getNormal();
}

CCVector3 StBlock::getTopCenter()
{
	return m_mainPlane->getProfileCenter() + m_mainPlane->getNormal() * m_top_height;
}

CCVector3 StBlock::getBottomCenter()
{
	return m_mainPlane->getProfileCenter() + m_mainPlane->getNormal() * m_bottom_height;
}

StBlock::StBlock(QString name/*="Block"*/)
	: ccGenericPrimitive(name)
	, m_mainPlane(nullptr)
	, m_top_facet(nullptr)
	, m_bottom_facet(nullptr)
{
}

ccGenericPrimitive* StBlock::clone() const
{
	ccPlane* clonePlane = ccHObjectCaster::ToPlane(m_mainPlane->clone());
	if (!clonePlane) {
		return nullptr;
	}
	return finishCloneJob(new StBlock(clonePlane, m_top_height, m_top_normal, m_bottom_height, m_bottom_normal, getName()));
}

ccFacet * StBlock::getTopFacet()
{
	return m_top_facet;
}

void StBlock::setTopFacet(ccFacet * facet)
{
	if (m_top_facet && m_top_facet != facet) {
		removeChild(m_top_facet);
// 		delete m_top_facet;
// 		m_top_facet = nullptr;
	}
	m_top_facet = facet;
	if (m_top_facet && !m_top_facet->isAncestorOf(this) && !m_top_facet->getParent()) {
		addChild(m_top_facet);
	//	m_top_facet->addDependency(this, ccHObject::DP_NOTIFY_OTHER_ON_DELETE | ccHObject::DP_NOTIFY_OTHER_ON_UPDATE);
	}
}

ccFacet * StBlock::getBottomFacet()
{
	return m_bottom_facet;
}

void StBlock::setBottomFacet(ccFacet * facet)
{
	if (m_bottom_facet && m_bottom_facet != facet) {
		removeChild(m_bottom_facet);
// 		delete m_bottom_facet;
// 		m_bottom_facet = nullptr;
	}
	m_bottom_facet = facet;
	if (m_bottom_facet && !m_bottom_facet->isAncestorOf(this)) {
		addChild(m_bottom_facet);
	//	m_bottom_facet->addDependency(this, ccHObject::DP_NOTIFY_OTHER_ON_DELETE | ccHObject::DP_NOTIFY_OTHER_ON_UPDATE);
	}
}

void StBlock::setTopHeight(double val)
{
	double add = val - m_top_height;
	m_top_height = val;

	buildUp();
	return;

	//! get top points
	std::vector<CCVector3> top_points;
	ccPointCloud* verts = vertices();
	if (!verts) { return; }
	for (unsigned int i = 0; i < verts->size() / 2; i++) {
		CCVector3& P = const_cast<CCVector3&>(*verts->getPoint(i * 2));
		P += m_mainPlane->getNormal() * add;
		top_points.push_back(P);
	}
	verts->invalidateBoundingBox();

	ccFacet* top = getTopFacet();
	if (!top) return;
	top->FormByContour(top_points, true);
}

void StBlock::setBottomHeight(double val)
{
	double add = val - m_bottom_height;
	m_bottom_height = val;

	buildUp();
	return;

	//! get bottom points
	std::vector<CCVector3> bot_points;
	ccPointCloud* verts = vertices();
	if (!verts) { return; }
	for (unsigned int i = 0; i < verts->size() / 2; i++) {
		CCVector3& P = const_cast<CCVector3&>(*verts->getPoint(i * 2 + 1));
		P += m_mainPlane->getNormal() * add;
		bot_points.push_back(P);
	}
	verts->invalidateBoundingBox();

	ccFacet* bottom = getBottomFacet();
	if (!bottom) return;
	bottom->FormByContour(bot_points, true);
	bottom->invertNormal();
}

std::vector<CCVector3> StBlock::deduceTopPoints()
{
	std::vector<CCVector3> top_points;
	if (!m_mainPlane) {
		return top_points;
	}

	std::vector<CCVector3> profiles = m_mainPlane->getProfile();
	CCVector3 plane_normal = m_mainPlane->getNormal();
	if (profiles.size() < 3) { return top_points; }

	vcg::Plane3d top_plane;
	CCVector3 top_center = getTopCenter();
	top_plane.Init({ top_center.x,top_center.y,top_center.z }, { m_top_normal.x,m_top_normal.y,m_top_normal.z });

	for (auto & pt : profiles) {
		vcg::Line3d line;
		line.SetOrigin({ pt.x, pt.y, pt.z });
		line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

		vcg::Point3d facet_pt;
		if (!vcg::IntersectionLinePlane(line, top_plane, facet_pt)) {
			top_points.clear(); return top_points;
		}
		top_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
	}
	return top_points;
}

std::vector<CCVector3> StBlock::deduceBottomPoints()
{
	std::vector<CCVector3> bottom_points;
	if (!m_mainPlane) {
		return bottom_points;
	}

	std::vector<CCVector3> profiles = m_mainPlane->getProfile();
	CCVector3 plane_normal = m_mainPlane->getNormal();
	if (profiles.size() < 3) { return bottom_points; }

	vcg::Plane3d bot_plane;
	CCVector3 bot_center = getBottomCenter();
	bot_plane.Init({ bot_center.x,bot_center.y,bot_center.z }, { m_bottom_normal.x,m_bottom_normal.y,m_bottom_normal.z });

	for (auto & pt : profiles) {
		vcg::Line3d line;
		line.SetOrigin({ pt.x, pt.y, pt.z });
		line.SetDirection({ plane_normal.x,plane_normal.y,plane_normal.z });

		vcg::Point3d facet_pt;
		if (!vcg::IntersectionLinePlane(line, bot_plane, facet_pt)) {
			bottom_points.clear();
			return bottom_points;
		}
		bottom_points.push_back(CCVector3(facet_pt.X(), facet_pt.Y(), facet_pt.Z()));
	}
	return bottom_points;
}

inline CCVector3 StBlock::getNormal() const
{
	return m_mainPlane->getNormal();
}

CCVector3 StBlock::getCenter() const
{
	return m_mainPlane->getCenter();
}

void StBlock::applyPlanarEntityChange(ccGLMatrix mat)
{
	//! main plane
	m_mainPlane->applyGLTransformation_recursive(&mat);

	//! for mesh
	applyGLTransformation_recursive(&mat);

	//! for m_top_normal
	m_top_normal = getTopFacet()->getNormal();

	//! for m_bottom_normal
	m_bottom_normal = getBottomFacet()->getNormal();
}

void StBlock::normalEditState(bool edit)
{ 
	m_editable = edit; 
	m_mainPlane->normalEditState(edit);
}

void StBlock::getEquation(CCVector3 & N, PointCoordinateType & constVal) const
{
	m_mainPlane->getEquation(N, constVal);
}

//! onUpdateOf(ccFacet)

void StBlock::drawMeOnly(CC_DRAW_CONTEXT & context)
{
	if (isVisible()) {
		ccGenericPrimitive::drawMeOnly(context);
	}
	if (getNormalEditState() && m_mainPlane && MACRO_Draw3D(context)) {
		m_mainPlane->draw(context);
	}
}

bool StBlock::toFile_MeOnly(QFile& out) const
{
	if (!ccGenericPrimitive::toFile_MeOnly(out))
		return WriteError();

	//we can't save the associated facet here
	//so instead we save it's unique ID (dataVersion>=20)
	//WARNING: the cloud must be saved in the same BIN file! (responsibility of the caller)
	uint32_t topUniqueID = (m_top_facet ? static_cast<uint32_t>(m_top_facet->getUniqueID()) : 0);
	if (out.write((const char*)&topUniqueID, 4) < 0)
		return WriteError();

	uint32_t botUniqueID = (m_bottom_facet ? static_cast<uint32_t>(m_bottom_facet->getUniqueID()) : 0);
	if (out.write((const char*)&botUniqueID, 4) < 0)
		return WriteError();

	//! top
	if (out.write((const char*)m_top_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return WriteError();
	if (out.write((const char*)&m_top_height, sizeof(double)) < 0)
		return WriteError();

	//! bottom
	if (out.write((const char*)m_bottom_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return WriteError();
	if (out.write((const char*)&m_bottom_height, sizeof(double)) < 0)
		return WriteError();

	//! plane
	if (!m_mainPlane->toFile(out)) {
		return WriteError();
	}

	return true;
}

bool StBlock::fromFile_MeOnly(QFile& in, short dataVersion, int flags)
{
	if (!ccGenericPrimitive::fromFile_MeOnly(in, dataVersion, flags))
		return ReadError();

	//as the associated facet can't be saved directly
	//we only store its unique ID (dataVersion>=20) --> we hope we will find it at loading time (i.e. this
	//is the responsibility of the caller to make sure that all dependencies are saved together)
	uint32_t vertUniqueID = 0;
	if (in.read((char*)&vertUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_top_facet' pointer!!!
	*(uint32_t*)(&m_top_facet) = vertUniqueID;

	if (in.read((char*)&vertUniqueID, 4) < 0)
		return ReadError();
	//[DIRTY] WARNING: temporarily, we set the vertices unique ID in the 'm_bottom_facet' pointer!!!
	*(uint32_t*)(&m_bottom_facet) = vertUniqueID;

	//! top
	if (in.read((char*)m_top_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return ReadError();
	if (in.read((char*)&m_top_height, sizeof(double)) < 0)
		return ReadError();

	//! bottom
	if (in.read((char*)m_bottom_normal.u, sizeof(PointCoordinateType) * 3) < 0)
		return ReadError();
	if (in.read((char*)&m_bottom_height, sizeof(double)) < 0)
		return ReadError();

	//! plane fake loading, the associate point cloud is not saved
	CC_CLASS_ENUM classID = ReadClassIDFromFile(in, dataVersion);
	if (classID != CC_TYPES::PLANE) {
		return CorruptError();
	}

 	ccPlane* plane = new ccPlane;
 	if (!plane->fromFile(in, dataVersion, flags)) {
 		return ReadError();
 	}
 	m_mainPlane = new ccPlane(plane->getXWidth(), plane->getYWidth(), &plane->getTransformation(), plane->getName());
	m_mainPlane->setProfile(plane->getProfile());
 
 	plane->setAssociatedCloud(0);
 	plane->setTriNormsTable(0, false);
 	plane->setTexCoordinatesTable(0, false);
 	delete plane;
 	plane = nullptr;

	return true;
}
