#include "stocker_parser.h"

#include "ccHObject.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccPlane.h"
#include "ccFacet.h"
#include "ccHObjectCaster.h"
#include "ccDBRoot.h"
#include "ccColorScalesManager.h"

#include "QFileInfo"
#include <QImageReader>
#include <QFileDialog>
#include "FileIOFilter.h"

#ifdef USE_STOCKER
#include "builderlod2/lod2parser.h"
#include "builderpoly/builderpoly.h"
#include "buildertexture/buildertexture.h"
#include "vcg/space/index/grid_static_ptr2d.h"
#include "vcg/space/index/closest2d.h"

using namespace stocker;
#endif // USE_STOCKER

#ifdef PCATPS_SUPPORT
#include "PC_ATPS.h"
#endif
#include "stockerDatabase.h"

template <typename T1, typename T2>
auto ccToPoints2(std::vector<T1> points, bool parallel = false)->std::vector<T2>
{
	std::vector<T2> pointsTO;
	if (parallel) {
		Concurrency::concurrent_vector<T2> points_To_Conc;
		Concurrency::parallel_for((size_t)0, points.size(), [&](size_t i) {
			points_To_Conc.push_back(T2(points[i].x, points[i].y));
		});
		pointsTO.assign(points_To_Conc.begin(), points_To_Conc.end());
	}
	else {
		for (auto & pt : points) {
			pointsTO.emplace_back(pt.x, pt.y);
		}
	}	
	return pointsTO;
}

template <typename T1, typename T2>
auto ccToPoints3(std::vector<T1> points, bool parallel = false)->std::vector<T2>
{
	std::vector<T2> pointsTO;
	if (parallel) {
		Concurrency::concurrent_vector<T2> points_To_Conc;
		Concurrency::parallel_for((size_t)0, points.size(), [&](size_t i) {
			points_To_Conc.push_back(T2(points[i].x, points[i].y, points[i].z));
		});
		pointsTO.assign(points_To_Conc.begin(), points_To_Conc.end());
	}
	else {
		for (auto & pt : points) {
			pointsTO.emplace_back(pt.x, pt.y, pt.z);
		}
	}
	return pointsTO;
}

template <typename T = stocker::Vec3d>
auto GetPointsFromCloud3d(ccHObject* entity, bool global)->std::vector<T>
{	
	std::vector<T> points;
	if (!entity) return points;
	if (entity->isA(CC_TYPES::POINT_CLOUD)) {
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (!cloud) return points;
		for (unsigned i = 0; i < cloud->size(); i++) {
			const CCVector3* pt = cloud->getPoint(i);
			if (global) {
				CCVector3d Pglobal = cloud->toGlobal3d<PointCoordinateType>(*pt);
				points.push_back({ Pglobal.x, Pglobal.y, Pglobal.z });
			}
			else {
				points.push_back({ (*pt).x, (*pt).y, (*pt).z });
			}
		}
	}
	else if (entity->isA(CC_TYPES::ST_PRIMGROUP)) {
		StPrimGroup* primGroup = ccHObjectCaster::ToStPrimGroup(entity);
		if (!primGroup) return points;				
		ccHObject::Container plane_container = primGroup->getValidPlanes();
		for (auto & pl : plane_container) {
			std::vector<T> cur_points = GetPointsFromCloud3d<T>(pl->getParent(), global);
			points.insert(points.end(), cur_points.begin(), cur_points.end());
		}
	}
	
	return points;
}
template <typename T = vcg::Point3f>
auto GetPointsFromCloud3f(ccHObject* entity, bool global)->std::vector<T>
{
	std::vector<T> points;
	if (!entity) return points;
	if (entity->isA(CC_TYPES::POINT_CLOUD)) {
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (!cloud) return points;

		for (unsigned i = 0; i < cloud->size(); i++) {
			const CCVector3* pt = cloud->getPoint(i);
			CCVector3 Pglobal = cloud->toGlobal3pc<PointCoordinateType>(*pt);
			points.push_back({ Pglobal.x, Pglobal.y, Pglobal.z });
		}
	}
	else if (entity->isA(CC_TYPES::ST_PRIMGROUP)) {
		StPrimGroup* primGroup = ccHObjectCaster::ToStPrimGroup(entity);
		if (!primGroup) return points;
		ccHObject::Container plane_container = primGroup->getValidPlanes();
		for (auto & pl : plane_container) {
			std::vector<T> cur_points = GetPointsFromCloud3f<T>(pl->getParent(), global);
			points.insert(points.end(), cur_points.begin(), cur_points.end());
		}
	}

	return points;
}

bool GetPointsFromCloud(ccHObject* entity, stocker::Contour3d &global, stocker::Contour3f &local)
{
	if (!entity) { return false; }
	if (entity->isA(CC_TYPES::POINT_CLOUD)) {
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (!cloud) return false;

		for (unsigned i = 0; i < cloud->size(); i++) {
			const CCVector3* pt = cloud->getPoint(i);
			local.push_back({ pt->x, pt->y, pt->z });
			CCVector3d Pglobal = cloud->toGlobal3d<PointCoordinateType>(*pt);
			global.push_back({ Pglobal.x, Pglobal.y, Pglobal.z });
		}
	}
	else if (entity->isA(CC_TYPES::ST_PRIMGROUP)) {
		StPrimGroup* primGroup = ccHObjectCaster::ToStPrimGroup(entity);
		if (!primGroup) return false;
		ccHObject::Container plane_container = primGroup->getValidPlanes();
		for (auto & pl : plane_container) {
			Contour3d global_; Contour3f local_;
			if (GetPointsFromCloud(GetPlaneCloud(pl), global_, local_)) {
				global.insert(global.end(), global_.begin(), global_.end());
				local.insert(local.end(), local_.begin(), local_.end());
			}
		}
	}
	return true;
}

double GetPointsAverageSpacing(ccHObject * pc)
{
	BDBaseHObject* bd_grp = GetRootBDBase(pc);
	StBuilding* bd = GetParentBuilding(pc);
	if (bd_grp && bd) {
		stocker::BuildUnit bd_unit = bd_grp->GetBuildingUnit(bd->getName().toStdString());
		if (bd_unit.GetName().Str() != "invalid" && bd_unit.average_spacing > 0) {
			return bd_unit.average_spacing;
		}
	}
	
	stocker::Contour3f points_local = GetPointsFromCloud3f<Vec3f>(pc, false);
	return points_local.size() >= 3 ? stocker::ComputeAverageSpacing3f(points_local, true) : 0.0f;
}

stocker::Contour3d GetPointsFromCloudInsidePolygonXY(ccHObject* entity, stocker::Polyline3d polygon, double height)
{	
	stocker::Contour3d points;
	if (entity->isA(CC_TYPES::HIERARCHY_OBJECT)) {
		ccHObject::Container planes_container, plane_cloud_container;
		entity->filterChildren(planes_container, true, CC_TYPES::PLANE, true);
		for (ccHObject* pl_obj : planes_container) {
			if (pl_obj->getParent() && pl_obj->getParent()->isEnabled()) {
				plane_cloud_container.push_back(pl_obj->getParent());
			}			
		}
		std::vector<Contour3d> planes_points = GetPointsFromCloudInsidePolygonsXY(plane_cloud_container, polygon, height);
		for (auto & pl_pts : planes_points) {
			points.insert(points.end(), pl_pts.begin(), pl_pts.end());
		}
		return points;
	}
	else if (!entity->isA(CC_TYPES::POINT_CLOUD)) return points;

	if (polygon.empty()) {
		return GetPointsFromCloud3d(entity);
	}

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!cloud) return points;	

	std::vector<vcg::Segment2d> polygon_2d;
	double min_polygon_height = DBL_MAX;
	for (auto & seg : polygon) {
		polygon_2d.push_back(vcg::Segment2d(ToVec2d(seg.P0()), ToVec2d(seg.P1())));
		if (seg.P0().Z() < min_polygon_height) { min_polygon_height = seg.P0().Z(); }
		if (seg.P1().Z() < min_polygon_height) { min_polygon_height = seg.P1().Z(); }
	}
	
	Concurrency::concurrent_vector<Vec3d> points_parallel;
	if (height > min_polygon_height) {
		Concurrency::parallel_for((size_t)0, (size_t)cloud->size(), [&](size_t i) {
			CCVector3 pt = *cloud->getPoint(i);
			if (vcg::PointInsidePolygon({ pt.x,pt.y }, polygon_2d) && pt.z < height && pt.z > min_polygon_height) {
				points_parallel.push_back({ pt.x, pt.y, pt.z });
			}
		});
	}
	else {
		Concurrency::parallel_for((size_t)0, (size_t)cloud->size(), [&](size_t i) {
			CCVector3 pt = *cloud->getPoint(i);
			if (vcg::PointInsidePolygon({ pt.x,pt.y }, polygon_2d)) {
				points_parallel.push_back({ pt.x, pt.y, pt.z });
			}
		});
	}
	
	points.assign(points_parallel.begin(), points_parallel.end());
	return points;
}

std::vector<stocker::Contour3d> GetPointsFromCloudInsidePolygonsXY(ccHObject::Container entities, stocker::Polyline3d polygon, double height, bool skip_empty)
{
	std::vector<stocker::Contour3d> all_points;
	for (ccHObject* entity : entities) {
		stocker::Contour3d points = GetPointsFromCloudInsidePolygonXY(entity, polygon, height);
		if (!skip_empty || (skip_empty && points.size() > 0)) {
			all_points.push_back(points);
		}		
	}
	return all_points;
}
std::vector<stocker::Contour3d> GetPointsFromCloudInsidePolygonsXY(ccHObject* entity, stocker::Polyline3d polygon, double height, bool skip_empty)
{
	ccHObject::Container planes_container, plane_cloud_container;
	entity->filterChildren(planes_container, true, CC_TYPES::PLANE, true);
	for (ccHObject* pl_obj : planes_container) {
		if (pl_obj->isEnabled() && pl_obj->getParent() && pl_obj->getParent()->isEnabled()) {
			plane_cloud_container.push_back(pl_obj->getParent());
		}
	}
	return GetPointsFromCloudInsidePolygonsXY(plane_cloud_container, polygon, height, skip_empty);
}

stocker::Contour3d GetPointsFromCloudInsidePolygon3d(ccHObject* entity, stocker::Polyline3d polygon, stocker::Contour3d& remained, double distance_threshold)
{
	stocker::Contour3d points;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!cloud) return points;

	//! FIT A PLANE
	Contour3d polygon_points = ToContour(polygon, 0);
	PlaneUnit plane_unit = FormPlaneUnit(polygon_points);
	//! PROJECT TO THE PLANE
	Polyline2d polygon_2d = Line3dToPlline2d(plane_unit, polygon);
	std::vector<vcg::Segment2d> vcg_polygon_2d;
	for (auto & seg : polygon_2d) { vcg_polygon_2d.push_back(seg); }

	//! GET INSIDE 2d	
	Concurrency::concurrent_vector<Vec3d> points_parallel, points_remained;
	Concurrency::parallel_for((size_t)0, (size_t)cloud->size(), [&](size_t i) {
		CCVector3 pt = *cloud->getPoint(i);
		Vec3d pt_3d(pt.x, pt.y, pt.z);
		Vec2d pt_2d = plane_unit.ToPlpoint2d(pt_3d);
		double distance = vcg::SignedDistancePointPlane(pt_3d, plane_unit.plane);
		if (fabs(distance) < distance_threshold && vcg::PointInsidePolygon(pt_2d, polygon_2d)) {
			points_parallel.push_back({ pt.x, pt.y, pt.z });
		}
		else {
			points_remained.push_back({ pt.x, pt.y, pt.z });
		}
	});
	points.assign(points_parallel.begin(), points_parallel.end());
	remained.clear(); remained.assign(points_remained.begin(), points_remained.end());
	return points;
}

stocker::Polyline3d GetPolygonFromPolyline(ccHObject* entity)
{
	stocker::Polyline3d polyline;
	if (!entity) return polyline;
	ccPolyline* ccpolyline = nullptr;
	if (entity->isA(CC_TYPES::POLY_LINE)) {
		ccpolyline = ccHObjectCaster::ToPolyline(entity);
	}
	else if (entity->isA(CC_TYPES::ST_FOOTPRINT)) {
		ccpolyline = ccHObjectCaster::ToStFootPrint(entity);
	}
	if (!ccpolyline) {
		throw std::runtime_error("not a polyline, internal error");
		return polyline;
	}
	unsigned lastvert = ccpolyline->isClosed() ? ccpolyline->size() : ccpolyline->size() - 1;
	for (size_t i = 0; i < lastvert; i++) {
		stocker::Seg3d seg;
		CCVector3 P0 = *(ccpolyline->getPoint(i));
		CCVector3 P1 = *(ccpolyline->getPoint((i + 1) % ccpolyline->size()));
		seg.P0() = stocker::parse_xyz(P0);
		seg.P1() = stocker::parse_xyz(P1);
		polyline.push_back(seg);
	}
	return polyline;
}

stocker::Polyline3d GetPolylineFromEntities(ccHObject::Container entities)
{
	stocker::Polyline3d polyline;
	for (auto & polyline_entity : entities) {
		Polyline3d cur = GetPolygonFromPolyline(polyline_entity);
		polyline.insert(polyline.end(), cur.begin(), cur.end());
	}
	return polyline;
}

bool GetBoundaryPointsAndLinesFromCloud(ccHObject* cloud_entity, stocker::Polyline3d & boundary_lines, stocker::Contour3d & boundary_points)
{
	boundary_lines.clear();
	boundary_points.clear();
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(cloud_entity);
	if (!cloud) return false;	

	ccHObject::Container boundary_entities;
	cloud_entity->filterChildren(boundary_entities, false, CC_TYPES::POLY_LINE, cloud_entity->getDisplay());

	boundary_lines = GetPolylineFromEntities(boundary_entities);
	
	unsigned int pt_start = boundary_lines.size();
	for (unsigned int i = pt_start; i < cloud->size(); i++) {
		const CCVector3* P = cloud->getPoint(i);
		boundary_points.push_back(stocker::parse_xyz(*P));
	}
	return true;
}

vector<vector<stocker::Contour3d>> GetOutlinesFromOutlineParent(ccHObject* entity)
{
	ccHObject::Container container_find;
	entity->filterChildren(container_find, false, CC_TYPES::POLY_LINE, true);
	QString name = QString();
	vector<vector<stocker::Contour3d>> contours_points;
	for (auto & component : container_find) {
		if (component->getName() != name) {
			contours_points.push_back(vector<stocker::Contour3d>());
			name = component->getName();
		}
		ccPolyline* poly_line = ccHObjectCaster::ToPolyline(component);
		vector<CCVector3> outline_points = poly_line->getPoints(true);
		Contour3d outline_points_;
		for (auto & pt : outline_points) {
			outline_points_.push_back(parse_xyz(pt));
		}
		contours_points.back().push_back(outline_points_);
	}
	return contours_points;
}

ccHObject::Container GetPlaneEntitiesBySelected(ccHObject* select)
{	
	ccHObject::Container plane_container;
	if (!select) { return plane_container; }
	
	if (isBuildingProject(select)) {
		BDBaseHObject* baseObj = GetRootBDBase(select); assert(baseObj);
		ccHObject::Container buildings = GetEnabledObjFromGroup(baseObj, CC_TYPES::ST_BUILDING, true, false);
		for (ccHObject* bd : buildings) {
			StPrimGroup* primGroup = baseObj->GetPrimitiveGroup(bd->getName());
			if (!primGroup || !primGroup->isEnabled()) { continue; }
			ccHObject::Container cur_valid_planes = primGroup->getValidPlanes();
			if (!cur_valid_planes.empty()) {
				plane_container.insert(plane_container.end(), cur_valid_planes.begin(), cur_valid_planes.end());
			}
		}
	}
	else if (select->isA(CC_TYPES::ST_PRIMGROUP)) {
		StPrimGroup* primGroup = ccHObjectCaster::ToStPrimGroup(select); assert(primGroup);
		ccHObject::Container cur_valid_planes = primGroup->getValidPlanes();
		if (!cur_valid_planes.empty()) {
			plane_container.insert(plane_container.end(), cur_valid_planes.begin(), cur_valid_planes.end());
		}
	}
	else {
		ccHObject* plane = GetPlaneFromPlaneOrCloud(select);
		if (plane) {
			plane_container.push_back(plane);
		}
	}
	return plane_container;
}

ccHObject::Container GetBuildingEntitiesBySelected(ccHObject* select)
{
	ccHObject::Container building_container;
	if (!select) { return building_container; }

	if (isBuildingProject(select) || select->getClassID() == CC_TYPES::HIERARCHY_OBJECT) {
		//BDBaseHObject* baseObj = GetRootBDBase(select); assert(baseObj);
		building_container = GetEnabledObjFromGroup(select, CC_TYPES::ST_BUILDING, true, false);
	}
	else {
		ccHObject* bd = GetParentBuilding(select);
		if (bd) building_container.push_back(bd);
	}
	return building_container;
}

ccPlane* GetPlaneFromCloud(ccHObject * entity)
{
	if (entity->isA(CC_TYPES::POINT_CLOUD)) {
		for (size_t i = 0; i < entity->getChildrenNumber(); i++) {
			if (entity->getChild(i)->isA(CC_TYPES::PLANE)) {
				return ccHObjectCaster::ToPlane(entity->getChild(i));
			}
		}
	}
	return nullptr;
}
ccPlane* GetPlaneFromPlaneOrCloud(ccHObject * entity)
{
	if (entity->isA(CC_TYPES::PLANE)) {
		return ccHObjectCaster::ToPlane(entity);
	}
	else {
		ccPlane* plane = GetPlaneFromCloud(entity);
		if (plane) { return plane; }
	}
	return nullptr;
}

ccHObject* GetPlaneEntityFromPrimGroup(ccHObject* prim, QString name)
{
	ccHObject::Container pc_find, pl_find;
	prim->filterChildrenByName(pc_find, false, name, true);
	if (pc_find.empty()) return nullptr;

	pc_find.front()->filterChildren(pl_find, false, CC_TYPES::PLANE, true);
	if (pl_find.empty()) return nullptr;
	return pl_find.front();
}

vcg::Plane3d GetVcgPlane(ccHObject* planeObj)
{
	ccPlane* ccPlane = ccHObjectCaster::ToPlane(planeObj);
	CCVector3 N; float constVal;
	ccPlane->getEquation(N, constVal);
	vcg::Plane3d vcgPlane;
	vcgPlane.SetDirection({ N.x, N.y, N.z });
	vcgPlane.SetOffset(constVal);
	return vcgPlane;
}

StBuilding* GetParentBuilding(ccHObject* obj) {
	ccHObject* bd_obj_ = obj;
	do {
		if (bd_obj_->isA(CC_TYPES::ST_BUILDING)) {
			return static_cast<StBuilding*>(bd_obj_);
		}
		bd_obj_ = bd_obj_->getParent();
	} while (bd_obj_);

	return nullptr;
}

ccPointCloud* GetPlaneCloud(ccHObject* planeObj) {
	return ccHObjectCaster::ToPointCloud(planeObj->isA(CC_TYPES::PLANE) ? planeObj->getParent() : planeObj);
}

std::vector<stocker::Outline3d> GetPlanarOutlines(ccHObject* entity, QString prefix)
{
	std::vector<stocker::Outline3d> outlines;

	ccHObject* outlineEnt = nullptr;
	if (entity->isA(CC_TYPES::POINT_CLOUD) && entity->getName() == prefix) {
		outlineEnt = entity;
	}
	else {
		ccHObject* plane_cloud = GetPlaneCloud(entity);
		if (plane_cloud) {
			ccHObject::Container filtered_children;
			if (plane_cloud->filterChildrenByName(filtered_children, true, prefix, false, CC_TYPES::POINT_CLOUD) == 0)
				return outlines;
			for (int i = filtered_children.size() - 1; i >= 0; i--) {
				if (filtered_children[i]->isEnabled()) {
					outlineEnt = filtered_children[i];
				}
			}
		}
	}

	if (!outlineEnt) { return outlines; }

	std::map<int, ccHObject::Container> map_index_outlines;
	for (size_t i = 0; i < outlineEnt->getChildrenNumber(); i++) {
		ccHObject* child = outlineEnt->getChild(i);
		if (!child || !child->isEnabled()) continue;

		int number = GetNumberExcludePrefix(child, prefix);
		if (number < 0) { continue; }

		map_index_outlines[number].push_back(child);
	}

	for (auto indexed_outline : map_index_outlines)	{
		stocker::Outline3d outline;
		for (ccHObject* sub_outline : indexed_outline.second) {
			stocker::Polyline3d polyline = GetPolygonFromPolyline(sub_outline);
			if (polyline.size() < 3) continue;
			stocker::Polygon3d polygon(polyline);
			if (!outline.empty()) {
				polygon.isHole = true;
				// TODO: check whether the hole is inside the outer boundary
			}
			outline.push_back(polygon);
		}
		outlines.push_back(outline);
	}
	return outlines;
}

bool SetGlobalShiftAndScale(ccHObject* obj)
{
	BDBaseHObject* baseObj = GetRootBDBase(obj);
	if (!baseObj) {
		return false;
	}
	
	if (obj->isKindOf(CC_TYPES::POLY_LINE)) {
		ccShiftedObject* shift = ccHObjectCaster::ToShifted(obj);
		shift->setGlobalScale(baseObj->global_scale);
		shift->setGlobalShift(CCVector3d(vcgXYZ(baseObj->global_shift)));
	}
	else {
		ccHObject::Container cloud_container;
		if (obj->isA(CC_TYPES::POINT_CLOUD)) {
			cloud_container.push_back(obj);
		}
		else {
			obj->filterChildren(cloud_container, true, CC_TYPES::POINT_CLOUD, false);
		}
		for (auto & _cld : cloud_container) {
			ccPointCloud* cloud_entity = ccHObjectCaster::ToPointCloud(_cld);
			cloud_entity->setGlobalScale(baseObj->global_scale);
			cloud_entity->setGlobalShift(CCVector3d(vcgXYZ(baseObj->global_shift)));
		}
	}
	return true;
}

void filterCameraByName(ccHObject * camera_group, QStringList name_list)
{
	for (size_t i = 0; i < camera_group->getChildrenNumber(); i++) {
		ccHObject* camera = camera_group->getChild(i); 
		camera->setEnabled(false);
		for (QString name : name_list) {
			if (camera->getName() == name) {
				camera->setEnabled(true);				
				break;
			}
		}
		camera->redrawDisplay();
	}
}

ccPlane* FitPlaneAndAddChild(ccPointCloud* cloud, const vcg::Plane3d* plane_para /*= nullptr*/)
{
	ccPlane* pPlane = nullptr;
	if (plane_para) {		
		PointCoordinateType* planeEquation = new PointCoordinateType[4];
		planeEquation[0] = plane_para->Direction().X();
		planeEquation[1] = plane_para->Direction().Y();
		planeEquation[2] = plane_para->Direction().Z();
		planeEquation[3] = plane_para->Offset();
		Contour3d points = GetPointsFromCloud3d(cloud, true);
		PlaneUnit plane_unit = FormPlaneUnit("temp", *plane_para, points, true);
		if (plane_unit.convex_hull_prj.size() >= 3) {
			std::vector<CCVector3> cc_profile;
			for (auto & pt : plane_unit.convex_hull_prj) {
				cc_profile.push_back(CCVector3(vcgXYZ(pt)));
			}
			pPlane = ccPlane::Fit(cc_profile, planeEquation);
		}
	}
	if (!pPlane) {
		double rms = 0; std::vector<CCVector3> c_hull;
		pPlane = ccPlane::Fit(cloud, &rms, &c_hull);
	}
	
	if (pPlane) {
		if (cloud->hasColors()) {
			pPlane->setColor(cloud->getPointColor(0));
		}
		pPlane->enableStippling(true);
	
		pPlane->setName("Plane");
		pPlane->applyGLTransformation_recursive();
		pPlane->showColors(true);
		pPlane->setVisible(true);
		pPlane->showNormals(cloud->hasNormals());

		cloud->addChild(pPlane);
		pPlane->setDisplay(cloud->getDisplay());
		pPlane->prepareDisplayForRefresh_recursive();
	}
	return pPlane;
}

ccPointCloud* AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col)
{
	if (lines.empty()) {
		return nullptr;
	}
	ccPointCloud* line_vert = new ccPointCloud(name);
	if (entity) {
		line_vert->setDisplay(entity->getDisplay());
		ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
		if (cloud) {
			line_vert->setGlobalShift(cloud->getGlobalShift());
			line_vert->setGlobalScale(cloud->getGlobalScale());
		}
		entity->addChild(line_vert);
	}
	AddSegmentsToVertices(line_vert, lines, name, col);
	return line_vert;
}

void AddSegmentsToVertices(ccPointCloud* cloud, stocker::Polyline3d lines, QString Prefix, ccColor::Rgb col)
{
	if (lines.empty() || !cloud) {
		return;
	}
		
	int index = GetMaxNumberExcludeChildPrefix(cloud, Prefix) + 1;
	MainWindow* win = MainWindow::TheInstance();
	assert(win);
	for (auto & ln : lines) {
		ccPolyline* cc_polyline = new ccPolyline(cloud);
		cc_polyline->setDisplay(cloud->getDisplay());
		cc_polyline->setColor(col);
		cc_polyline->showColors(true);
		cc_polyline->setName(Prefix + QString::number(index++));
		cc_polyline->reserve(2);

		cloud->addPoint(CCVector3(vcgXYZ(ln.P0())));
		cc_polyline->addPointIndex(cloud->size() - 1);

		cloud->addPoint(CCVector3(vcgXYZ(ln.P1())));
		cc_polyline->addPointIndex(cloud->size() - 1);

		cc_polyline->setClosed(false);
		cloud->addChild(cc_polyline);
		win->addToDB(cc_polyline, cloud->getDBSourceType(), false, false);
	}
}

template <typename T = stocker::Vec3d>
ccPointCloud* AddPointsAsPointCloud(std::vector<T> points, QString name, ccColor::Rgb col= ccColor::white)
{
	if (points.empty()) { return nullptr; }
	ccPointCloud* cloud = new ccPointCloud(name);

	//! get plane points
	if (!cloud->reserveThePointsTable(points.size()))
		return nullptr;
	for (auto & pt : points) {
		cloud->addPoint(CCVector3(vcgXYZ(pt)));
	}
	cloud->setRGBColor(col);
	cloud->showColors(true);
	
	return cloud;
}

template <typename T = stocker::Vec3d>
ccPointCloud* AddPointsAsPlane(std::vector<T> points, QString name, ccColor::Rgb col, const vcg::Plane3d* plane_para /*= nullptr*/)
{	
	ccPointCloud* plane_cloud = AddPointsAsPointCloud(points, name, col);
	if (!plane_cloud)return nullptr;
		
	//! add plane
	ccPlane* plane = FitPlaneAndAddChild(plane_cloud, plane_para);
#ifdef DEBUG_TEST
	CCVector3 n; float o; plane->getEquation(n, o);
	std::cout << "cc plane: " << n.x << " " << n.y << " " << n.z << " " << o << std::endl;
#endif // DEBUG_TEST
	if (!plane && plane_cloud) {
		delete plane_cloud;
		plane_cloud = nullptr;
	}
	return plane_cloud;
}

ccPointCloud* AddSegmentsAsPlane(stocker::Polyline3d lines, QString lines_prefix, ccColor::Rgb col, ccHObject* _exist_cloud)
{
	ccPointCloud* plane_cloud = nullptr;
	if (_exist_cloud) {
		plane_cloud = ccHObjectCaster::ToPointCloud(_exist_cloud);
	}
	else {
		plane_cloud = AddPointsAsPlane(stocker::ToContour(lines, 3), "Plane", col);
	}
	ccPointCloud* line_vert = AddSegmentsAsChildVertices(plane_cloud, lines, lines_prefix, col);

	if (!line_vert && plane_cloud && !_exist_cloud) {
		delete plane_cloud;
		plane_cloud = nullptr;
	}

	return plane_cloud;
}

ccPolyline* AddPolygonAsPolyline(stocker::Contour3d points, QString name, ccColor::Rgb col, bool close)
{
	ccPointCloud* cloudObj = AddPointsAsPointCloud(points, "vertices", col);
	if (!cloudObj) { return nullptr; }
	ccPolyline* polylineObj = new ccPolyline(cloudObj);
	polylineObj->reserve(cloudObj->size());
	for (size_t i = 0; i < cloudObj->size(); i++) {
		polylineObj->addPointIndex(i);
	}
// 	if (close) {
// 		polylineObj->addPointIndex(0);
// 		polylineObj->setClosed(true);
// 	}
// 	else {
// 		polylineObj->setClosed(false);
// 	}
	polylineObj->setClosed(close);
	polylineObj->addChild(cloudObj);
	cloudObj->setEnabled(false);

	return polylineObj;
}

ccPolyline* AddPolygonAsPolyline(stocker::Polyline3d polygon, QString name, ccColor::Rgb col, bool close)
{
	if (polygon.empty()) { return nullptr; }
	Contour3d points = ToContour(polygon, 3);
	return AddPolygonAsPolyline(points, name, col, close);
}

StFootPrint* AddPolygonAsFootprint(stocker::Contour3d polygon, QString name, ccColor::Rgb col, bool close)
{
	ccPolyline* polyline = AddPolygonAsPolyline(polygon, name, col, close);
	StFootPrint* footptObj = new StFootPrint(0);
	ccPointCloud* vertices = 0;
	footptObj->initWith(vertices, *polyline);
	footptObj->setAssociatedCloud(vertices);
	footptObj->setColor(col);
	footptObj->showColors(true);
	footptObj->setName(name);

	delete polyline;
	polyline = nullptr;
	return footptObj;
}

template <typename T = stocker::Contour3d>
StPrimGroup* AddPlanesPointsAsNewGroup(QString name, std::vector<T> planes_points, stocker::vcgPlaneVec3d* planes = nullptr)
{
	StPrimGroup* group = new StPrimGroup(name);

	for (size_t i = 0; i < planes_points.size(); i++) {
		ccPointCloud* plane_cloud = AddPointsAsPlane(planes_points[i],
			BDDB_PLANESEG_PREFIX + QString::number(i),
			ccColor::Generator::Random(),
			planes ? &((*planes)[i]) : nullptr);
		if (plane_cloud) {
			group->addChild(plane_cloud);
		}
	}
	return group;
}

StPrimGroup* parsePlaneSegmentationResult(ccPointCloud* entity_cloud, std::vector<stocker::Contour3d> planes_points, stocker::vcgPlaneVec3d* planes = nullptr,
	ccPointCloud* todo_cloud = nullptr, std::vector<stocker::Point_Normal>* unassigned_points = nullptr, bool saveFile = true)
{
	if (!entity_cloud) {
		return nullptr;
	}
	StPrimGroup* group = AddPlanesPointsAsNewGroup<stocker::Contour3d>("unnamed", planes_points, planes);
	if (!group)	{
		return false;
	}
	
	if (entity_cloud->getDisplay())
		group->setDisplay_recursive(entity_cloud->getDisplay());
	
	ccHObject::Container group_clouds, group_planes;

	group->filterChildren(group_clouds, false, CC_TYPES::POINT_CLOUD, true);
	CCVector3d global_shift = entity_cloud->getGlobalShift();
	double global_scale = entity_cloud->getGlobalScale();
	Concurrency::parallel_for_each(group_clouds.begin(), group_clouds.end(), [&](auto & ent) {
		ccPointCloud* ent_cld = ccHObjectCaster::ToPointCloud(ent);
		ent_cld->setGlobalShift(global_shift);
		ent_cld->setGlobalScale(global_scale);
	});

	group->filterChildren(group_planes, true, CC_TYPES::PLANE, true);
	Concurrency::parallel_for_each(group_planes.begin(), group_planes.end(), [&](auto & ent) {
		ent->setVisible(false);
	});

	BDBaseHObject* baseObj = GetRootBDBase(entity_cloud); 
	bool project_loaded = false;
	if (baseObj) {
		StPrimGroup* primGroup = baseObj->GetPrimitiveGroup(GetBaseName(entity_cloud->getName()));
		if (primGroup) {
			primGroup->removeAllChildren();
			group->transferChildren(*primGroup);
			primGroup->setMetaData(group->metaData());
			delete group;
			group = primGroup;
			project_loaded = true;
		}
	}
	if (!project_loaded) {
		ccHObject* parent = entity_cloud->getParent();
		if (parent) {
			parent->addChild(group);
		}
		else {
			entity_cloud->addChild(group);
		}
		group->setName(getPrimGroupNameByCloudName(entity_cloud->getName()));
	}
	
	if (todo_cloud && unassigned_points) {
		std::vector<stocker::Point_Normal> uss = *unassigned_points;
		if (todo_cloud->reserveThePointsTable(uss.size())) {
			todo_cloud->reserveTheNormsTable();
			todo_cloud->reserveTheRGBTable();
			for (stocker::Point_Normal pt : *unassigned_points) {
				todo_cloud->addPoint(CCVector3(vcgXYZ(pt.first)));
				todo_cloud->addNorm(CCVector3(vcgXYZ(pt.second)));
			}
			todo_cloud->setRGBColor(ccColor::black);
			todo_cloud->showColors(true);
			todo_cloud->setEnabled(false);
		}
	}
	if (saveFile) {
		QFileInfo path_info = QFileInfo(entity_cloud->getPath());
		if (path_info.exists()) {
			QString path = path_info.absolutePath() + "/" + path_info.completeBaseName() + ".prim.ply";
			SavePlaneParaMesh(path.toStdString(), *planes, planes_points, *unassigned_points);
		}
	}
	if (group) entity_cloud->setEnabled(false);
	return group;
}

StPrimGroup* LoadPlaneParaAsPrimtiveGroup(ccPointCloud* entity_cloud, ccPointCloud* todo_cloud)
{
	QString cloud_path = entity_cloud->getPath();
	if (!QFileInfo(cloud_path).exists() && entity_cloud->getParent()) {
		cloud_path = entity_cloud->getParent()->getPath();
	}
	QString path = getPrimPathByCloudPath(cloud_path);
	if (!QFileInfo(path).exists()) return nullptr;
	stocker::vcgPlaneVec3d planes;
	std::vector<stocker::Contour3d> planes_points;
	std::vector<stocker::Point_Normal> unassigned_points;
	if (!stocker::LoadPlaneParaMesh(path.toStdString(), planes, planes_points, unassigned_points))
		return nullptr;

	if (planes.empty() || planes.size() != planes_points.size()) {
		return nullptr;
	}
	return parsePlaneSegmentationResult(entity_cloud, planes_points, &planes, todo_cloud, &unassigned_points, false);
}

ccHObject* PlaneSegmentationRgGrow(ccHObject* entity, bool overwrite,
	int min_pts, double distance_epsilon, double seed_raius,
	double growing_radius,
	double merge_threshold, double split_threshold)
{
	ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!entity_cloud || !entity_cloud->hasNormals() || entity_cloud->size() == 0) {
		return nullptr;
	}

	if (!overwrite) {
		StPrimGroup* group_load = LoadPlaneParaAsPrimtiveGroup(entity_cloud, nullptr);
		if (group_load) return group_load;
	}

	std::vector<vcg::Plane3d> planes;
	std::vector<stocker::Contour3d> planes_points;

	stocker::BuilderLOD2 builder_3d4em(true);
	stocker::Contour3d point_cloud;
	for (unsigned i = 0; i < entity_cloud->size(); i++) {
		CCVector3 pt = *entity_cloud->getPoint(i);
		point_cloud.push_back(Vec3d(pt.x, pt.y, pt.z));
	}
	builder_3d4em.SetBuildingPoints(point_cloud);
	builder_3d4em.SetPlaneSegOption(min_pts, distance_epsilon, seed_raius, growing_radius);
	builder_3d4em.PlaneSegmentation();
	std::vector<Contour3d> pp_3d4em = builder_3d4em.GetSegmentedPoints();

	for (auto & pl_pts : pp_3d4em) {		
		vcg::Plane3d plane;
		stocker::Vec3d cen;
		stocker::FitPlane(pl_pts, plane, cen);
		planes.push_back(plane);
		planes_points.push_back(pl_pts);
	}

	if (merge_threshold > 0 || split_threshold > 0) {
		stocker::Option_PlaneRefinement option_refine;
		option_refine.merge_threshold = merge_threshold;
		option_refine.split_threshold = split_threshold;
		if (!stocker::PlaneRefinement(planes, planes_points, option_refine)) {
			return nullptr;
		}
	}
	return parsePlaneSegmentationResult(entity_cloud, planes_points, (planes.size() == planes_points.size()) ? &planes : nullptr, nullptr, nullptr);
}

ccHObject* PlaneSegmentationRansac(ccHObject* entity, bool overwrite, ccPointCloud* todo_cloud,
	int min_pts, double distance_epsilon, double seed_raius,
	double normal_threshold, double ransac_probability,
	double merge_threshold, double split_threshold)
{
	ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!entity_cloud || !entity_cloud->hasNormals() || entity_cloud->size() == 0) {
		return nullptr;
	}

	if (!overwrite) {
		StPrimGroup* group_load = LoadPlaneParaAsPrimtiveGroup(entity_cloud, todo_cloud);
		if (group_load) return group_load;
	}

	stocker::GLMesh mesh;
	for (unsigned i = 0; i < entity_cloud->size(); i++) {
		CCVector3 pt = *entity_cloud->getPoint(i);
		CCVector3 normal = entity_cloud->getPointNormal(i);
		stocker::StMeshAL::AddVertex(mesh, parse_xyz(pt), parse_xyz(normal));
	}

	std::vector<vcg::Plane3d> planes;
	std::vector<stocker::Contour3d> planes_points;
	std::vector<Point_Normal> unassigned_points;
	
	stocker::Option_PlaneSegmentation option;
	option.min_points = min_pts;
	option.distance_epsilon = distance_epsilon;
	option.cluster_epsilon = seed_raius;
	option.normal_threshold = normal_threshold;
	option.ransac_probability = ransac_probability;
	if (!stocker::PlaneSegmentation(mesh, planes, planes_points, unassigned_points, option)) {
		return nullptr;
	}
	
	if (merge_threshold > 0 || split_threshold > 0) {
		stocker::Option_PlaneRefinement option_refine;
		option_refine.merge_threshold = merge_threshold;
		option_refine.split_threshold = split_threshold;
		if (!stocker::PlaneRefinement(planes, planes_points, option_refine)) {
			return nullptr;
		}
	}
	return parsePlaneSegmentationResult(entity_cloud, planes_points, (planes.size() == planes_points.size()) ? &planes : nullptr, todo_cloud, &unassigned_points);
}

/*
	kappa_t: the minimum point number for a valid plane. (20)
	delta_t: the threshold of curvature for multi-scale supervoxel segmentation. (0.05)
	tau_t: the threshold of distance tolerance value for point-to-plane and plane-to-plane. (0.1)
	gamma_t: the threshold of neighborhood for point-to-plane and plane-to-plane. (0.2)
	epsilon_t: the threshold of NFA tolerance value for a-contrario rigorous planar supervoxel generation. (-3.0)
	theta_t: the threshold of normal vector angle for hybrid region growing. (0.2618)
	iter_times: 1-for converge, 0-for only 1
*/
ccHObject* PlaneSegmentationATPS(ccHObject* entity, bool overwrite,	ccPointCloud* todo_cloud, 
	bool* iter_times,
	int* kappa_t, double* delta_t, double* tau_t, 
	double* gamma_t, double* epsilon_t, double* theta_t)
{
	ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!entity_cloud || entity_cloud->size() == 0) {
		return nullptr;
	}

	if (!overwrite) {
		StPrimGroup* group_load = LoadPlaneParaAsPrimtiveGroup(entity_cloud, todo_cloud);
		if (group_load) return group_load;
	}

	ATPS::ATPS_Plane atps_plane;
	if (kappa_t && delta_t && tau_t && gamma_t && epsilon_t && theta_t) {
		atps_plane.set_parameters(*kappa_t, *delta_t, *tau_t, *gamma_t, *epsilon_t, *theta_t, 0);
	}
	if (iter_times) {
		atps_plane.set_iter(static_cast<int>(*iter_times));
	}

	typedef std::vector<ATPS::SVPoint3d> ATPSpVec;
	ATPSpVec points = GetPointsFromCloud3d<ATPS::SVPoint3d>(entity_cloud);
	double res = atps_plane.get_res(points);
	atps_plane.set_parameters(res);
	
	ATPSpVec unassigned_points;
	std::vector<ATPSpVec> planes_points;
	std::vector<std::vector<double>> params;
	if (!atps_plane.ATPS_PlaneSegmentation(points, planes_points, unassigned_points, params) || planes_points.size() != params.size()) {
		return nullptr;
	}
	std::cout << entity_cloud->getName().toStdString() << ": plane segmentation done" << std::endl;

	std::vector<vcg::Plane3d> planes;
	for (auto & pl : params) {
		if (pl.size() != 4) {
			std::cout << "plane param size not equals to 4" << std::endl;
			planes.clear();
			break;
		}
		planes.push_back(vcg::Plane3d(-pl[3], { pl[0],pl[1],pl[2] }));
	}

	std::vector<Contour3d> planes_points_stocker;
	std::vector<stocker::Point_Normal> unassigned_stocker;
	for (auto pl_pts : planes_points) {
		stocker::Contour3d pl_pts_st;
		for (auto pt : pl_pts) {
			pl_pts_st.emplace_back(vcgXYZ(pt));
		}
		planes_points_stocker.push_back(pl_pts_st);
	}
	for (auto pt : unassigned_points) {
		unassigned_stocker.push_back(std::make_pair(Vec3d(vcgXYZ(pt)), Vec3d(0, 0, 0)));
	}

	return parsePlaneSegmentationResult(entity_cloud, planes_points_stocker, (planes.size() == planes_points_stocker.size()) ? &planes : nullptr, todo_cloud, &unassigned_stocker);
}

void CalculatePlaneQuality(ccHObject::Container primObjs, int mode)
{
	QString rms_meta = QString("RMS");
	for (ccHObject* ent : primObjs) {
		if (ent->hasMetaData(QString("Flatness"))) { ent->removeMetaData(QString("Flatness")); }

		ccPlane* planeObj = ccHObjectCaster::ToPlane(ent); if (!planeObj) continue;
		ccPointCloud* plane_cloud = GetPlaneCloud(planeObj); if (!plane_cloud) continue;
		
		double rms(0); bool rms_ok = false;
		
		if (planeObj->hasMetaData(rms_meta)) {
			rms = planeObj->getMetaData(rms_meta).toDouble(&rms_ok);
		}
		if (!rms_ok) {
			//! distance from point to plane
			vcg::Plane3d vcg_plane = GetVcgPlane(planeObj);
			double dSumSq = 0.0;
			unsigned count = plane_cloud->size();
			for (unsigned i = 0; i < count; ++i)
			{
				CCVector3 P = *plane_cloud->getPoint(i);
				dSumSq += vcg::PlanePointSquaredDistance(vcg_plane, vcg::Point3d(P.x, P.y, P.z));
			}
			rms = sqrt(dSumSq / count);
			planeObj->setMetaData(QString("RMS"), QVariant(rms));
			rms_ok = true;
		}
		if (!rms_ok) continue;

		double flatness = rms;
		if (mode == 0) {
			flatness /= (planeObj->getXWidth()*planeObj->getYWidth());
		}
		else {
			stocker::Contour3d points = ccToPoints3<CCVector3, stocker::Vec3d>(planeObj->getProfile());
			double length(0);
			for (auto seg : MakeLoopPolylinefromContour(points))
				length += seg.Length();
			if (fabs(length) > FLT_EPSILON)	{
				flatness /= length;
			}
		}
		planeObj->setMetaData(QString("Flatness"), QVariant(flatness));
	}
	return;
}

void RetrieveUnassignedPoints(ccHObject* original_cloud, ccHObject* prim_group, ccPointCloud* todo_point)
{
	Contour3d all_points = GetPointsFromCloud3d(original_cloud);
	Contour3d used_points = GetPointsFromCloud3d(prim_group);
	Contour3d unassigned_points = stocker::GetUnassignedPoints(used_points, all_points);
	std::cout << "found " << unassigned_points.size() << std::endl;
	if (!todo_point) {
		throw std::runtime_error("nullptr todo point");
		return;
	}
	for (auto & pt : unassigned_points) {
		todo_point->addPoint(CCVector3(vcgXYZ(pt)));
	}
	todo_point->setRGBColor(ccColor::black);
	todo_point->showColors(true);
}

void RetrieveAssignedPoints(ccPointCloud* todo_cloud, ccPointCloud* plane_cloud, double distance_threshold)
{
	// todo_cloud
	// convex hull
	ccPlane* plane = nullptr;
	for (size_t i = 0; i < plane_cloud->getChildrenNumber(); i++) {
		if (plane_cloud->getChild(i)->isA(CC_TYPES::PLANE))	{
			plane = ccHObjectCaster::ToPlane(plane_cloud->getChild(i));
			break;
		}
	}
	if (!plane) { return; }
	std::vector<CCVector3> profile = plane->getProfile(); Contour3d st_profile;
	for (auto pt : profile) { st_profile.push_back(parse_xyz(pt)); }
	Polyline3d convex_hull = MakeLoopPolylinefromContour(st_profile);
	Contour3d remained;
	Contour3d points_in_plane = GetPointsFromCloudInsidePolygon3d(todo_cloud, convex_hull, remained, distance_threshold);
	if (points_in_plane.empty()) {
		return;
	}
	plane_cloud->reserveThePointsTable(plane_cloud->size() + points_in_plane.size());
	for (auto & pt : points_in_plane) {
		plane_cloud->addPoint(CCVector3(vcgXYZ(pt)));
	}
	ccColor::Rgb col = plane_cloud->hasColors() ? plane_cloud->getPointColor(0) : ccColor::Generator::Random();
	if (!plane_cloud->resizeTheRGBTable(true)) {
		throw runtime_error("not enough memory");
		return;
	}
	plane_cloud->setRGBColor(col);
	plane_cloud->showColors(true);

	todo_cloud->clear();
	todo_cloud->reserveThePointsTable(remained.size());
	for (auto & pt : remained) {
		todo_cloud->addPoint(CCVector3(vcgXYZ(pt)));
	}	
	if (!todo_cloud->resizeTheRGBTable(true)) {
		throw runtime_error("not enough memory");
		return;
	}
	todo_cloud->setRGBColor(ccColor::black);
	todo_cloud->showColors(true);
}

ccHObject::Container CalcPlaneIntersections(ccHObject::Container entity_planes, double distance)
{
#ifdef USE_STOCKER
	stocker::PlaneData plane_units;
	for (size_t i = 0; i < entity_planes.size(); i++) {
		if (!entity_planes[i]->isEnabled()) continue;

		stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(entity_planes[i]->getParent());
		if (cur_plane_points.size() < 3) continue;

		char name[32]; sprintf(name, "%d", i);
		plane_units.push_back(FormPlaneUnit(cur_plane_points, name, true));
	}
	//////////////////////////////////////////////////////////////////////////
	stocker::Polyline3d ints_all; vector<stocker::Polyline3d> ints_per_plane;
	ints_per_plane.resize(plane_units.size());
	for (size_t i = 0; i < plane_units.size() - 1; i++) {
		for (size_t j = i + 1; j < plane_units.size(); j++) {
			stocker::Seg3d cur_ints;
			if (!stocker::IntersectionPlanePlaneStrict(plane_units[i], plane_units[j], cur_ints, distance))
				continue;

			ints_per_plane[i].push_back(cur_ints);
			ints_per_plane[j].push_back(cur_ints);
			ints_all.push_back(cur_ints);
		}
	}
	//////////////////////////////////////////////////////////////////////////
	ccHObject::Container segs_add;
	for (size_t i = 0; i < plane_units.size(); i++) {
		int num;
		sscanf(plane_units[i].GetName().Str().c_str(), "%d", &num);
		ccHObject* add = AddSegmentsAsChildVertices(entity_planes[num]->getParent(), ints_per_plane[i], "Intersection", ccColor::cyan);
		if (add) segs_add.push_back(add);
	}
	return segs_add;
#endif // USE_STOCKER
	return ccHObject::Container();
}

ccHObject* CalcPlaneBoundary(ccHObject* planeObj, double distance, double minpts, double radius)
{
#ifdef USE_STOCKER
	ccPointCloud* point_cloud_obj = GetPlaneCloud(planeObj);
	if (!point_cloud_obj) return nullptr;
	/// get boundary points
	Contour2d boundary_points_2d;
	Contour3d cur_plane_points = GetPointsFromCloud3d(point_cloud_obj);
	PlaneUnit plane_unit = FormPlaneUnit(cur_plane_points, "temp", true);
	Contour2d points_2d = Point3dToPlpoint2d(plane_unit, cur_plane_points);
	vector<bool>bd_check;
	stocker::ComputeBoundaryPts2d(points_2d, bd_check, 32, true);
	assert(points_2d.size() == bd_check.size());
	for (size_t i = 0; i < bd_check.size(); i++) {
		if (bd_check[i]) {
			boundary_points_2d.push_back(points_2d[i]);
		}
	}

 	Contour3d boundary_points_3d = Plpoint2dToPoint3d(plane_unit, boundary_points_2d);
	//! add boundary points
	ccPointCloud* boundary_points = new ccPointCloud(BDDB_BOUNDARY_PREFIX);
	for (auto & pt : boundary_points_3d) {
		boundary_points->addPoint(CCVector3(vcgXYZ(pt)));
	}
	boundary_points->setRGBColor(ccColor::yellow);
	boundary_points->showColors(true);
	point_cloud_obj->addChild(boundary_points);
	boundary_points->setGlobalScale(point_cloud_obj->getGlobalScale());
	boundary_points->setGlobalShift(point_cloud_obj->getGlobalShift());

	/// get ransac based lines
// 	Polyline3d bdry_lines_2d; IndexGroup indices;
// 	LineRansacfromPoints(boundary_points_3d, bdry_lines_2d, indices, distance, minpts, radius);
// 
// 	ccHObject* line_vert_ransac = AddSegmentsAsChildVertices(boundary_points, bdry_lines_2d, "RansacLine", ccColor::red);

	/// get image based boundary lines
	Polyline3d detected_lines;
	stocker::LineFromPlanePoints(cur_plane_points, detected_lines);

	ccHObject* line_vert_image = AddSegmentsAsChildVertices(boundary_points, detected_lines, "ImageBased", ccColor::yellow);
	
	return boundary_points;
#endif // USE_STOCKER
	return nullptr;
}

ccHObject* DetectLineRansac(ccHObject* entity, double distance, double minpts, double radius)
{
#ifdef USE_STOCKER
	ccHObject* point_cloud_obj = GetPlaneCloud(entity);
	if (!point_cloud_obj) return nullptr;
	Contour3d cur_plane_points;
	
	cur_plane_points = GetPointsFromCloud3d(point_cloud_obj);
	
	Polyline3d bdry_lines_2d; IndexGroup indices;
	LineRansacfromPoints(cur_plane_points, bdry_lines_2d, indices, distance, minpts, radius);

	ccHObject* line_vert = AddSegmentsAsChildVertices(point_cloud_obj, bdry_lines_2d, "RansacLine", ccColor::red);
	return line_vert;
#endif // USE_STOCKER
	return nullptr;	
}

ccHObject* AddOutlinesAsChild(std::vector<std::vector<stocker::Contour3d>> contours_points, QString name, ccHObject* parent = nullptr)
{
	if (contours_points.empty()) return nullptr;
	ccPointCloud* line_vert = new ccPointCloud(name);
	int component_number = 0;
	for (vector<stocker::Contour3d> & component : contours_points) {
		for (stocker::Contour3d & st_contours : component) {
			ccPolyline* cc_polyline = new ccPolyline(line_vert);
 			cc_polyline->setDisplay(parent->getDisplay());
			cc_polyline->setColor(ccColor::green);
			cc_polyline->showColors(true);
			line_vert->addChild(cc_polyline);
			cc_polyline->setName(name + QString::number(component_number));
			cc_polyline->reserve(static_cast<unsigned>(st_contours.size() + 1));
			for (auto & pt : st_contours) {
				line_vert->addPoint(CCVector3(pt.X(), pt.Y(), pt.Z()));
				cc_polyline->addPointIndex(line_vert->size() - 1);
			}
			cc_polyline->setClosed(true);
		}
		component_number++;
	}
	if (parent)
		parent->addChild(line_vert);
	return line_vert;
}

ccHObject* CalcPlaneOutlines(ccHObject* planeObj, double alpha)
{
#ifdef USE_STOCKER
	ccPointCloud* point_cloud_obj = GetPlaneCloud(planeObj);
	if (!point_cloud_obj) return nullptr;
	stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(point_cloud_obj);
	if (cur_plane_points.size() < 3) {
		return nullptr;
	}

	//! get boundary
	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, alpha, false, 2);
	return AddOutlinesAsChild(contours_points, BDDB_OUTLINE_PREFIX, point_cloud_obj);
#endif // USE_STOCKER
}

#include "vcg/space/intersection2.h"
void ShrinkPlaneToOutline(ccHObject * planeObj, double alpha, double distance_epsilon)
{
#ifdef USE_STOCKER
	ccHObject* parent_cloud = planeObj->getParent();
	if (!parent_cloud) {
		std::cout << "failed to shrink plane" << planeObj->getName().toStdString() << std::endl;
		return;
	}
	stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(parent_cloud);
	if (cur_plane_points.size() < 3) {
		parent_cloud->setEnabled(false);
		return;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(parent_cloud);

	vcg::Plane3d vcgPlane = GetVcgPlane(planeObj);
	PlaneUnit plane_unit = FormPlaneUnit("temp", vcgPlane, cur_plane_points, true);
 	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, alpha, false, 2);
 	Contour3d concave_contour = contours_points.front().front();
	Contour2d concave_2d = Point3dToPlpoint2d(plane_unit, concave_contour);
	Polyline2d concave_polygon = MakeLoopPolylinefromContour(concave_2d);
		
	vector<size_t> inside_index;
	stocker::Contour3d inside_points;
	for (unsigned int i = 0; i < cloud->size(); i++) {
		CCVector3 point = *cloud->getPoint(i);
		vcg::Point2d pt_2d = plane_unit.Point3dPrjtoPlpoint2d({ parse_xyz(point) });
		if (vcg::PointInsidePolygon(pt_2d, concave_polygon)) {
			inside_index.push_back(i);
			inside_points.push_back(parse_xyz(point));					
		}
	}
	PlaneUnit plane_unit_inside = FormPlaneUnit(inside_points, "temp", true);
	CCLib::ReferenceCloud remained(cloud);
	Contour3d cur_plane_points_remained;
	for (size_t i = 0; i < inside_index.size(); i++) {
		Vec3d st_pt = parse_xyz(*cloud->getPoint(i));
		if (plane_unit_inside.IsInPlane(st_pt, distance_epsilon)) {
			remained.addPointIndex(i);
			cur_plane_points_remained.push_back(st_pt);
		}
	}
	ccPointCloud* newCloud = cloud->partialClone(&remained);
	newCloud->setName(cloud->getName());
	cloud->setName(cloud->getName() + "-del");
	parent_cloud->setEnabled(false);
	ccHObject* parent = parent_cloud->getParent();
	parent->addChild(newCloud);
	
	FitPlaneAndAddChild(newCloud);	

	int index_old = parent->getChildIndex(cloud);
	int index_new = parent->getChildIndex(newCloud);
	parent->swapChildren(index_old, index_new);
	MainWindow* win = MainWindow::TheInstance();
	win->addToDB(newCloud, planeObj->getDBSourceType());

	win->removeFromDB(cloud);

	vector<vector<stocker::Contour3d>> contours_points_remained = stocker::GetPlanePointsOutline(cur_plane_points_remained, alpha, false, 2);
	do {
		contours_points_remained.pop_back();
	} while (contours_points_remained.size() > 1);
	ccHObject* outlines_add = AddOutlinesAsChild(contours_points_remained, BDDB_OUTLINE_PREFIX, newCloud);
	win->addToDB(outlines_add, planeObj->getDBSourceType());
	
#endif // USE_STOCKER
}

void CreateIntersectionPoint(ccHObject* p1, ccHObject* p2)
{
	ccPolyline* line1 = ccHObjectCaster::ToPolyline(p1); if (!line1) return;
	ccPolyline* line2 = ccHObjectCaster::ToPolyline(p2); if (!line2) return;

	StBuilding* buildingObj = GetParentBuilding(line1);
	if (!buildingObj || buildingObj != GetParentBuilding(line2)) { return; }

	BDBaseHObject* baseObj = GetRootBDBase(buildingObj); if (!baseObj) return;
	ccPointCloud* todoCloud = baseObj->GetTodoPoint(buildingObj->getName()); if (!todoCloud) return;

	Polyline3d seg1_temp = GetPolygonFromPolyline(line1); if (seg1_temp.empty()) return;
	Polyline3d seg2_temp = GetPolygonFromPolyline(line2); if (seg2_temp.empty()) return;

	Seg3d seg1 = seg1_temp.front();
	Seg3d seg2 = seg2_temp.front();

	Vec3d ints_pt;
	if (!IntersectionLineLine(seg1, seg2, ints_pt))return;

	todoCloud->addPoint(CCVector3(vcgXYZ(ints_pt)));
	todoCloud->setPointColor(todoCloud->size() - 1, ccColor::red);
	todoCloud->prepareDisplayForRefresh();
}

ccHObject* PlaneFrameOptimization(ccHObject* planeObj, stocker::FrameOption option)
{
#ifdef USE_STOCKER
	ccHObject* point_cloud_obj = GetPlaneCloud(planeObj);
	if (!point_cloud_obj) return nullptr;
	
	std::string base_name = GetParentBuilding(point_cloud_obj)->getName().toStdString();

	BDBaseHObject* baseObj = GetRootBDBase(planeObj);
	std::string output_prefix;
	if (baseObj) {	
		stocker::BuildUnit bd_unit = baseObj->GetBuildingUnit(base_name);
		output_prefix = bd_unit.file_path.root_dir + "\\primitives\\frame_opt\\";
		CreateDir(output_prefix.c_str());
		output_prefix = output_prefix + base_name;
	}
	

	//////////////////////////////////////////////////////////////////////////
	// frame optimization

	std::string plane_unit_name = base_name + "-" + point_cloud_obj->getName().toStdString();
	stocker::FrameOptmzt frame_opt(plane_unit_name);
	
	frame_opt.SetOption(option);

	vcg::Plane3d vcgPlane = GetVcgPlane(planeObj);

	// prepare plane points
	Contour3d plane_points = GetPointsFromCloud3d(point_cloud_obj);

	// prepare boundary lines
	Polyline3d boundary_lines; Contour3d boundary_points; {		
		ccHObject::Container container_find, container_objs;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_BOUNDARY_PREFIX, true);
		if (!container_find.empty()) {
			GetBoundaryPointsAndLinesFromCloud(container_find.back(), boundary_lines, boundary_points);
		}		
	}

	// prepare outline
	Contour3d outline_points; {
		ccHObject::Container container_find;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_OUTLINE_PREFIX, true);
		if (!container_find.empty()) {
			auto outlines_points_all = GetOutlinesFromOutlineParent(container_find.back());
			if (!outlines_points_all.empty()) {
				outline_points = outlines_points_all.front().front();
			}
		}		
	}

	// prepare intersection
	Polyline3d intersections; {
		ccHObject::Container container_find, container_objs;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_INTERSECT_PREFIX, true);
		if (!container_find.empty()) {
			container_find.back()->filterChildren(container_objs, false, CC_TYPES::POLY_LINE, true);
			intersections = GetPolylineFromEntities(container_objs);
		}		
	}

	// prepare image lines
	Polyline3d image_lines;	{
		ccHObject::Container container_find, container_objs;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_IMAGELINE_PREFIX, true);
		if (!container_find.empty()) {
			container_find.back()->filterChildren(container_objs, false, CC_TYPES::POLY_LINE, true);
			image_lines = GetPolylineFromEntities(container_objs);
		}		
	}

	//! add data to frame
	frame_opt.PreparePlanePoints(vcgPlane, plane_points);
	if (baseObj) {
		frame_opt.PrepareImageList(baseObj->GetImageData());
	}	
	frame_opt.PrepareBoundaryPoints(boundary_points);
	frame_opt.PrepareImageLines(image_lines);
	frame_opt.PrepareIntersection(intersections);
//	frame_opt.PrepareBoundaryLines(boundary_lines, option.snap_epsilon);

	Polyline3d boundary_to_loop = frame_opt.PrepareBoundaryLines(outline_points, option.snap_epsilon);

	//! pre-process
//	Polyline3d boundary_to_loop;
//	boundary_to_loop = frame_opt.CloseBoundaryByConcaveHull(outline_points, option.snap_epsilon);

	//! candidate selection	
// 	frame_opt.GenerateCandidate(option.candidate_buffer_h, option.candidate_buffer_v, output_prefix + "-candi.ply");
// 	frame_opt.ComputeConfidence(option.lamda_coverage, option.lamda_sharpness);
// 	frame_opt.CandidateSelection(option.lamda_smooth_term);

	//! post-process
	Polyline3d frame_loop;
	if (!frame_opt.GenerateFrame(boundary_to_loop, frame_loop)) {
		std::cout << "cannot derive enclosed polygon" << std::endl;
	}
	frame_opt.ShrinkSharpVertex(CC_DEG_TO_RAD*25);

	//! get result
	Contour3d frame_points;
	frame_opt.OutputFrame(frame_points);
	vector<vector<Contour3d>> frames_to_add(1);
	frames_to_add.back().push_back(frame_points);

	ccHObject* plane_frame = AddOutlinesAsChild(frames_to_add, BDDB_PLANEFRAME_PREFIX, point_cloud_obj);
	return plane_frame;
#endif
}

ccHObject * PlaneFrameLineGrow(ccHObject * planeObj, double alpha, double intersection, double minpts)
{
	ccHObject* point_cloud_obj = GetPlaneCloud(planeObj);
	if (!point_cloud_obj) return nullptr;

	std::string base_name = GetParentBuilding(point_cloud_obj)->getName().toStdString();

	BDBaseHObject* baseObj = GetRootBDBase(planeObj);
	std::string output_prefix;
	if (baseObj) {
		stocker::BuildUnit bd_unit = baseObj->GetBuildingUnit(base_name);
		output_prefix = bd_unit.file_path.root_dir + "\\primitives\\frame_opt\\";
		CreateDir(output_prefix.c_str());
		output_prefix = output_prefix + base_name;
	}
	vector<vector<Contour3d>> frames_to_add;
	
	std::vector<stocker::Outline3d> outlines = GetPlanarOutlines(planeObj, BDDB_OUTLINE_PREFIX);

	if (outlines.empty()) {
		Contour3d plane_points = GetPointsFromCloud3d(point_cloud_obj);
		vector<Contour3d> frame;
		PolygonGeneralizationLineGrow_Plane(plane_points, frame, alpha, true, intersection);
		frames_to_add.push_back(frame);
	}
	else {
		for (stocker::Outline3d outline : outlines) {
			if (outline.empty() || outline[0].isHole) {
				continue;
			}

			std::vector<Contour3d> frame;
			for (size_t i = 0; i < outline.size(); i++) {
				Contour3d sub_frame;
				vector<Contour3d> sub_frames_temp;
				stocker::Polygon3d contour = outline[i];

				Contour3d contour_points = ToContour(contour, 0);
				if (!PolygonGeneralizationLineGrow_Contour(contour_points, sub_frames_temp, alpha, false, intersection))
					continue;
				
				sub_frame = sub_frames_temp.front();
				Vec3d normal = stocker::GetPolygonNormal(sub_frame);
				if (normal * Vec3d(0, 0, 1) < 0) {
					std::reverse(sub_frame.begin(), sub_frame.end());
				}
				frame.push_back(sub_frame);
			}

			frames_to_add.push_back(frame);
		}
	}
	
	ccHObject* plane_frame = AddOutlinesAsChild(frames_to_add, BDDB_PLANEFRAME_PREFIX, point_cloud_obj);
	return plane_frame;
}

bool FastPlanarTextureMapping(ccHObject* planeObj)
{
	ccHObject* baseObj = GetRootBDBase(planeObj);
	if (!baseObj) {
		throw std::runtime_error("cannot get base project");
		return false;
	}
	ccPlane* cc_plane = ccHObjectCaster::ToPlane(planeObj);
	if (!cc_plane) {
		return false;
	}
	QString imageFilename =
		QFileDialog::getOpenFileName(NULL,
			"Open image",
			"",
			"All (*.*);;png (*.png);;jpg (*.jpg)");
	QImageReader reader(imageFilename);
	//m_image = QImage(filename);
	QImage image = reader.read();
	if (image.isNull())
	{
		throw std::runtime_error(reader.errorString().toStdString());
		return false;
	}
	cc_plane->setAsTexture(image, imageFilename);

	planeObj->prepareDisplayForRefresh_recursive();
	return true;
}

bool TextureMappingBuildings(ccHObject::Container buildings, stocker::IndexVector* task_indices, double refine_length, double sampling_grid, int max_view)
{
	if (buildings.empty()) return false;
	BDBaseHObject* baseObj = GetRootBDBase(buildings.front());
	if (!baseObj) return false;

	stocker::TextureMapping texture_mapping;
	texture_mapping.setOutputDir(baseObj->getPath().toStdString() + "models");
	
	std::vector<stocker::ImageUnit> image_units = baseObj->GetImageData();
	
	for (stocker::ImageUnit & image_unit : image_units) {
		Vec3d view_pos = image_unit.GetViewPos();
		image_unit.m_camera.SetViewPoint(view_pos + baseObj->global_shift);
	}
	texture_mapping.setImages(image_units);
	//! collect polygons, roof and facade
	for (ccHObject* entity : buildings) {
		StBuilding* bdObj = ccHObjectCaster::ToStBuilding(entity);
		if (!bdObj) continue;

		BuildUnit build_unit = baseObj->GetBuildingUnit(bdObj->getName().toStdString());
		StBlockGroup* bdGroup = baseObj->GetBlockGroup(bdObj->getName());
		if (!bdGroup) continue;
		ccHObject::Container blocks = GetEnabledObjFromGroup(bdGroup, CC_TYPES::ST_BLOCK, true, true);
		if (blocks.empty()) { continue; }

		/// walls
		std::vector<stocker::Polygon3d> wall_polygons;
		for (ccHObject* blockEnt : blocks) {
			StBlock* blockObj = ccHObjectCaster::ToStBlock(blockEnt); assert(blockObj);
			std::vector<std::vector<CCVector3>> cur_wall_polys;
			if (!blockObj->getWallPolygons(cur_wall_polys)) continue;
			assert(cur_wall_polys.size() >= 5);
						
			for (auto poly : cur_wall_polys) {
				stocker::Contour3d poly_points = ccToPoints3<CCVector3, stocker::Vec3d>(poly);
				stocker::Polygon3d polygon = MakeLoopPolylinefromContour(poly_points);
				wall_polygons.push_back(polygon);
			}
		}

		/// vis images
		IndexVector visImageIndice;
		for (auto img_name : build_unit.image_list) {
			auto img_iter = std::find_if(image_units.begin(), image_units.end(), [=](ImageUnit img) {
				return img_name == img.GetName().Str();
			});
			if (img_iter != image_units.end()) {
				visImageIndice.push_back(std::distance(image_units.begin(), img_iter));
			}
		}
		texture_mapping.addMesh(wall_polygons, visImageIndice, build_unit.file_path.model_dir);
	}

	if (task_indices) {
		texture_mapping.setTaskIndice(*task_indices);
	}

	if (refine_length > 0) {
		texture_mapping.m_refineMesh = true;
		texture_mapping.m_refineLength = refine_length;
	}
	else {
		texture_mapping.m_refineMesh = false;
	}
	texture_mapping.m_imageline_dir = baseObj->m_options.prj_file.root_dir + "images/ImageLines/";
	texture_mapping.m_sampling_grid = sampling_grid;
	texture_mapping.m_max_view = max_view;

	if (!texture_mapping.runTextureMapping())
		return false;

	return true;
}

bool TextureMappingPlanes(ccHObject::Container primObjs, stocker::IndexVector * task_indices, double refine_length, double sampling_grid, int max_view)
{
	if (primObjs.empty()) return false;
	BDBaseHObject* baseObj = GetRootBDBase(primObjs.front());
	if (!baseObj) return false;

	stocker::TextureMapping texture_mapping;
	texture_mapping.setOutputDir(baseObj->getPath().toStdString() + "models");

	std::vector<stocker::ImageUnit> image_units = baseObj->GetImageData();
	for (stocker::ImageUnit & image_unit : image_units) {
		Vec3d view_pos = image_unit.GetViewPos();
		image_unit.m_camera.SetViewPoint(view_pos + baseObj->global_shift);
	}
	texture_mapping.setImages(image_units);

	//! should parse the task indices
	stocker::IndexVector ori_task_indices;
	if (task_indices) ori_task_indices = *task_indices;
	stocker::IndexVector parse_task_indices;
	size_t task_count(0);
	for (size_t pi = 0; pi < primObjs.size(); ++pi) {
		ccHObject* prim_entity = primObjs[pi];
		

	 	StBuilding* bdObj = GetParentBuilding(prim_entity);
		if (!bdObj) continue;
		BuildUnit build_unit = baseObj->GetBuildingUnit(bdObj->getName().toStdString());

		/// vis images
		IndexVector visImageIndice;
		for (auto img_name : build_unit.image_list) {
			auto img_iter = std::find_if(image_units.begin(), image_units.end(), [=](ImageUnit img) {
				return img_name == img.GetName().Str();
			});
			if (img_iter != image_units.end()) {
				visImageIndice.push_back(std::distance(image_units.begin(), img_iter));
			}
		}
		
		std::vector<stocker::Outline3d> prim_outlines = GetPlanarOutlines(prim_entity, BDDB_PLANEFRAME_PREFIX);
		std::string model_dir = build_unit.file_path.model_dir + prim_entity->getName().toStdString() + "/";
		for (size_t i = 0; i < prim_outlines.size(); i++) {
			stocker::Outline3d outline = prim_outlines[i];
			std::string outline_output_dir = model_dir + to_string(i);
			texture_mapping.addPlane(outline, visImageIndice, outline_output_dir);
			
			if (std::find(ori_task_indices.begin(), ori_task_indices.end(), pi) != ori_task_indices.end()) {
				parse_task_indices.push_back(task_count);
			}
			task_count++;
		}
		std::cout << prim_entity->getName().toStdString() << " added" << std::endl;
	}

	if (!parse_task_indices.empty()) {
		texture_mapping.setTaskIndice(parse_task_indices);
	}

	if (refine_length > 0) {
		texture_mapping.m_refineMesh = true;
		texture_mapping.m_refineLength = refine_length;
	}

	texture_mapping.m_imageline_dir = baseObj->m_options.prj_file.root_dir + "images/ImageLines/";	
	texture_mapping.m_sampling_grid = sampling_grid;
	texture_mapping.m_max_view = max_view;
	texture_mapping.m_remove_selfintersection = false;

	if (!texture_mapping.runTextureMapping())
		return false;

	return true;
}

ccHObject* ConstrainedMesh(ccHObject* planeObj, int rare_pts)
{
	ccHObject* plane_cloud_obj = planeObj->getParent();
	if (!planeObj->isA(CC_TYPES::PLANE) || !plane_cloud_obj) {
		return nullptr;
	}

	Contour3d plane_points = GetPointsFromCloud3d(plane_cloud_obj);
	vcg::Plane3d vcg_plane = GetVcgPlane(planeObj);
	PlaneUnit plane_unit = FormPlaneUnit("", vcg_plane, plane_points, false);
	Contour2d plane_points_2d = Point3dToPlpoint2d(plane_unit, plane_points);

	// prepare boundary lines
	Polyline3d boundary_lines; Contour3d boundary_points; {
		ccHObject::Container container_find, container_objs;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_BOUNDARY_PREFIX, true);
		if (!container_find.empty()) {
			GetBoundaryPointsAndLinesFromCloud(container_find.back(), boundary_lines, boundary_points);
		}
	}

	// prepare outline
	Contour3d outline_points; {
		ccHObject::Container container_find;
		planeObj->getParent()->filterChildrenByName(container_find, false, BDDB_OUTLINE_PREFIX, true);
		if (!container_find.empty()) {
			auto outlines_points_all = GetOutlinesFromOutlineParent(container_find.back());
			if (!outlines_points_all.empty()) {
				outline_points = outlines_points_all.front().front();
			}
		}
	}
	Contour2d outline_points_2d = Point3dToPlpoint2d(plane_unit, outline_points);
	Polyline2d outline_2d = MakeLoopPolylinefromContour(outline_points_2d);
	
	//Polyline3d outline_poly = MakeLoopPolylinefromContour(outline_points);

	Polyline2d sharp_2d = Line3dToPlline2d(plane_unit, boundary_lines);

	if (rare_pts > 0) {
		double area(-DBL_MAX);
		if (outline_points_2d.size() > 2) {
			area = CalcPolygonArea(outline_points_2d);
		}
		else {
			ccPlane* cc_Plane = ccHObjectCaster::ToPlane(planeObj); assert(cc_Plane);
			area = cc_Plane->getXWidth() * cc_Plane->getYWidth();
		}
		if (area > 0)
			rarefydata(plane_points, rare_pts * area);
	}

	Contour2d point_pool = plane_points_2d;
// 	for (auto pt : boundary_points) {		
// 		if (DistancePointPolygon(pt,line_pool) < 1)	continue;
// 		bool add = true;
// 		for (auto & pt_ : point_pool) {
// 			if (vcg::Distance(pt, pt_) < 1) {
// 				add = false;
// 				break;
// 			}			
// 		}
// 		if (!add) continue;
// 		
// 		point_pool.push_back(pt);
// 	}
	

	std::vector<Triangle2d> triangles_2d;
	PlaneTriangulationConstriantbySharp(point_pool, sharp_2d, outline_points_2d, triangles_2d, false, false);

	std::vector<Triangle3d> triangles_3d(triangles_2d.size());
	Concurrency::parallel_for(size_t(0), triangles_2d.size(), [&](size_t i) {
		triangles_3d[i] = Triangle3d(
			plane_unit.ToPlpoint3d(triangles_2d[i].P(0)),
			plane_unit.ToPlpoint3d(triangles_2d[i].P(1)),
			plane_unit.ToPlpoint3d(triangles_2d[i].P(2)));
	});

	GLMesh mesh_out;
	TrianglesToMesh(triangles_3d, mesh_out);

	ccPointCloud* vertices = new ccPointCloud("vertices");
	for (auto & pt : mesh_out.vert) {		
		vertices->addPoint(CCVector3(vcgXYZ(pt.P())));
	}
	if (vertices->reserveTheNormsTable()) {
		for (auto & pt : mesh_out.vert) {
			vertices->addNorm(CCVector3(vcgXYZ(plane_unit.plane.Direction())));
		}
	}
	
	ccMesh* mesh = new ccMesh(vertices);
	mesh->setName(GetBaseName(planeObj->getParent()->getName() + ".cdt"));
	mesh->addChild(vertices);
	ccPointCloud* point_cloud_entity = ccHObjectCaster::ToPointCloud(plane_cloud_obj);
	vertices->setGlobalShift(point_cloud_entity->getGlobalShift());
	vertices->setGlobalScale(point_cloud_entity->getGlobalScale());	

	for (auto & face : mesh_out.face) {
		mesh->addTriangle(
			vcg::tri::Index(mesh_out, face.cV(0)),
			vcg::tri::Index(mesh_out, face.cV(1)),
			vcg::tri::Index(mesh_out, face.cV(2)));
	}
	planeObj->getParent()->addChild(mesh);
	planeObj->getParent()->prepareDisplayForRefresh_recursive();

	return mesh;
}

bool DeduceFootPrintHeight(ccHObject* point_cloud, ccHObject* primitive, ccHObject* footprint, Contour3d & point_inside, double & height)
{
	if (!point_cloud && !primitive) return false;
	StFootPrint* polyObj = ccHObjectCaster::ToStFootPrint(footprint);
	assert(polyObj);
	Polyline3d polygon = GetPolygonFromPolyline(polyObj);
	
	Contour3d points;
	if (point_cloud) {
		points = GetPointsFromCloudInsidePolygonXY(point_cloud, polygon, -DBL_MAX);
	}
	else if (primitive) {
		points = GetPointsFromCloudInsidePolygonXY(primitive, polygon, -DBL_MAX);
	}
	if (points.empty()) {
		return false;
	}
	
	sort(points.begin(), points.end(), [&](Vec3d _l, Vec3d _r) {return _l.Z() < _r.Z(); });
	double min_height = points.front().Z();
	double max_height = points.back().Z();
	height = max_height;

	// partite into 0.5m / layer
	double step = 0.5;
	int layer_index(0);
	std::map<int, int> layer_count;
//	[ min_height + step * layer_index, min_height + step * (layer_index + 1) )

	for (size_t i = 0; i < points.size(); i++) {
		if (points[i].Z() > min_height + step * (layer_index + 1)) {
			layer_index++;
			continue;
		}
		layer_count[layer_index]++;
		if (min_height + step * (layer_index + 1) >= max_height) {
			break;
		}
	}
	ccPlane* cc_plane = nullptr;
	for (auto layer : layer_count) {
		if (layer.second > 0.8*points.size()) {
			height = layer.first*step + min_height;
		}			
	}
	//! check if there is a close one
	if (primitive) {

	}	
	return true;
}

std::vector<std::vector<int>> GroupFootPrint(ccHObject::Container footprintObjs)
{
	// TODO: get the relationships of the polygons
	// now the polygons should not be overlapped
	// if more than one polygons are given, will create multiple models	
	std::vector<std::vector<int>> components;
	for (size_t i = 0; i < footprintObjs.size(); i++) {
		std::vector<int> compo_temp;
		compo_temp.push_back(i);
		components.push_back(compo_temp);
	}
	return components;
}

bool isVertical(ccPlane* planeObj, double angle_degree = 15)
{
	return planeObj->isVerticalToDirection(CCVector3(0, 0, 1), angle_degree);
}

ccHObject::Container GetNonVerticalPlaneClouds(ccHObject* stprim_group, double angle_degree = 15, ccHObject::Container* vertical = nullptr)
{
	ccHObject::Container primObjs_temp = GetEnabledObjFromGroup(stprim_group, CC_TYPES::PLANE, true, true);
	ccHObject::Container primObjs;
	for (auto & plane_entity : primObjs_temp) {
		ccPlane* planeObj = ccHObjectCaster::ToPlane(plane_entity);
		if (!planeObj || !planeObj->isEnabled()) { continue; }
		//! skip vertical
		if (isVertical(planeObj, angle_degree)) {
			if (vertical) vertical->push_back(planeObj->getParent());
		}
		else {
			if (!planeObj->getParent() || !planeObj->getParent()->isEnabled()) { continue; }
			primObjs.push_back(planeObj->getParent());
		}
	}
	return primObjs;
}

ccHObject::Container GenerateFootPrints_PP(ccHObject* prim_group, double ground)
{
	ccHObject::Container foot_print_objs;
	BDBaseHObject* baseObj = GetRootBDBase(prim_group);
	if (!baseObj) { return foot_print_objs; }

	//! skip walls
	ccHObject::Container primObjs = GetNonVerticalPlaneClouds(prim_group, 15);

	std::vector<stocker::Contour3d> planes_points = GetPointsFromCloudInsidePolygonsXY(primObjs, stocker::Polyline3d(), false);
	if (planes_points.empty()) { return foot_print_objs; }

	std::vector<std::vector<Contour3d>> components_foots;
	std::vector<std::vector<double>> components_top_heights;
	std::vector<std::vector<double>> components_bottom_heights;
	DeriveRoofLayerFootPrints_PP(planes_points, components_foots, components_top_heights, components_bottom_heights, 1, false, 1, 50, 50);

	int cur_compo_count = 0; // TODO: get component count from block
	
	QString building_name = GetBaseName(prim_group->getName());
	auto buildUnit = baseObj->GetBuildingUnit(building_name.toStdString());

	StBlockGroup* block_group = baseObj->GetBlockGroup(building_name);
	int biggest = GetMaxNumberExcludeChildPrefix(block_group, BDDB_FOOTPRINT_PREFIX);
	assert(components_foots.size() == components_top_heights.size());
	for (size_t i = 0; i < components_foots.size(); i++) {
		int compoId = cur_compo_count + i;
		assert(components_top_heights[i].size() == components_foots[i].size());
		std::vector<double> footprints_tops = components_top_heights[i];
		for (size_t j = 0; j < components_foots[i].size(); j++) {
			Contour3d foot_print = components_foots[i][j];
			double top_height = components_top_heights[i][j];
			double bottom_height = components_bottom_heights[i][j];
			QString name = BDDB_FOOTPRINT_PREFIX + QString::number(++biggest);
			StFootPrint* footptObj = AddPolygonAsFootprint(foot_print, name, ccColor::magenta, true);
			if (!footptObj) continue;

			footptObj->setComponentId(compoId);
			footptObj->setHighest(top_height);
			footptObj->setBottom(ground);
			footptObj->setLowest(bottom_height);

			foot_print_objs.push_back(footptObj);
			block_group->addChild(footptObj);
		}
	}
	return foot_print_objs;
}

ccHObject::Container GenerateFootPrints(ccHObject* prim_group, double ground)
{
	ccHObject::Container foot_print_objs;
	BDBaseHObject* baseObj = GetRootBDBase(prim_group);
	if (!baseObj) { return foot_print_objs; }

	//! skip walls
	ccHObject::Container primObjs = GetNonVerticalPlaneClouds(prim_group, 15);

	std::vector<stocker::Contour3d> planes_points = GetPointsFromCloudInsidePolygonsXY(primObjs, stocker::Polyline3d(), DBL_MAX, true);
	if (planes_points.empty()) { return foot_print_objs; }

	std::vector<Contour3d> footprints;
	std::vector<std::vector<Contour3d>> fps_planes;
	if (!DeriveRoofLayerFootPrints(planes_points, footprints, fps_planes, 1, false, 1, 50, 50))
	if (footprints.size() != fps_planes.size()) {
		return foot_print_objs;
	}

	MainWindow* win = MainWindow::TheInstance();
	if (!win) return foot_print_objs;

	for (ccHObject* prim : primObjs) {
		win->removeFromDB(prim);
// 		QString del_name = "del-" + prim->getName();
// 		prim->setName(del_name);
// 		prim->setEnabled(false);
	}
	ccHObject::Container new_primObjs;

	QString building_name = GetBaseName(prim_group->getName());
	auto buildUnit = baseObj->GetBuildingUnit(building_name.toStdString());

	StBlockGroup* block_group = baseObj->GetBlockGroup(building_name);
	int biggest = GetMaxNumberExcludeChildPrefix(block_group, BDDB_FOOTPRINT_PREFIX);
	for (size_t i = 0; i < footprints.size(); i++) {
		Contour3d foot_print = footprints[i];
		std::vector<Contour3d> foot_planes = fps_planes[i];

		//! get names and top and bottom height from planes
		QStringList cur_plane_names;
		Contour3d cur_all_points;
		for (Contour3d plane_points : foot_planes) {
			int num = GetMaxNumberExcludeChildPrefix(prim_group, "Plane");
			QString plane_name = "Plane" + QString::number(++num);
			ccPointCloud* cloud = AddPointsAsPlane(plane_points, plane_name, ccColor::Generator::Random());
			prim_group->addChild(cloud);
			new_primObjs.push_back(cloud);

			cur_plane_names.append(plane_name);
			cur_all_points.insert(cur_all_points.end(), plane_points.begin(), plane_points.end());
		}
		Concurrency::parallel_sort(cur_all_points.begin(), cur_all_points.end(), [&](Vec3d l, Vec3d r) {return l.Z() < r.Z(); });
		double top_height = cur_all_points.back().Z();
		double bottom_height = cur_all_points.front().Z();

		//! construct the footprint
		QString name = BDDB_FOOTPRINT_PREFIX + QString::number(++biggest);
		StFootPrint* footptObj = AddPolygonAsFootprint(foot_print, name, ccColor::magenta, true);
		if (!footptObj) continue;

		footptObj->setComponentId(0);
		footptObj->setHighest(top_height);
		footptObj->setBottom(ground);
		footptObj->setLowest(bottom_height);

		footptObj->setPlaneNames(cur_plane_names);

		foot_print_objs.push_back(footptObj);
		block_group->addChild(footptObj);
	}

	prim_group->setDisplay_recursive(prim_group->getDisplay());
	for (ccHObject* plane : new_primObjs) {
		plane->setDisplay_recursive(prim_group->getDisplay());
		ccHObject* p = GetPlaneFromCloud(plane);
		if (p) { p->setVisible(false); }
	}
	
	return foot_print_objs;
}

ccHObject* LoD1FromFootPrint_Flat(ccHObject* ftObj)
{
	ccHObject* block_entity = nullptr;

	return block_entity;
}

ccHObject* LoD1FromFootPrint_Planar(ccHObject* entity)
{
	ccPlane* planeObj = GetPlaneFromPlaneOrCloud(entity);
	if (!planeObj) {
		return nullptr;
	}
	ccPointCloud* plane_cloud = GetPlaneCloud(planeObj);

	StFootPrint* footprintObj = nullptr;
	if (entity->isA(CC_TYPES::ST_FOOTPRINT)) {
		footprintObj = ccHObjectCaster::ToStFootPrint(entity); if (!footprintObj) return nullptr;
		

	}
	else {
		
		ccPlane* planeObj = GetPlaneFromPlaneOrCloud(entity);
		
		
	}
	ccHObject* block_entity = nullptr;
	if (block_entity) {
		footprintObj->addChild(block_entity); 
	}
	return block_entity;
}

ccHObject* LoD1FromFootPrint(ccHObject* buildingObj)
{	
	BDBaseHObject* baseObj = GetRootBDBase(buildingObj);
	if (!baseObj) {
		return nullptr;
	}

	QString building_name = GetBaseName(buildingObj->getName());
	ccHObject* cloudObj = baseObj->GetOriginPointCloud(building_name, true);
	ccHObject* prim_group_obj = baseObj->GetPrimitiveGroup(building_name);

	StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(building_name);
	ccHObject::Container footprintObjs = blockgroup_obj->getValidFootPrints();

	int biggest = GetMaxNumberExcludeChildPrefix(blockgroup_obj, BDDB_BLOCK_PREFIX);
	for (size_t i = 0; i < footprintObjs.size(); i++) {
		StFootPrint* foot_print = ccHObjectCaster::ToStFootPrint(footprintObjs[i]);
		if (!foot_print || !foot_print->isEnabled()) continue;
		
		//! get height
		Contour3d points_inside;
		double height = foot_print->getHeight();
		double ground = foot_print->getBottom();

		if (fabs(height - ground) < 1e-6) {
			if (!DeduceFootPrintHeight(cloudObj, prim_group_obj, footprintObjs[i], points_inside, height)) {
				std::cout << "cannot deduce height from footprint " << foot_print->getName().toStdString() << std::endl;
				continue;
			}			
		}		
		
		std::vector<CCVector3> foot_print_points = foot_print->getPoints(false);

		std::vector<CCVector3> top_points;
		//std::vector<CCVector3> bottom_points;
		for (auto & pt : foot_print_points) {
			top_points.push_back(CCVector3(pt.x, pt.y, height));
			//bottom_points.push_back(CCVector3(pt.x, pt.y, ground));
		}
		StBlock* block_entity = StBlock::Create(top_points, ground);
		block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(++biggest));
		foot_print->addChild(block_entity);
	}
	return blockgroup_obj;
}

//! 3D4EM	.lod2.model
ccHObject* LoD2FromFootPrint_WholeProcess(ccHObject* buildingObj, ccHObject::Container footprintObjs, double ground_height)
{
	if (!buildingObj) { return nullptr; }
	BDBaseHObject* baseObj = GetRootBDBase(buildingObj);
	if (!baseObj) { return nullptr;	}
	QString building_name = buildingObj->getName();
	BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
	StPrimGroup* prim_group_obj = baseObj->GetPrimitiveGroup(building_name);
	StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName());
	if (!prim_group_obj || !blockgroup_obj) { return nullptr; }

	bool use_footprint = true;
	if (footprintObjs.empty()) {
		use_footprint = false;		
		Contour3d convex_points;
		for (auto & pt : build_unit.convex_hull_xy)	{
			convex_points.push_back(Vec3d(pt.X(), pt.Y(), ground_height));
		}
		Polyline3d convex_lines = MakeLoopPolylinefromContour(convex_points);
		ccPointCloud* convex_cloud = AddPointsAsPointCloud(convex_points, "vertices");
		convex_cloud->setDisplay(buildingObj->getDisplay());
		convex_cloud->setGlobalShift(CCVector3d(vcgXYZ(baseObj->global_shift)));
		convex_cloud->setGlobalScale(baseObj->global_scale);
		StFootPrint* convex_hull = new StFootPrint(convex_cloud);
		convex_hull->setName(BDDB_FOOTPRINT_PREFIX + QString::number(0));
		convex_hull->setColor(ccColor::magenta);
		convex_hull->showColors(true);
		convex_hull->reserve(convex_points.size());
		for (size_t i = 0; i < convex_points.size(); i++) {
			convex_hull->addPointIndex(i, (i + 1) % convex_points.size());		
		}
		convex_hull->setClosed(false);
		convex_hull->addChild(convex_cloud);
		blockgroup_obj->addChild(convex_hull);
		footprintObjs.push_back(convex_hull);
	}

	std::vector<std::vector<int>> components = GroupFootPrint(footprintObjs);

	for (size_t i = 0; i < components.size(); i++) {
		stocker::BuilderLOD2 builder_3d4em(true);

		std::vector<Contour3d> contours;
		std::vector<int> cur_component = components[i];
		Polyline3d first_polygon; double footprint_height(-999999);
		bool valid = false;
		ccHObject* first_footprint = nullptr;
		QStringList plane_names;
		for (size_t j = 0; j < cur_component.size(); j++) {
			first_footprint = footprintObjs[cur_component[j]];
			if (!first_footprint->isEnabled()) {
				continue;
			}
			Polyline3d polygon = GetPolygonFromPolyline(first_footprint);
			if (j == 0) {
				valid = true;
				first_polygon = polygon;
				footprint_height = ccHObjectCaster::ToStFootPrint(first_footprint)->getHighest();
				plane_names = ccHObjectCaster::ToStFootPrint(first_footprint)->getPlaneNames();
			}
			contours.push_back(ToContour(polygon, 0));
		}
		if (!valid) {
			continue;
		}
		if (use_footprint) {
			builder_3d4em.SetFootPrint(contours);
		}

		char output_path[256];
		{
			//! just for debug 
			QString model_name = BDDB_LOD2MODEL_PREFIX + QString::number(i);
			sprintf(output_path, "%s%s%s%s%s",
				build_unit.file_path.model_dir.c_str(),
				building_name.toStdString().c_str(), ".", model_name.toStdString().c_str(), ".obj");
			builder_3d4em.SetOutputPath(output_path);
		}
		builder_3d4em.SetGroundHeight(ground_height);
		
		ccHObject::Container primObjs;
		if (!plane_names.empty()) {
			for (size_t pi = 0; pi < prim_group_obj->getChildrenNumber(); pi++) {
				ccHObject* child_cloud = prim_group_obj->getChild(pi); if (!child_cloud) continue;
				if (plane_names.indexOf(child_cloud->getName()) >= 0) {
					primObjs.push_back(child_cloud);
				}
			}
		}
		else {
			primObjs = GetNonVerticalPlaneClouds(prim_group_obj, 15);
		}
		if (primObjs.empty()) {	continue; }
		std::vector<Contour3d> points = GetPointsFromCloudInsidePolygonsXY(primObjs, first_polygon, footprint_height);
		builder_3d4em.SetSegmentedPoints(points);		

		if (!builder_3d4em.BuildingReconstruction()) continue;

		if (!QFile::exists(QString(output_path))) continue;

		std::vector<Contour3d> roof_polygons = builder_3d4em.GetRoofPolygons();
		int block_number = GetMaxNumberExcludeChildPrefix(blockgroup_obj, BDDB_BLOCK_PREFIX) + 1;
		for (Contour3d roof_points : roof_polygons) {
			std::vector<CCVector3> top_points;
			//std::vector<CCVector3> bottom_points;
			for (auto & pt : roof_points) {
				top_points.push_back(CCVector3(vcgXYZ(pt)));
				//bottom_points.push_back(CCVector3(pt.X(), pt.Y(), ground_height));
			}
			StBlock* block_entity = StBlock::Create(top_points, ground_height);
			if (block_entity) {
				block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(block_number++));
				first_footprint->addChild(block_entity);
			}
		}
	}
	return blockgroup_obj;
}

ccHObject* LoD2FromFootPrint(ccHObject* buildingObj, ccHObject::Container footprintObjs)
{
	if (!buildingObj || footprintObjs.empty()) { return nullptr; }
	BDBaseHObject* baseObj = GetRootBDBase(buildingObj);
	if (!baseObj) { return nullptr; }
	QString building_name = buildingObj->getName();
	BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
	ccHObject* prim_group_obj = baseObj->GetPrimitiveGroup(building_name);
	StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName());
	if (!prim_group_obj || !blockgroup_obj) { return nullptr; }

	for (size_t i = 0; i < footprintObjs.size(); i++) {
		StFootPrint* ftObj = ccHObjectCaster::ToStFootPrint(footprintObjs[i]);
		if (!ftObj) { continue; }

		Polyline3d polygon = GetPolygonFromPolyline(ftObj);
		stocker::BuilderLOD2 builder_3d4em(true);
		if (polygon.size() >= 3) {
			builder_3d4em.SetFootPrint(ToContour(polygon, 0));
		}

		char output_path[256];
		{
			//! just for debug 
			QString model_name = BDDB_LOD2MODEL_PREFIX + ftObj->getName();
			sprintf(output_path, "%s%s%s%s%s",
				build_unit.file_path.model_dir.c_str(),
				building_name.toStdString().c_str(), ".", model_name.toStdString().c_str(), ".obj");
			builder_3d4em.SetOutputPath(output_path);
		}

		double ground_height = ftObj->getBottom();
		builder_3d4em.SetGroundHeight(ground_height);

		ccHObject::Container primObjs;
		QStringList plane_names = ftObj->getPlaneNames();
		if (!plane_names.empty()) {
			for (size_t pi = 0; pi < prim_group_obj->getChildrenNumber(); pi++) {
				ccHObject* child_cloud = prim_group_obj->getChild(pi); if (!child_cloud) continue;
//#ifdef _DEBUG
				if (!child_cloud->isEnabled()) continue;
//#endif // _DEBUG

				if (plane_names.indexOf(child_cloud->getName()) >= 0) {
					primObjs.push_back(child_cloud);
				}
			}
		}
		else {
			primObjs = GetNonVerticalPlaneClouds(prim_group_obj, 15);
		}
		if (primObjs.empty()) {
			// TODO: LOD1
			std::vector<CCVector3> top_points;
			for (auto pt : ToContour(polygon, 0)) {
				top_points.push_back(CCVector3(pt.X(), pt.Y(), pt.Z()));
			}
			StBlock* block_entity = StBlock::Create(top_points, ground_height);
			int block_number = GetMaxNumberExcludeChildPrefix(ftObj, BDDB_BLOCK_PREFIX) + 1;
			block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(block_number++));
			ftObj->addChild(block_entity);
			continue;
		}


		for (auto & pt : polygon) {
			pt.P0().Z() = ftObj->getLowest();
			pt.P1().Z() = ftObj->getLowest();
		}
		std::vector<Contour3d> points = GetPointsFromCloudInsidePolygonsXY(primObjs, polygon, ftObj->getHighest());
		builder_3d4em.SetSegmentedPoints(points);

		try {
			if (!builder_3d4em.BuildingReconstruction()) continue;
		}
		catch (const std::exception& e) {
			STOCKER_ERROR_ASSERT(e.what());
			continue;
		}
		catch (...) {
			continue;
		}

		if (!QFile::exists(QString(output_path))) continue;

		std::vector<Contour3d> roof_polygons = builder_3d4em.GetRoofPolygons();
		int block_number = GetMaxNumberExcludeChildPrefix(ftObj, BDDB_BLOCK_PREFIX) + 1;
		for (Contour3d & roof_points : roof_polygons) {
			std::vector<CCVector3> top_points;
			for (auto & pt : roof_points) {
				top_points.push_back(CCVector3(vcgXYZ(pt)));
			}

			StBlock* block_entity = nullptr;
			try	{
				block_entity = StBlock::Create(top_points, ground_height);
			}
			catch (const std::exception& e)	{
				if (block_entity) {
					delete block_entity;
					block_entity = nullptr;
				}
				continue;
			}
			if (block_entity) {
				block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(block_number++));
				ftObj->addChild(block_entity);
			}
		}
	}
	return blockgroup_obj;
}

#include <QMessageBox>
ccHObject* LoD2FromFootPrint(ccHObject* entity)
{
	ccHObject* buildingObj = nullptr;
	ccHObject::Container footprintObjs;
	if (entity->isA(CC_TYPES::ST_FOOTPRINT)) {
		footprintObjs.push_back(entity);
		buildingObj = GetParentBuilding(entity);
	}
	else if (entity->isA(CC_TYPES::ST_BUILDING)) {
		BDBaseHObject* baseObj = GetRootBDBase(entity);
		if (!baseObj) {
			return nullptr;
		}
		buildingObj = entity;
		StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName());
		footprintObjs = blockgroup_obj->getValidFootPrints();
	}
	
	return LoD2FromFootPrint(buildingObj, footprintObjs);
}

void SubstituteFootPrintContour(StFootPrint* footptObj, stocker::Contour3d points)
{
	ccHObject* existing_cloud = nullptr;
	for (size_t ci = 0; ci < footptObj->getChildrenNumber(); ci++) {
		ccHObject* child = footptObj->getChild(ci);
		if (child->isA(CC_TYPES::POINT_CLOUD) && child->getName() == "vertices") {
			existing_cloud = child;
			break;
		}
	}
	MainWindow* win = MainWindow::TheInstance();
	ccPointCloud* vertices = AddPointsAsPointCloud(points, "vertices", ccColor::magenta);
	footptObj->clear();
	footptObj->initWith(vertices, *footptObj);
	footptObj->addPointIndex(0);
	footptObj->setClosed(true);
	win->addToDB(vertices, footptObj->getDBSourceType(), false, false);

	if (existing_cloud) {
		win->db(footptObj->getDBSourceType())->unselectAllEntities();
		win->db(footptObj->getDBSourceType())->selectEntity(existing_cloud);
		win->db(footptObj->getDBSourceType())->deleteSelectedEntities();
	}
}

ccHObject::Container PackPolygons(ccHObject::Container polygonEntites, int sample)
{
	ccHObject::Container output_polygons;
	std::vector<stocker::Contour3d> footprints_points_pp;

	std::vector<stocker::Polyline3d> pp_polygons;
	std::vector<stocker::Contour3d> pp_poly_points;

	ccBBox box;
	for (ccHObject* polyObj : polygonEntites) {
		box += polyObj->getOwnBB();
	}
	
	float sample_dist = box.getDiagNorm() / (float)sample;
 	vcg::Box2f box_2f = vcg::Box2f({ box.minCorner().x,box.minCorner().y }, { box.maxCorner().x,box.maxCorner().y });
 	std::vector<stocker::Vec2_<float>> pts_2f = stocker::PointSampleInBox2<float>(box_2f, sample_dist);

	//grid failed, don't know why
// 	std::vector<stocker::Vec2T_<float>> pts_2Tf(pts_2f.begin(), pts_2f.end());
// 	vcg::GridStaticPtr2D<stocker::Vec2T_<float>, float> Grid2D;
// 	Grid2D.Set(pts_2Tf.begin(), pts_2Tf.end());

 	pp_poly_points.resize(polygonEntites.size());
	pp_polygons.resize(polygonEntites.size());
 	Concurrency::parallel_for(size_t(0), polygonEntites.size(), [&](size_t i) {
	//for (size_t i = 0; i < polygonEntites.size(); ++i) {
		pp_polygons[i] = GetPolygonFromPolyline(polygonEntites[i]);

		ccBBox box = polygonEntites[i]->getOwnBB();
		for (auto pt : pts_2f) {
			if (box.contains(CCVector3(pt.X(),pt.Y(), box.getCenter().z))) {
				pp_poly_points[i].push_back({ pt.X(),pt.Y(),box.getCenter().z });
			}
		}

// 		vcg::PMarker2<stocker::Vec2T_<float>> TM;
// 		std::vector<stocker::Vec2T_<float>*> p_pts;
// 		Grid2D.GetInBox(TM, box_2f, p_pts);
// 		for (auto pt : p_pts) {
// 			pp_poly_points[i].push_back({ (*pt).X(),(*pt).Y(),0 });
// 		}
	//}
 	});
	
	for (size_t i = 0; i < pp_poly_points.size(); i++) {
		ccPointCloud* new_pc = AddPointsAsPointCloud(pp_poly_points[i], "points", ccColor::Generator::Random());
		if (new_pc) {
			polygonEntites[i]->addChild(new_pc);
			MainWindow::TheInstance()->addToDB(new_pc, polygonEntites[i]->getDBSourceType());
		}
	}

	stocker::PolygonPartition poly_partition;
	poly_partition.setOutputDir("./Output");
	poly_partition.m_max_iter = 1;
	poly_partition.m_run_cap_hole = false;
	poly_partition.setPolygon(pp_polygons, pp_poly_points);
	if (!poly_partition.runBP()) {
		return output_polygons;
	}
	std::vector<std::pair<size_t, stocker::Outline3d>> results = poly_partition.getResultPolygon();
	for (auto & NO_poly : results) {
		stocker::Outline3d poly = NO_poly.second;
		if (poly.empty()) continue;
		//! for now, no holes for footprint
		footprints_points_pp.push_back(ToContour(poly.front(), 0));
		ccPolyline* new_poly = AddPolygonAsPolyline(poly.front(), "temp", ccColor::Generator::Random(), true);
		output_polygons.push_back(new_poly);
	}
	return output_polygons;
}

bool PackPlaneFrames(ccHObject* buildingObj, int max_iter, bool cap_hole, double ptsnum_ratio, double data_ratio,
	double ints_thre, double cluster_hori, double cluster_verti)
{
	MainWindow* win = MainWindow::TheInstance(); if (!win) return false;
	try {
		BDBaseHObject* baseObj = GetRootBDBase(buildingObj); if (!baseObj) return false;
		QString building_name = buildingObj->getName();
		BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
		StPrimGroup* prim_group_obj = baseObj->GetPrimitiveGroup(building_name);
		if (!prim_group_obj) return false;

		ccHObject::Container horizontal_planes, vertical_planes;
		horizontal_planes = GetNonVerticalPlaneClouds(prim_group_obj, 15, &vertical_planes);

		//! generate intersection lines
		//stocker::PlaneData plane_units;
		std::vector<stocker::Contour3d> planes_points;
		std::vector<stocker::Polyline3d> planes_frames;
		IndexVector planes_original_index;

		for (size_t i = 0; i < horizontal_planes.size(); i++) {
			ccHObject* plane_entity = horizontal_planes[i];
			stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(plane_entity);
			if (cur_plane_points.size() < 3) continue;

			Contour3d outline_points; {
				ccHObject::Container container_find;
				plane_entity->filterChildrenByName(container_find, false, BDDB_PLANEFRAME_PREFIX, true);
				if (!container_find.empty()) {
					auto outlines_points_all = GetOutlinesFromOutlineParent(container_find.back());
					if (!outlines_points_all.empty()) {
						outline_points = outlines_points_all.front().front();
					}
				}
				
			}
			if (outline_points.size() < 3) continue;
			
			planes_frames.push_back(MakeLoopPolylinefromContour(outline_points));
			//plane_units.push_back(FormPlaneUnit(cur_plane_points, plane_entity->getName().toStdString(), true));
			planes_points.push_back(cur_plane_points);
			planes_original_index.push_back(i);
		}

// 		stocker::Polyline3d intersections;
// 		for (size_t i = 0; i < plane_units.size() - 1; i++)	{
// 			for (size_t j = i + 1; j < plane_units.size(); j++) {
// 				stocker::Seg3d ints_seg;
// 				if (stocker::IntersectionPlanePlaneStrict(plane_units[i], plane_units[j], ints_seg, ints_thre)) {
// 					intersections.push_back(ints_seg);
// 				}
// 			}
// 		}
// 		stocker::SimpleSaveToPly("d:/ints_original.ply",Contour3d(), intersections);
		//Polyline2d ints_2d = ToPolyline2d(intersections);
		//facade_projected.insert(facade_projected.end(), ints_2d.begin(), ints_2d.end());
		//intersections = ClusterSegments(intersections, cluster_hori, cluster_verti);
		//stocker::SimpleSaveToPly("d:/ints_clustered.ply", Contour3d(), intersections);

		std::vector<stocker::Seg2d> facade_projected;
		for (ccHObject* planeEnt : vertical_planes) {
			ccPlane* planeObj = GetPlaneFromPlaneOrCloud(planeEnt); if (!planeObj) continue;
			std::vector<CCVector3> profile = planeObj->getProfile();
			stocker::Contour2d profile_pts_proj;
			for (auto pt : profile) { profile_pts_proj.push_back(stocker::Vec2d(pt.x, pt.y)); }
			stocker::Seg2d seg;
			double seg_q = stocker::FitSegment2d(profile_pts_proj, seg);
			if (_finite(seg_q)) facade_projected.push_back(seg);
		}

		
		facade_projected = stocker::ClusterSegments(facade_projected, cluster_hori, cluster_verti);

		std::vector<stocker::Contour3d> result_planes_frames;
		IndexVector result_planes_index;
		{
			stocker::PolygonPartition poly_partition;
			std::string output_dir;
			if (!buildingObj->getPath().isEmpty()) {
				output_dir = QFileInfo(buildingObj->getPath()).absoluteFilePath().toStdString() + "/footprints";
			}
			else {
				output_dir = QFileInfo(QString::fromStdString(build_unit.file_path.ori_points)).absolutePath().toStdString() + "/footprints";
			}
			poly_partition.setOutputDir(output_dir);

			poly_partition.m_ints_thre = ints_thre;
			poly_partition.m_max_iter = max_iter;
			poly_partition.m_run_cap_hole = cap_hole;
			poly_partition.m_data_pts_ratio = ptsnum_ratio;
			poly_partition.m_data_ratio = data_ratio;

			//TODO: should give outlines rather than polygons
			poly_partition.setPolygon(planes_frames, planes_points);

			poly_partition.setFacades(facade_projected);

			if (!poly_partition.runBP()) {
				return false;
			}
			std::vector<std::pair<size_t, Outline3d>> results = poly_partition.getResultPolygon();
			for (auto & NO_poly : results) {
				if (NO_poly.second.empty()) continue;
				//! for now, no holes for footprint
				result_planes_index.push_back(NO_poly.first);
				result_planes_frames.push_back(ToContour(NO_poly.second.front(), 0));
			}
		}

		if (result_planes_frames.empty() || result_planes_frames.size() != result_planes_index.size()) return false;
		
		for (size_t i = 0; i < result_planes_frames.size(); i++) {
			size_t origin_index = result_planes_index[i];
			ccHObject* plane_entity = horizontal_planes[origin_index];

			/// clear all the old frames
			ccHObject::Container container_find;
			plane_entity->filterChildrenByName(container_find, false, BDDB_PLANEFRAME_PREFIX, true);
			for (ccHObject* del : container_find) {
				win->removeFromDB(del);
			}
			vector<vector<Contour3d>> frames_to_add(1);
			frames_to_add.back().push_back(result_planes_frames[i]);

			ccHObject* plane_frame = AddOutlinesAsChild(frames_to_add, BDDB_PLANEFRAME_PREFIX, plane_entity);
		}
	}
	catch (const std::exception&e) {
		STOCKER_ERROR_ASSERT(e.what());
		return false;
	}
	catch (...) {
		STOCKER_ERROR_ASSERT("internal error!");
		return false;
	}

	return true;
}

bool PackFootprints_PPP(ccHObject* buildingObj, int max_iter, bool cap_hole, double ptsnum_ratio, double data_ratio)
{
	try {
		BDBaseHObject* baseObj = GetRootBDBase(buildingObj); if (!baseObj) return false;
		QString building_name = buildingObj->getName();
		BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
		StPrimGroup* prim_group_obj = baseObj->GetPrimitiveGroup(building_name);
		StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName());
		if (!prim_group_obj || !blockgroup_obj) { return false; }

		//! get footprints
		ccHObject::Container footprints = blockgroup_obj->getValidFootPrints();
		std::vector<std::vector<Contour3d>> layers_planes_points;
		std::vector<stocker::Contour3d> footprints_points;
		IndexVector footprints_original_index;
		
		for (size_t i = 0; i < footprints.size(); ++i) {
			StFootPrint* ftObj = ccHObjectCaster::ToStFootPrint(footprints[i]);
			QStringList plane_names = ftObj->getPlaneNames();
			std::vector<Contour3d> planes_points;
			for (auto & pl_name : plane_names) {
				ccPlane* pl_entity = prim_group_obj->getPlaneByName(pl_name); if (!pl_entity) continue;
				if (!isVertical(pl_entity, 15)) {
					ccPointCloud* plane_cloud = GetPlaneCloud(pl_entity); if (!plane_cloud) continue;
					planes_points.push_back(GetPointsFromCloud3d(plane_cloud));
				}
			}
			layers_planes_points.push_back(planes_points);

			Contour3d ft_pts;
			for (auto & pt : ftObj->getPoints(true)) {
				ft_pts.emplace_back(pt.x, pt.y, pt.z);
			}
			if (ft_pts.size() < 3) continue;

			footprints_points.push_back(ft_pts);
			footprints_original_index.push_back(i);
		}

		std::vector<stocker::Seg2d> facade_projected;
		ccHObject::Container all_planes = prim_group_obj->getValidPlanes();
		for (ccHObject* planeEnt : all_planes) {
			ccPlane* planeObj = ccHObjectCaster::ToPlane(planeEnt); if (!planeObj) continue;
			if (isVertical(planeObj, 15)) {
				std::vector<CCVector3> profile = planeObj->getProfile();
				stocker::Contour2d profile_pts_proj;
				for (auto pt : profile) { profile_pts_proj.push_back(stocker::Vec2d(pt.x, pt.y)); }
				stocker::Seg2d seg;
				double seg_q = stocker::FitSegment2d(profile_pts_proj, seg);
				if (_finite(seg_q)) facade_projected.push_back(seg);
			}
		}

		std::vector<stocker::Contour3d> footprints_points_pp;
		IndexVector footprints_pp_index;
		{
			std::vector<Polyline3d> polygons;
			std::vector<stocker::Polygon2d> polygons_2d;
			std::vector<Contour3d> polygons_points;
			for (auto & poly : footprints_points) {
				stocker::Polyline3d poly_3d = stocker::MakeLoopPolylinefromContour(poly);
				polygons.push_back(poly_3d);
				polygons_2d.push_back(ToPolyline2d(poly_3d));
			}
			for (auto & poly : layers_planes_points) {
				Contour3d pts;
				for (auto & pt : poly) {
					pts.insert(pts.end(), pt.begin(), pt.end());
				}
				polygons_points.push_back(pts);
			}

			stocker::PolygonPartition poly_partition;
			std::string output_dir;
			if (!buildingObj->getPath().isEmpty()) {
				output_dir = QFileInfo(buildingObj->getPath()).absoluteFilePath().toStdString() + "/footprints";
			}
			else {
				output_dir = QFileInfo(QString::fromStdString(build_unit.file_path.ori_points)).absolutePath().toStdString() + "/footprints";
			}
			poly_partition.setOutputDir(output_dir);

			poly_partition.m_max_iter = max_iter;
			poly_partition.m_run_cap_hole = cap_hole;
			poly_partition.m_data_pts_ratio = ptsnum_ratio;
			poly_partition.m_data_ratio = data_ratio;

			//TODO: should give outlines rather than polygons
			poly_partition.setPolygon(polygons, polygons_points, true, false);

			stocker::SimpleSaveToPly("d:/projected.ply", Contour3d(), ToPolyline3d(facade_projected));
			stocker::Polyline2d facade_clustered = stocker::ClusterSegments(facade_projected, 1, 0.1);

			facade_clustered = stocker::collectCloseSegmentsAroundPolygons(polygons_2d, facade_clustered, 2);
			stocker::SimpleSaveToPly("d:/clustered.ply", Contour3d(), ToPolyline3d(facade_clustered));
					
			poly_partition.setFacades(facade_clustered);

			if (!poly_partition.runBP()) {
				return false;
			}
			std::vector<std::pair<size_t, Outline3d>> results = poly_partition.getResultPolygon();
			for (auto & NO_poly : results) {
				if (NO_poly.second.empty()) continue;
				//! for now, no holes for footprint
				footprints_pp_index.push_back(NO_poly.first);
				footprints_points_pp.push_back(ToContour(NO_poly.second.front(), 0));
			}
		}

		if (footprints_points_pp.empty() || footprints_points_pp.size() != footprints_pp_index.size()) return false;

		for (size_t i = 0; i < footprints.size(); i++) {
			footprints[i]->setEnabled(false);
			footprints[i]->setName("del-" + footprints[i]->getName());
		}
		int biggest = GetMaxNumberExcludeChildPrefix(blockgroup_obj, BDDB_FOOTPRINT_PREFIX);
		for (size_t i = 0; i < footprints_points_pp.size(); i++) {
			QString name = BDDB_FOOTPRINT_PREFIX + QString::number(++biggest);
			StFootPrint* footptObj = AddPolygonAsFootprint(footprints_points_pp[i], name, ccColor::magenta, true);

			footptObj->setComponentId(0);
			footptObj->setHighest(build_unit.ground_height);
			footptObj->setBottom(build_unit.ground_height);
			footptObj->setLowest(build_unit.ground_height);
			if (footprints_pp_index[i] >= footprints_original_index.size()) {
				std::cout << "--STOCKER FAIL: error footprint index" << std::endl;
				return false;
			}
			size_t original_index = footprints_original_index[footprints_pp_index[i]];
			StFootPrint* ftOriObj = ccHObjectCaster::ToStFootPrint(footprints[original_index]);
			if (ftOriObj) footptObj->setPlaneNames(ftOriObj->getPlaneNames());
			blockgroup_obj->addChild(footptObj);
		}
	}
	catch (const std::exception&e) {
		STOCKER_ERROR_ASSERT(e.what());
		return false;
	}

	return true;
}

ccHObject* LoD2FromFootPrint_PPP(ccHObject* entity, int max_iter, bool cap_hole, double ptsnum_ratio, double data_ratio, double ints_thre, double cluster_hori, double cluster_verti)
{
	BDBaseHObject* baseObj = GetRootBDBase(entity); if (!baseObj) return false;
	ccHObject* buildingObj = nullptr;
	if (entity->isA(CC_TYPES::ST_FOOTPRINT)) {
		buildingObj = GetParentBuilding(entity);
	}
	else if (entity->isA(CC_TYPES::ST_BUILDING)) {
		buildingObj = entity;
	}
	else return nullptr;
	
	QString building_name = buildingObj->getName();
	BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
	StPrimGroup* prim_group_obj = baseObj->GetPrimitiveGroup(building_name);
	StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName());
	if (!prim_group_obj || !blockgroup_obj) { return false; }

	//! get footprints
	ccHObject::Container footprints;
	if (entity->isA(CC_TYPES::ST_FOOTPRINT)) {
		footprints.push_back(entity);
	}
	else footprints = blockgroup_obj->getValidFootPrints();

	if (footprints.empty()) {
		return nullptr;
	}

	ccHObject::Container vertical_primObjs;
	ccHObject::Container planar_primObjs = GetNonVerticalPlaneClouds(prim_group_obj, 15, &vertical_primObjs);

	std::vector<stocker::Seg2d> facade_projected;
	for (ccHObject* planeEnt : vertical_primObjs) {
		ccPlane* planeObj = GetPlaneFromPlaneOrCloud(planeEnt); if (!planeObj) continue;
		std::vector<CCVector3> profile = planeObj->getProfile();
		stocker::Contour2d profile_pts_proj;
		for (auto pt : profile) { profile_pts_proj.push_back(stocker::Vec2d(pt.x, pt.y)); }
		stocker::Seg2d seg;
		double seg_q = stocker::FitSegment2d(profile_pts_proj, seg);
		if (_finite(seg_q)) facade_projected.push_back(seg);
	}
	facade_projected = stocker::ClusterSegments(facade_projected, cluster_hori, cluster_verti);

	for (ccHObject* fpEntity : footprints) {
		StFootPrint* ftObj = ccHObjectCaster::ToStFootPrint(fpEntity);
		if (!ftObj) { continue; }
		std::cout << "LoD2 - " << ftObj->getName().toStdString() << std::endl;

		double ground_height = ftObj->getBottom();

		Polyline3d footprint_polygon = GetPolygonFromPolyline(ftObj);

		//! collect planes
		ccHObject::Container primObjs;
		QStringList plane_names = ftObj->getPlaneNames();
		std::vector<stocker::Contour3d> planes_points;
		std::vector<stocker::Polyline3d> planes_frames;
		ccHObject::Container planes_used_for_ppp;

		if (!plane_names.empty()) {
			for (size_t pi = 0; pi < prim_group_obj->getChildrenNumber(); pi++) {
				ccHObject* child_cloud = prim_group_obj->getChild(pi); if (!child_cloud) continue;
				
				if (!child_cloud->isEnabled()) continue;
				if (plane_names.indexOf(child_cloud->getName()) >= 0) {
					primObjs.push_back(child_cloud);
				}
			}

			for (ccHObject* primObj : primObjs) {
				Contour3d outline_points; {
					ccHObject::Container container_find;
					primObj->filterChildrenByName(container_find, false, BDDB_PLANEFRAME_PREFIX, true);
					if (!container_find.empty()) {
						auto outlines_points_all = GetOutlinesFromOutlineParent(container_find.back());
						if (!outlines_points_all.empty()) {
							outline_points = outlines_points_all.front().front();
						}
					}
				}
				if (outline_points.size() < 3) continue;
				
				planes_points.push_back(GetPointsFromCloud3d(GetPlaneCloud(primObj)));
				planes_frames.push_back(MakeLoopPolylinefromContour(outline_points));
				planes_used_for_ppp.push_back(primObj);
			}
		}
		else {
			// TODO:
			continue;

			primObjs = planar_primObjs;
			stocker::Polyline3d footprint_polygon_lowest = footprint_polygon;
			for (auto & pt : footprint_polygon_lowest) {
				pt.P0().Z() = ftObj->getLowest();
				pt.P1().Z() = ftObj->getLowest();
			}
			planes_points = GetPointsFromCloudInsidePolygonsXY(primObjs, footprint_polygon_lowest, ftObj->getHighest());
			//planes_frames
		}
		if (planes_points.empty()) {
			// TODO: LOD1
			std::vector<CCVector3> top_points;
			for (auto pt : ToContour(footprint_polygon, 0)) {
				top_points.push_back(CCVector3(pt.X(), pt.Y(), pt.Z()));
			}
			StBlock* block_entity = StBlock::Create(top_points, ground_height);
			int block_number = GetMaxNumberExcludeChildPrefix(ftObj, BDDB_BLOCK_PREFIX) + 1;
			block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(block_number++));
			ftObj->addChild(block_entity);
			continue;
		}
		
		std::vector<stocker::Contour3d> result_planes_frames;
		IndexVector result_planes_index;
		if (planes_frames.size() == 1) {
			result_planes_frames.push_back(ToContour(footprint_polygon, 0));
			result_planes_index.push_back(0);
		}
		else //! pack planes
		{
			stocker::PolygonPartition poly_partition;

			std::string output_dir;
			if (!buildingObj->getPath().isEmpty()) {
				output_dir = QFileInfo(buildingObj->getPath()).absoluteFilePath().toStdString() + "/footprints/" + ftObj->getName().toStdString();
			}
			else {
				output_dir = QFileInfo(QString::fromStdString(build_unit.file_path.ori_points)).absolutePath().toStdString() + "/footprints/" + ftObj->getName().toStdString();
			}
			poly_partition.setOutputDir(output_dir);

			poly_partition.m_ints_thre = ints_thre;
			poly_partition.m_max_iter = max_iter;
			poly_partition.m_run_cap_hole = cap_hole;
			poly_partition.m_data_pts_ratio = ptsnum_ratio;
			poly_partition.m_data_ratio = data_ratio;

			poly_partition.setPolygon(planes_frames, planes_points, true, true);

			std::vector<stocker::Polygon2d> planes_frames_2d;
			for (auto pl_frm : planes_frames) {
				planes_frames_2d.push_back(ToPolyline2d(pl_frm));
			}
			stocker::Polyline2d facade_valid = stocker::collectCloseSegmentsAroundPolygons(planes_frames_2d, facade_projected, 1);
			poly_partition.setFacades(facade_valid);

			if (!poly_partition.runBP()) {
				return false;
			}

			std::vector<std::pair<size_t, Outline3d>> results = poly_partition.getResultPolygon();
			for (auto & NO_poly : results) {
				if (NO_poly.second.empty()) continue;
				//! for now, no holes for footprint
				result_planes_index.push_back(NO_poly.first);
				result_planes_frames.push_back(ToContour(NO_poly.second.front(), 0));
			}
		}

		//! parse result to footprint
		for (size_t i = 0; i < result_planes_frames.size(); i++) {
			Contour3d roof_points = result_planes_frames[i];
			size_t roof_plane_index = result_planes_index[i];
			std::vector<CCVector3> top_points;
			for (auto & pt : roof_points) {
				top_points.push_back(CCVector3(vcgXYZ(pt)));
			}
			StBlock* block_entity = nullptr;
			try {
				block_entity = StBlock::Create(top_points, ground_height);
			}
			catch (const std::exception& e) {
				if (block_entity) {
					delete block_entity;
					block_entity = nullptr;
				}
				continue;
			}
			if (block_entity) {
				int block_number = GetMaxNumberExcludeChildPrefix(ftObj, BDDB_BLOCK_PREFIX) + 1;
				block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(block_number));
				block_entity->setMetaData("plane", planes_used_for_ppp[roof_plane_index]->getName());
				ftObj->addChild(block_entity);
			}
		}
	}

	return blockgroup_obj;
}

bool PackFootprints_PPRepair(ccHObject* buildingObj)
{
	try {
		BDBaseHObject* baseObj = GetRootBDBase(buildingObj); if (!baseObj) return false;
		QString building_name = buildingObj->getName();
		BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
		StPrimGroup* prim_group_obj = baseObj->GetPrimitiveGroup(building_name);
		StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName());
		if (!prim_group_obj || !blockgroup_obj) { return false; }

		//! get footprints
		ccHObject::Container footprints = blockgroup_obj->getValidFootPrints();
		std::vector<std::vector<Contour3d>> layers_planes_points;
		std::vector<stocker::Contour3d> footprints_points;
		for (ccHObject* ft_entity : footprints) {
			StFootPrint* ftObj = ccHObjectCaster::ToStFootPrint(ft_entity);
			QStringList plane_names = ftObj->getPlaneNames();
			std::vector<Contour3d> planes_points;
			for (auto & pl_name : plane_names) {
				ccPlane* pl_entity = prim_group_obj->getPlaneByName(pl_name); if (!pl_entity) continue;
				if (isVertical(pl_entity, 15)) continue;
				ccPointCloud* plane_cloud = GetPlaneCloud(pl_entity); if (!plane_cloud) continue;
				planes_points.push_back(GetPointsFromCloud3d(plane_cloud));
			}
			layers_planes_points.push_back(planes_points);

			Contour3d ft_pts;
			for (auto & pt : ftObj->getPoints(true)) {
				ft_pts.emplace_back(pt.x, pt.y, pt.z);
			}
			footprints_points.push_back(ft_pts);
		}
		std::vector<stocker::Contour3d> footprints_points_pp;
		
		if (!FootPrintsPlanarPartition(layers_planes_points, footprints_points, footprints_points_pp)) return false;

		if (footprints_points_pp.empty()) return false;

		for (size_t i = 0; i < footprints.size(); i++) {
			footprints[i]->setEnabled(false);
			footprints[i]->setName("del-" + footprints[i]->getName());
		}
		int biggest = GetMaxNumberExcludeChildPrefix(blockgroup_obj, BDDB_FOOTPRINT_PREFIX);
		for (size_t i = 0; i < footprints_points_pp.size(); i++) {
			QString name = BDDB_FOOTPRINT_PREFIX + QString::number(++biggest);
			StFootPrint* footptObj = AddPolygonAsFootprint(footprints_points_pp[i], name, ccColor::magenta, true);

			footptObj->setComponentId(0);
			footptObj->setHighest(build_unit.ground_height);
			footptObj->setBottom(build_unit.ground_height);
			footptObj->setLowest(build_unit.ground_height);
			blockgroup_obj->addChild(footptObj);
		}

// 		if (footprints_points.size() == footprints_points_pp.size()) {
// 			footprints_points = footprints_points_pp;
// 		}
// 
// 		for (auto & polygon : footprints_points) {
// 			RepairPolygon(polygon, CC_DEG_TO_RAD * 5);
// 		}
// 
// 		if (footprints_points.size() == footprints_points_pp.size()) {
// 			for (size_t i = 0; i < footprints.size(); i++) {
// 				StFootPrint* ftObj = ccHObjectCaster::ToStFootPrint(footprints[i]);
// 				SubstituteFootPrintContour(ftObj, footprints_points[i]);
// 				ftObj->prepareDisplayForRefresh();
// 			}
// 		}
	}
	catch (const std::exception&e) {
		STOCKER_ERROR_ASSERT(e.what());
		return false;
	}
	catch (...) {
		STOCKER_ERROR_ASSERT("unknown");
		return false;
	}

	return true;
}

void GetPlanesInsideFootPrint(ccHObject* footprint, ccHObject* prim_group, CCVector3 settings, bool bVertical, bool clearExisting) 
{
	StFootPrint* footprintObj = ccHObjectCaster::ToStFootPrint(footprint);
	StPrimGroup* primgroupObj = ccHObjectCaster::ToStPrimGroup(prim_group);

	QStringList plane_names;
	if (!clearExisting)	{
		plane_names = footprintObj->getPlaneNames();
	}

	std::vector<CCVector3> ftpts = footprintObj->getPoints(false);
	Contour3d ftpts_stocker; for (auto & pt : ftpts) { ftpts_stocker.emplace_back(pt.x, pt.y, pt.z); }
	Polyline2d footprint_polygon = MakeLoopPolylinefromContour(ToContour2d(ftpts_stocker));
	double min_z = footprintObj->getLowest() - settings.y;
	double max_z = footprintObj->getHighest() + settings.y;

	ccHObject::Container all_planes = primgroupObj->getValidPlanes();
	for (ccHObject* plane : all_planes) {
		ccPlane* planeObj = ccHObjectCaster::ToPlane(plane);

		bool is_plane_vertical = planeObj->isVerticalToDirection(CCVector3(0, 0, 1), 15);
		if (!bVertical && is_plane_vertical) { continue; }
		ccPointCloud* plane_cloud = GetPlaneCloud(plane);
		assert(plane_cloud); if (!plane_cloud) continue;
		if (!clearExisting && plane_names.indexOf(plane_cloud->getName()) >= 0) { continue; }
				
		Polyline2d plane_polygon = MakeLoopPolylinefromContour(ccToPoints2<CCVector3, Vec2d>(planeObj->getProfile(), true));
		if (plane_polygon.empty()) continue;

		Contour3d planes_points = GetPointsFromCloud3d(plane_cloud);
		if (planes_points.size() < settings.z) { continue; }
		
		//! vertical plane, center point to the footprint distance
		if (is_plane_vertical) {
// 			CCVector3 center_pt = CalcMean(planeObj->getProfile());
// 			if (!vcg::PointInsidePolygon(Vec2d(center_pt.x, center_pt.y), footprint_polygon)) continue;
			
			Concurrency::concurrent_vector<short> count;
			bool min_count_achieved = false;
			Concurrency::parallel_for((size_t)0, planes_points.size(), [&](size_t pi) {
				if (vcg::PointInsidePolygon(planes_points[pi].ToVec2(), footprint_polygon)
					/*&& planes_points[pi].Z() > min_z && planes_points[pi].Z() < max_z*/) {
					count.push_back(0);
					if (count.size() >= (size_t)settings.z) {
						min_count_achieved = true;
						return;
					}
				}
			});
 			if (min_count_achieved) { plane_names.push_back(plane_cloud->getName()); }
		}
		//! non-vertical plane, 
		else {
			if (DistancePolygonPolygon(footprint_polygon, plane_polygon) > settings.x) continue;
		
			Concurrency::concurrent_vector<short> count;
			bool min_count_achieved = false;
			Concurrency::parallel_for((size_t)0, planes_points.size(), [&](size_t pi) {
				if (vcg::PointInsidePolygon(planes_points[pi].ToVec2(), footprint_polygon)
					&& planes_points[pi].Z() > min_z && planes_points[pi].Z() < max_z) {
					count.push_back(0);
					if (count.size() >= (size_t)settings.z) {
						min_count_achieved = true;
						return;
					}
				}
			});
			if (min_count_achieved) { plane_names.push_back(plane_cloud->getName()); }
		}
	}

	footprintObj->setPlaneNames(plane_names);
	footprintObj->prepareDisplayForRefresh();
}
#include "vcg/complex/algorithms/crease_cut.h"
#include "vcg/math/base.h"
#include "vcg/complex/algorithms/update/topology.h"
ccHObject::Container LoadMeshAsBlock(QString filename)
{
	ccHObject::Container blocks;
	typedef vcg::tri::io::ImporterOBJ<PolyMesh> PMeshIObj;
	GLMesh poly_mesh;	int loadmask;
	GLMeshIObj::LoadMask(filename.toStdString().c_str(), loadmask);
	if (GLMeshIObj::Open(poly_mesh, filename.toStdString().c_str(), loadmask) != vcg::ply::E_NOERROR) {
		return blocks;
	}
	vcg::tri::Clean<GLMesh>::MergeCloseVertex(poly_mesh, 0.00001);
	vcg::tri::UpdateTopology<GLMesh>::FaceFace(poly_mesh);
	vcg::tri::CreaseCut(poly_mesh, vcg::math::ToRad(20.f));
	GLMesh out_mesh;
	vcg::tri::BuildFromFaceEdgeSel(poly_mesh, out_mesh);

	FILE * fp = fopen("D:/2.obj", "w");

	int vert_count(1);
	for (size_t i = 0; i < out_mesh.edge.size(); i++) {
		vcg::Point3d pt = out_mesh.edge[i].P(0);
		fprintf(fp, "v %lf %lf %lf\n", pt.X(), pt.Y(), pt.Z());
		pt = out_mesh.edge[i].P(1);
		fprintf(fp, "v %lf %lf %lf\n", pt.X(), pt.Y(), pt.Z());

		fprintf(fp, "l %d %d\n", vert_count++, vert_count++);
	}
	fclose(fp);
	return blocks;
}