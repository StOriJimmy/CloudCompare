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

#include "stocker_parser.h"

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
#include "polyfit/model/map_enumerator.h"
#include "polyfit/model/map_serializer.h"
using namespace stocker;
#endif // USE_STOCKER

QString GetBaseName(QString name) {	
	return name.mid(0, name.indexOf('.'));
}

ccHObject* FitPlaneAndAddChild(ccPointCloud* cloud)
{
	ccHObject* cc_plane = nullptr;
	double rms = 0;
	ccPlane* pPlane = ccPlane::Fit(cloud, &rms);
	if (pPlane) {
		cc_plane = static_cast<ccHObject*>(pPlane);
		if (cloud->hasColors())	{
			pPlane->setColor(cloud->getPointColor(0));
		}		
		pPlane->enableStippling(true);
	}
	if (cc_plane) {
		cc_plane->setName("Plane");
		cc_plane->applyGLTransformation_recursive();
		cc_plane->showColors(true);
		cc_plane->setVisible(true);
		cc_plane->showNormals(cloud->hasNormals());

		cloud->addChild(cc_plane);
		cc_plane->setDisplay(cloud->getDisplay());
		cc_plane->prepareDisplayForRefresh_recursive();
	}
	return cc_plane;
}

stocker::Contour3d GetPointsFromCloud(ccHObject* entity) {
	stocker::Contour3d points;
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!cloud) return points;

	for (unsigned i = 0; i < cloud->size(); i++) {
		CCVector3 pt = *cloud->getPoint(i);
		points.push_back({ pt.x, pt.y, pt.z });
	}
	return points;
}

stocker::Contour3d GetPointsFromCloudInsidePolygonXY(ccHObject* entity, stocker::Polyline3d polygon, double height)
{	
	stocker::Contour3d points;
	if (entity->isA(CC_TYPES::ST_PRIMITIVE)) {
		ccHObject::Container planes_container, plane_cloud_container;
		entity->filterChildren(planes_container, true, CC_TYPES::PLANE, true);
		for (ccHObject* pl_obj : planes_container) {
			if (pl_obj->getParent() && pl_obj->getParent()->isEnabled()) {
				plane_cloud_container.push_back(pl_obj->getParent());
			}			
		}
		std::vector<Contour3d> planes_points;
		planes_points = GetPointsFromCloudInsidePolygonXY(plane_cloud_container, polygon, height);
		for (auto & pl_pts : planes_points) {
			points.insert(points.end(), pl_pts.begin(), pl_pts.end());
		}
		return points;
	}
	else if (!entity->isA(CC_TYPES::POINT_CLOUD)) return points;

	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!cloud) return points;

	std::vector<vcg::Segment2d> polygon_2d;
	bool use_height = false;
	for (auto & seg : polygon) {
		polygon_2d.push_back(vcg::Segment2d(ToVec2d(seg.P0()), ToVec2d(seg.P1())));
		if (height > seg.P0().Z() || height > seg.P1().Z()) {
			use_height = true;
		}
	}
	Concurrency::concurrent_vector<Vec3d> points_parallel;
	if (use_height) {
		Concurrency::parallel_for((size_t)0, (size_t)cloud->size(), [&](size_t i) {
			CCVector3 pt = *cloud->getPoint(i);
			if (vcg::PointInsidePolygon({ pt.x,pt.y }, polygon_2d) && pt.z < height) {
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

std::vector<stocker::Contour3d> GetPointsFromCloudInsidePolygonXY(ccHObject::Container entities, stocker::Polyline3d polygon, double height, bool skip_empty)
{
	std::vector<stocker::Contour3d> all_points;
	for (ccHObject* entity : entities) {
		stocker::Contour3d points = GetPointsFromCloudInsidePolygonXY(entity, polygon, height);
		if (points.size() > 0) {
			all_points.push_back(points);
		}		
	}
	return all_points;
}

stocker::Contour3d GetPointsFromCloudInsidePolygon(ccHObject* entity, stocker::Polyline3d polygon, double distance_threshold)
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
	Concurrency::concurrent_vector<Vec3d> points_parallel;
	Concurrency::parallel_for((size_t)0, (size_t)cloud->size(), [&](size_t i) {
		CCVector3 pt = *cloud->getPoint(i);
		Vec3d pt_3d(pt.x, pt.y, pt.z);
		Vec2d pt_2d = plane_unit.ToPlpoint2d(pt_3d);
		if (vcg::PointInsidePolygon(pt_2d, polygon_2d) && vcg::SignedDistancePointPlane(pt_3d, plane_unit.plane) < distance_threshold) {
			points_parallel.push_back({ pt.x, pt.y, pt.z });
		}
	});
	points.assign(points_parallel.begin(), points_parallel.end());
	return points;
}

stocker::Polyline3d GetPolygonFromPolyline(ccHObject* entity)
{
	stocker::Polyline3d polyline;
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

ccHObject::Container GetEnabledObjFromGroup(ccHObject* entity, CC_CLASS_ENUM type, bool check_enable, bool recursive)
{
	if (entity) {
		ccHObject::Container group;
		entity->filterChildren(group, recursive, type, true, entity->getDisplay());
		if (check_enable) {
			ccHObject::Container group_enabled;
			for (auto & gp : group) {
				if ((gp->getParent()) && (!gp->getParent()->isEnabled())) {
					continue;
				}
				if (gp->isEnabled())
					group_enabled.push_back(gp);
			}
			return group_enabled;
		}
		else {
			return group;
		}		
	}
	return ccHObject::Container();
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

ccHObject::Container BDBaseHObject::GetHObjContainer(CC_CLASS_ENUM type, QString suffix, bool check_enable)
{
	ccHObject::Container entities = GetEnabledObjFromGroup(this, type, check_enable);
	ccHObject::Container output;
	for (auto & entity : entities) {
		if (entity->getName().endsWith(suffix)) {
			output.push_back(entity);
		}
	}
	return output;
}
ccHObject * BDBaseHObject::GetHObj(CC_CLASS_ENUM type, QString suffix, QString basename, bool check_enable)
{
	ccHObject::Container entities = GetEnabledObjFromGroup(this, type, check_enable);
	ccHObject* output = nullptr;
	for (auto & entity : entities) {
		if (entity->getName().endsWith(suffix) &&
			GetBaseName(entity->getName()) == basename) {
			return entity;
		}
	}
	return nullptr;
}
StBuilding* BDBaseHObject::GetBuildingGroup(QString building_name, bool check_enable) {
	ccHObject* obj = GetHObj(CC_TYPES::ST_BUILDING, "", building_name, check_enable);
	if (obj) return static_cast<StBuilding*>(obj);
	return nullptr;
}
ccPointCloud * BDBaseHObject::GetOriginPointCloud(QString building_name, bool check_enable) {
	ccHObject* obj = GetHObj(CC_TYPES::POINT_CLOUD, BDDB_ORIGIN_CLOUD_SUFFIX, building_name, check_enable);
	if(obj) return static_cast<ccPointCloud*>(obj);
	return nullptr;
}
StPrimGroup * BDBaseHObject::GetPrimitiveGroup(QString building_name, bool check_enable) {
	ccHObject* obj = GetHObj(CC_TYPES::ST_PRIMITIVE, BDDB_PRIMITIVE_SUFFIX, building_name, check_enable);
	if (obj) return static_cast<StPrimGroup*>(obj);
	StPrimGroup* group = new StPrimGroup(building_name + BDDB_PRIMITIVE_SUFFIX);
	if (group) {
		ccHObject* bd = GetBuildingGroup(building_name, false);
		if (bd) { bd->addChild(group); MainWindow::TheInstance()->addToDB(group); return group; }
	}
	return nullptr;
}
StBlockGroup * BDBaseHObject::GetBlockGroup(QString building_name, bool check_enable) {
	ccHObject* obj = GetHObj(CC_TYPES::ST_BLOCKGROUP, BDDB_BLOCKGROUP_SUFFIX, building_name, check_enable);
	if (obj) return static_cast<StBlockGroup*>(obj);
	StBlockGroup* group = new StBlockGroup(building_name + BDDB_BLOCKGROUP_SUFFIX);
	if (group) { 
		ccHObject* bd = GetBuildingGroup(building_name, false);
		if (bd) { bd->addChild(group); MainWindow::TheInstance()->addToDB(group); return group; }
	}
	return nullptr;
}
ccHObject * BDBaseHObject::GetHypothesisGroup(QString building_name, bool check_enable) {
	return GetHObj(CC_TYPES::HIERARCHY_OBJECT, BDDB_POLYFITHYPO_SUFFIX, building_name, check_enable);
}
std::string BDBaseHObject::GetPathModelObj(std::string building_name)
{
	auto& bd = block_prj.m_builder.sbuild.find(stocker::BuilderBase::BuildNode::Create(building_name));
	if (bd == block_prj.m_builder.sbuild.end())	{
		throw std::runtime_error("cannot find building!" + building_name);
	}
	return std::string((*bd)->data.file_path.model_dir + building_name + MODEL_LOD3_OBJ_SUFFIX);
}

stocker::BuildUnit BDBaseHObject::GetBuildingUnit(std::string building_name) {
	auto iter = block_prj.m_builder.sbuild.find(BuilderBase::BuildNode::Create(building_name));
	if (iter == block_prj.m_builder.sbuild.end()) {
		return stocker::BuildUnit("invalid");
	}
	else return (*iter)->data;
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

bool IsBDBaseObj(ccHObject* obj) {
	return obj->isA(CC_TYPES::ST_PROJECT);
}
BDBaseHObject* GetRootBDBase(ccHObject* obj) {
	ccHObject* bd_obj_ = obj;
	do {
		if (IsBDBaseObj(bd_obj_)) {
			return static_cast<BDBaseHObject*>(bd_obj_);
		}
		bd_obj_ = bd_obj_->getParent();
	} while (bd_obj_);

	return nullptr;
}

bool SetGlobalShiftAndScale(ccHObject* obj)
{
	BDBaseHObject* baseObj = GetRootBDBase(obj);
	if (!baseObj) {
		return false;
	}
	ccHObject::Container cloud_container;
	if (obj->isA(CC_TYPES::POINT_CLOUD)) {
		cloud_container.push_back(obj);
	}
	else {
		obj->filterChildren(cloud_container, true, CC_TYPES::POINT_CLOUD, false);
	}
	for (auto & _cld : cloud_container)	{
		ccPointCloud* cloud_entity = ccHObjectCaster::ToPointCloud(_cld);
		cloud_entity->setGlobalScale(baseObj->global_scale);
		cloud_entity->setGlobalShift(CCVector3d(vcgXYZ(baseObj->global_shift)));
	}
	return true;
}

int GetNumberExcludePrefix(ccHObject * obj, QString prefix)
{
	QString name = obj->getName();
	if (name.startsWith(prefix) && name.length() > prefix.length()) {
		QString number = name.mid(prefix.length(), name.length());
		return number.toInt();
	}
	return -1;
}

int GetMaxNumberExcludeChildPrefix(ccHObject * obj, QString prefix/*, CC_CLASS_ENUM type = CC_TYPES::OBJECT*/)
{
	set<int> name_numbers;
	for (size_t i = 0; i < obj->getChildrenNumber(); i++) {
		QString name = obj->getChild(i)->getName();
		int number = GetNumberExcludePrefix(obj->getChild(i), prefix);
		if (number >= 0) {
			name_numbers.insert(number);
		}
	}
	if (!name_numbers.empty()) {
		return *name_numbers.rbegin();
	}
	return -1;
}

ccHObject* AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col)
{
	if (lines.empty()) {
		return nullptr;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);

	ccPointCloud* line_vert = new ccPointCloud(name);
	int i(0);
	for (auto & ln : lines) {
		ccPolyline* cc_polyline = new ccPolyline(line_vert);
		cc_polyline->setDisplay(entity->getDisplay());
		cc_polyline->setColor(col);
		cc_polyline->showColors(true);
		cc_polyline->setName(name + QString::number(i));
		if (cloud) {
			cc_polyline->setGlobalShift(cloud->getGlobalShift());
			cc_polyline->setGlobalScale(cloud->getGlobalScale());
		}
		cc_polyline->reserve(2);

		line_vert->addPoint(CCVector3(vcgXYZ(ln.P0())));
		cc_polyline->addPointIndex(line_vert->size() - 1);

		line_vert->addPoint(CCVector3(vcgXYZ(ln.P1())));
		cc_polyline->addPointIndex(line_vert->size() - 1);

		cc_polyline->setClosed(false);
		line_vert->addChild(cc_polyline);
		i++;
	}

	entity->addChild(line_vert);
	return line_vert;
}

StPrimGroup* AddPlanesPointsAsNewGroup(QString name, std::vector<stocker::Contour3d> planes_points)
{
	StPrimGroup* group = new StPrimGroup(name);

	for (size_t i = 0; i < planes_points.size(); i++) {
		ccPointCloud* plane_cloud = new ccPointCloud(BDDB_PLANESEG_PREFIX + QString::number(i));// TODO

		//! get plane points
		for (auto & pt : planes_points[i]) {
			plane_cloud->addPoint(CCVector3(vcgXYZ(pt)));
		}
		ccColor::Rgb col = ccColor::Generator::Random();
		plane_cloud->setRGBColor(col);
		plane_cloud->showColors(true);

		//! add plane
		FitPlaneAndAddChild(plane_cloud);

		group->addChild(plane_cloud);
	}
	return group;
}

ccHObject* PlaneSegmentationRgGrow(ccHObject* entity,
	int min_pts, double distance_epsilon, double seed_raius,
	double growing_radius,
	double merge_threshold, double split_threshold)
{
	ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!entity_cloud->hasNormals()) {
		return nullptr;
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

	StPrimGroup* group = AddPlanesPointsAsNewGroup(GetBaseName(entity->getName()) + BDDB_PRIMITIVE_SUFFIX, planes_points);
	group->setDisplay_recursive(entity->getDisplay());
	ccHObject::Container group_clouds;
	group->filterChildren(group_clouds, false, CC_TYPES::POINT_CLOUD, true);
	for (auto & ent : group_clouds) {
		ccPointCloud* ent_cld = ccHObjectCaster::ToPointCloud(ent);
		ent_cld->setGlobalShift(entity_cloud->getGlobalShift());
		ent_cld->setGlobalScale(entity_cloud->getGlobalScale());
	}
	entity->getParent()->addChild(group);
	return group;
}

ccHObject* PlaneSegmentationRansac(ccHObject* entity,
	int min_pts, double distance_epsilon, double seed_raius,
	double normal_threshold, double ransac_probability,
	double merge_threshold, double split_threshold)
{
	ccPointCloud* entity_cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!entity_cloud->hasNormals()) {
		return nullptr;
	}

	stocker::GLMesh mesh;
	for (unsigned i = 0; i < entity_cloud->size(); i++) {
		CCVector3 pt = *entity_cloud->getPoint(i);
		CCVector3 normal = entity_cloud->getPointNormal(i);
		stocker::GLMeshAL::AddVertex(mesh, parse_xyz(pt), parse_xyz(normal));
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

	StPrimGroup* group = AddPlanesPointsAsNewGroup(GetBaseName(entity->getName()) + BDDB_PRIMITIVE_SUFFIX, planes_points);
	group->setDisplay_recursive(entity->getDisplay());
	ccHObject::Container group_clouds;
	group->filterChildren(group_clouds, false, CC_TYPES::POINT_CLOUD, true);
	for (auto & ent : group_clouds) {
		ccPointCloud* ent_cld = ccHObjectCaster::ToPointCloud(ent);
		ent_cld->setGlobalShift(entity_cloud->getGlobalShift());
		ent_cld->setGlobalScale(entity_cloud->getGlobalScale());
	}
	entity->getParent()->addChild(group);
	return group;	
}

ccHObject::Container CalcPlaneIntersections(ccHObject::Container entity_planes, double distance)
{
#ifdef USE_STOCKER
	stocker::PlaneData plane_units;
	for (size_t i = 0; i < entity_planes.size(); i++) {
		if (!entity_planes[i]->isEnabled()) continue;

		stocker::Contour3d cur_plane_points = GetPointsFromCloud(entity_planes[i]->getParent());
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
		ccHObject* add = AddSegmentsAsChildVertices(entity_planes[num]->getParent(), ints_per_plane[i], "Intersection", ccColor::red);
		if (add) segs_add.push_back(add);
	}
	return segs_add;
#endif // USE_STOCKER
	return ccHObject::Container();
}

ccHObject* CalcPlaneBoundary(ccHObject* planeObj)
{
#ifdef USE_STOCKER
	/// get boundary points
	Contour2d boundary_points_2d;
	Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
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

	/// get boundary lines
	Polyline3d detected_lines;
	stocker::LineFromPlanePoints(cur_plane_points, detected_lines);

 	Contour3d boundary_points_3d = Plpoint2dToPoint3d(plane_unit, boundary_points_2d);
// 	IndexGroup line_index_group;
// 	LineRansacfromPoints(boundary_points_3d, detected_lines, line_index_group, p2l_distance, boundary_minpts);

	ccHObject* line_vert = AddSegmentsAsChildVertices(planeObj->getParent(), detected_lines, BDDB_BOUNDARY_PREFIX, ccColor::yellow);

	if (!line_vert) {
		return nullptr;
	}
	ccPointCloud* line_cloud = ccHObjectCaster::ToPointCloud(line_vert);
	for (auto & pt : boundary_points_3d) {
		line_cloud->addPoint(CCVector3(vcgXYZ(pt)));
	}
	line_cloud->setRGBColor(ccColor::yellow);
	line_cloud->showColors(true);

	return line_vert;
#endif // USE_STOCKER
	return nullptr;
}

ccHObject* DetectLineRansac(ccHObject* entity, double distance, double minpts, double radius)
{
#ifdef USE_STOCKER
	Contour3d cur_plane_points;
	if (entity->isA(CC_TYPES::POLY_LINE)) {
		ccPolyline* poly_line = ccHObjectCaster::ToPolyline(entity);
		if (!poly_line) return nullptr;
		vector<CCVector3> points = poly_line->getPoints(false);
		for (auto & pt : points) {
			cur_plane_points.push_back(parse_xyz(pt));
		}
	}
	else if (entity->isA(CC_TYPES::POINT_CLOUD)) {
		cur_plane_points = GetPointsFromCloud(entity);
	}
	Polyline3d bdry_lines_2d; IndexGroup indices;
	LineRansacfromPoints(cur_plane_points, bdry_lines_2d, indices, distance, minpts, radius);

	ccHObject* line_vert = AddSegmentsAsChildVertices(entity, bdry_lines_2d, "RansacLine", ccColor::red);
	return line_vert;
#endif // USE_STOCKER
	return nullptr;	
}

ccHObject* AddOutlinesAsChild(vector<vector<stocker::Contour3d>> contours_points, QString name, ccHObject* parent)
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
	parent->addChild(line_vert);
	return line_vert;
}

ccHObject* CalcPlaneOutlines(ccHObject* planeObj, double alpha)
{
#ifdef USE_STOCKER
	stocker::Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
	if (cur_plane_points.size() < 3) {
		return nullptr;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(planeObj->getParent());

	//! get boundary
	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, alpha, false, 2);
	return AddOutlinesAsChild(contours_points, BDDB_OUTLINE_PREFIX, planeObj->getParent());
//	if (contours_points.empty()) return nullptr;
// 	ccPointCloud* line_vert = new ccPointCloud(BDDB_OUTLINE_PREFIX);
// 	int component_number = 0;
// 	for (vector<stocker::Contour3d> & component : contours_points) {
// 		for (stocker::Contour3d & st_contours : component) {
// 			ccPolyline* cc_polyline = new ccPolyline(line_vert);
// 			cc_polyline->setDisplay(planeObj->getDisplay());
// 			cc_polyline->setColor(ccColor::green);
// 			cc_polyline->showColors(true);
// 			line_vert->addChild(cc_polyline);
// 			cc_polyline->setName(BDDB_OUTLINE_PREFIX + QString::number(component_number));
// 			cc_polyline->setWidth(2);
// 			cc_polyline->setGlobalShift(cloud->getGlobalShift());
// 			cc_polyline->setGlobalScale(cloud->getGlobalScale());
// 			cc_polyline->reserve(static_cast<unsigned>(st_contours.size() + 1));
// 			for (auto & pt : st_contours) {
// 				line_vert->addPoint(CCVector3(pt.X(), pt.Y(), pt.Z()));
// 				cc_polyline->addPointIndex(line_vert->size() - 1);
// 			}
// 			cc_polyline->setClosed(true);
// 		}
// 		component_number++;
// 	}
// 	planeObj->getParent()->addChild(line_vert);
// 	return line_vert;
#endif // USE_STOCKER
}

#include "vcg/space/intersection2.h"
void ShrinkPlaneToOutline(ccHObject * planeObj, double alpha, double distance_epsilon, MainWindow* win)
{
#ifdef USE_STOCKER
	ccHObject* parent_cloud = planeObj->getParent();
	if (!parent_cloud) {
		std::cout << "failed to shrink plane" << planeObj->getName().toStdString() << std::endl;
		return;
	}
	stocker::Contour3d cur_plane_points = GetPointsFromCloud(parent_cloud);
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
	Polyline2d concave_polygon = MakeLoopPolylinefromContour2d(concave_2d);
		
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
	cloud->setName(cloud->getName() + "-delete");
	parent_cloud->setEnabled(false);
	ccHObject* parent = parent_cloud->getParent();
	parent->addChild(newCloud);
	
	FitPlaneAndAddChild(newCloud);	

	int index_old = parent->getChildIndex(cloud);
	int index_new = parent->getChildIndex(newCloud);
	parent->swapChildren(index_old, index_new);
	win->addToDB(newCloud);

	win->removeFromDB(cloud);

	vector<vector<stocker::Contour3d>> contours_points_remained = stocker::GetPlanePointsOutline(cur_plane_points_remained, alpha, false, 2);
	do {
		contours_points_remained.pop_back();
	} while (contours_points_remained.size() > 1);
	ccHObject* outlines_add = AddOutlinesAsChild(contours_points_remained, BDDB_OUTLINE_PREFIX, newCloud);
	win->addToDB(outlines_add);
//	win->db()->removeElement(cloud);
	
#endif // USE_STOCKER
}

ccHObject* PlaneFrameOptimization(ccHObject* planeObj, stocker::FrameOption option)
{
#ifdef USE_STOCKER
	std::string base_name = GetBaseName(planeObj->getParent()->getParent()->getName()).toStdString();

	BDBaseHObject* baseObj = GetRootBDBase(planeObj);
	std::string output_prefix;
	if (baseObj) {		
		auto bd_find = baseObj->block_prj.m_builder.sbuild.find(BuilderBase::BuildNode::Create(base_name));
		if (bd_find == baseObj->block_prj.m_builder.sbuild.end()) {
			return nullptr;
		}
		output_prefix = (*bd_find)->data.file_path.root_dir + "\\primitives\\frame_opt\\";
		CreateDir(output_prefix.c_str());
		output_prefix = output_prefix + base_name;
	}
	

	//////////////////////////////////////////////////////////////////////////
	// frame optimization

	std::string plane_unit_name = base_name + "-" + planeObj->getParent()->getName().toStdString();
	stocker::FrameOptmzt frame_opt(planeObj->getParent()->getName().toStdString());
	
	frame_opt.SetOption(option);

	vcg::Plane3d vcgPlane = GetVcgPlane(planeObj);

	// prepare plane points
	Contour3d plane_points = GetPointsFromCloud(planeObj->getParent());

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
		frame_opt.PrepareImageList(GetRootBDBase(planeObj)->block_prj.m_builder);
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

	ccHObject* plane_frame = AddOutlinesAsChild(frames_to_add, BDDB_PLANEFRAME_PREFIX, planeObj->getParent());
	return plane_frame;
#endif
}

PointSet* GetPointSetFromPlaneObjs(ccHObject::Container planeObjs)
{
	PointSet* pset = new PointSet;
	std::vector<vec3>& points = pset->points();
	std::vector<vec3>& normals = pset->normals();
	unsigned int pt_idx(0);
	for (auto & planeObj : planeObjs) {
		ccPointCloud* cloud_entity = ccHObjectCaster::ToPointCloud(planeObj->getParent());
		if (!cloud_entity) continue;

		//////////////////////////////////////////////////////////////////////////
		VertexGroup* plane_grp = new VertexGroup;
		vcg::Plane3d vcg_pl = GetVcgPlane(planeObj);
		plane_grp->set_plane(Plane3d(
			vcg_pl.Direction().X(),
			vcg_pl.Direction().Y(),
			vcg_pl.Direction().Z(),
			-vcg_pl.Offset()));

		plane_grp->set_label(cloud_entity->getName().toStdString());

		for (size_t i = 0; i < cloud_entity->size(); i++) {
			CCVector3 pt_get = *(cloud_entity->getPoint(i));
			points.push_back(vec3(pt_get.x, pt_get.y, pt_get.z));			
			if (!cloud_entity->hasNormals()) {
				normals.push_back(vec3(vcg_pl.Direction().X(), vcg_pl.Direction().Y(), vcg_pl.Direction().Z()));
			}
			else {
				pt_get = cloud_entity->getPointNormal(i);
				normals.push_back(vec3(pt_get.x, pt_get.y, pt_get.z));
			}
			plane_grp->push_back(pt_idx++);
		}

		if (!plane_grp->empty()) {
			plane_grp->set_point_set(pset);
			pset->groups().push_back(plane_grp);
		}
	}
	return pset;
}

ccHObject * PolyfitGenerateHypothesis(ccHObject * primitive_group, PolyFitObj * polyfit_obj)
{
	if (!polyfit_obj) {
		polyfit_obj = new PolyFitObj();
	}
	ccHObject::Container planeObjs = GetEnabledObjFromGroup(primitive_group, CC_TYPES::PLANE, true, true);
	
	polyfit_obj->initGenerator(planeObjs);
	
	polyfit_obj->GenerateHypothesis();

	if (!polyfit_obj->hypothesis_mesh_) {
		throw std::runtime_error("cannot generate hypothesis mesh");
	}
	ccHObject* hypoObj = new ccHObject(GetBaseName(primitive_group->getName()) + BDDB_POLYFITHYPO_SUFFIX);

//	bd00000000.hypothesis
//	-Plane0						point cloud
//	 --Plane					Plane
//	  ---vertices				(Plane accessory)
//	  ---compressed normals		(Plane accessory)
//	  ---Facet0					Facet
//	   ----Contour points		(Facet accessory)
	
	BDBaseHObject* baseObj = GetRootBDBase(primitive_group);
	CCVector3d global_shift(0, 0, 0);
	double global_scale(0);
	if (baseObj) {
		global_shift = CCVector3d(vcgXYZ(baseObj->global_shift));
		global_scale = baseObj->global_scale;
	}
	PointSet* pset = polyfit_obj->hypothesis_->point_set();
	std::vector<vec3>& points = pset->points();
	std::vector<vec3>& normals = pset->normals();

 	ConcVector(ccPointCloud*) conc_plane_cloud;
	
	ConcParForBegin(polyfit_obj->hypothesis_->point_set()->groups().size())
	{
		//! associate point cloud for this plane
		VertexGroup* grp = pset->groups()[conc_index];
		ccPointCloud* plane_cloud = new ccPointCloud(grp->label().c_str());
		for (unsigned int pt_index : *grp) {
			vec3 pt_vert = points[pt_index];
			plane_cloud->addPoint(CCVector3(pt_vert.data()[0], pt_vert.data()[1], pt_vert.data()[2]));
		}
		if (plane_cloud->reserveTheNormsTable()) {
			for (unsigned int pt_index : *grp) {
				vec3 pt_vert = normals[pt_index];
				plane_cloud->addNorm(CCVector3(pt_vert.data()[0], pt_vert.data()[1], pt_vert.data()[2]));
			}
		}

		ccColor::Rgb col = ccColor::Generator::Random();
		plane_cloud->setRGBColor(col);
		plane_cloud->showColors(true);
		plane_cloud->setGlobalShift(global_shift);
		plane_cloud->setGlobalScale(global_scale);

		//! add plane as child of the point cloud
		ccHObject* plane_entity = FitPlaneAndAddChild(plane_cloud);
		plane_entity->setVisible(false);
		conc_plane_cloud.push_back(ConcPairObj(plane_cloud));
	}
	ConcParForEnd 

 	ConcSort(ccPointCloud*, conc_plane_cloud);
 	for (auto & obj : conc_plane_cloud) {
 		hypoObj->addChild(GetConcObj(obj));
 	}
 	conc_plane_cloud.clear(); conc_plane_cloud.shrink_to_fit();

	MapFacetAttribute<VertexGroup*> facet_attrib_supporting_vertex_group_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_supporting_vertex_group());
	
	FOR_EACH_FACET(Map, polyfit_obj->hypothesis_mesh_, it) {
		Map::Facet* f = it;

		//! add to the plane it belongs
		/// get plane by name
		std::string support_plane_name = facet_attrib_supporting_vertex_group_[f]->label();
		ccHObject* plane_entity = GetPlaneEntityFromPrimGroup(hypoObj, support_plane_name.c_str());
		if (!plane_entity) throw std::runtime_error("cannot find plane");

		Polygon3d contour_polygon = Geom::facet_polygon(f);
		vector<CCVector3> ccv_poly;
		for (auto & pt : contour_polygon) {
			ccv_poly.push_back(CCVector3(pt.data()[0], pt.data()[1], pt.data()[2]));
		}
		//! each facet is a facet entity under plane
		PointCoordinateType plane_equation[4];
		ccPlane* cc_plane = ccHObjectCaster::ToPlane(plane_entity);
		CCVector3 N; PointCoordinateType dis; cc_plane->getEquation(N, dis);
		plane_equation[0] = N.x; plane_equation[1] = N.y; plane_equation[2] = N.z; plane_equation[3] = dis;
		ccFacet* facet_entity = ccFacet::CreateFromContour(ccv_poly, f->label().c_str(), plane_equation);
		
		ccPolyline* contour_entity = facet_entity->getContour();
		if (contour_entity) {
			contour_entity->setGlobalShift(global_shift);
			contour_entity->setGlobalScale(global_scale);
		}
		else {
			std::string error_info = "error contour: plane-" + support_plane_name + " facet-" + f->label();
			ccLog::Warning(error_info.c_str());
		}
		plane_entity->addChild(facet_entity);
	}
	hypoObj->setDisplay_recursive(primitive_group->getDisplay());
	if (primitive_group->getParent()) {
		primitive_group->getParent()->addChild(hypoObj);
		primitive_group->setEnabled(false);
	}
	return hypoObj;
}

void PolyfitComputeConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj)
{
	if (polyfit_obj->building_name != GetBaseName(hypothesis_group->getName()).toStdString() || polyfit_obj->status < PolyFitObj::STT_hypomesh) {
		throw std::runtime_error("please generate hypothesis firstly");
		return;
	}
	ccHObject::Container planeObjs = GetEnabledObjFromGroup(hypothesis_group, CC_TYPES::PLANE, true, true);
	polyfit_obj->ComputeConfidence();

	MapFacetAttribute<VertexGroup*> facet_attrib_supporting_vertex_group_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_supporting_vertex_group());

	MapFacetAttribute<double> facet_attrib_supporting_point_num_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_supporting_point_num());
	MapFacetAttribute<double> facet_attrib_facet_area_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_facet_area());
	MapFacetAttribute<double> facet_attrib_covered_area_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_covered_area());
	MapFacetAttribute<double> facet_attrib_confidence_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_confidence());

	vector<double> all_conf;

	//////////////////////////////////////////////////////////////////////////AUTOFILTER
	std::map<ccHObject*, PlaneUnit*> plane_data;
	std::map<ccHObject*, stocker::Polyline2d> plane_convexhull;
//	if (polyfit_obj->auto_filter)
//	{
		for (auto & planeObj : planeObjs) {
			std::string plane_name = GetBaseName(planeObj->getParent()->getName()).toStdString();
			stocker::Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
			PlaneUnit plane_unit_ = FormPlaneUnit(plane_name, GetVcgPlane(planeObj), cur_plane_points, true);
			PlaneUnit* plane_unit = new PlaneUnit(plane_name, GetVcgPlane(planeObj), plane_unit_.convex_hull_prj);

			plane_data[planeObj] = plane_unit;
			stocker::Polyline2d plane_convex_hull = MakeLoopPolylinefromContour2d(Point3dToPlpoint2d(plane_unit_, plane_unit_.convex_hull_prj));
			plane_convexhull[planeObj] = plane_convex_hull;
		}
//	}
	//////////////////////////////////////////////////////////////////////////

	//! assign confidence information to hypothesis
	FOR_EACH_FACET(Map, polyfit_obj->hypothesis_mesh_, it) {
		Map::Facet* f = it;

		//! add to the plane it belongs
		/// get plane by name
		std::string support_plane_name = facet_attrib_supporting_vertex_group_[f]->label();
		
		ccHObject* plane_entity = GetPlaneEntityFromPrimGroup(hypothesis_group, support_plane_name.c_str());
		if (!plane_entity) throw std::runtime_error("cannot find plane");

		ccHObject::Container container_find = GetEnabledObjFromGroup(plane_entity, CC_TYPES::FACET, true, false);
		for (auto & child : container_find) {
			if (child->getName().toStdString() != f->label()) continue;

			ccFacet* facet = ccHObjectCaster::ToFacet(child);
			//! display
			double fitting = facet_attrib_supporting_point_num_[f];
			facet->setFitting(fitting);
			double area = facet_attrib_facet_area_[f];
			facet->setSurface(area);
			double coverage = facet_attrib_covered_area_[f] / area;
			facet->setCoverage(coverage);
			double confidence = facet_attrib_confidence_[f];
			facet->setConfidence(confidence);
			
			all_conf.push_back(confidence);

			//! convex_hull, 
			PlaneUnit plane_unit = *plane_data[plane_entity];
			stocker::Polyline2d plane_ch = plane_convexhull[plane_entity];
			ccPolyline* contour_entity = facet->getContour(); vector<CCVector3>ccv_poly = contour_entity->getPoints(true);
			Contour3d facet_contour_temp; for (auto & pt : ccv_poly) { facet_contour_temp.push_back(parse_xyz(pt)); }
			stocker::Polyline2d facet_contour = MakeLoopPolylinefromContour2d(Point3dToPlpoint2d(plane_unit, facet_contour_temp));
			//! check overlap
			double distance = DistancePolygonPolygon(facet_contour, plane_ch);
			facet->setDistance(distance);
			if (polyfit_obj->auto_filter) {
				if (distance > 2 && facet->getFitting() < 1) {
					facet->setEnabled(false);
				}
			}

			break;
		}		
	}
	//! colorize all the facet
	sort(all_conf.begin(), all_conf.end());
	double min_conf(all_conf.front()), max_conf(all_conf.back()), diag_conf;
	if (all_conf.size() > 40) {
		min_conf = all_conf[all_conf.size()*0.05];
		max_conf = all_conf[all_conf.size()*0.95];
	}
	diag_conf = max_conf - min_conf;
	ccColorScale::Shared colorScale = ccColorScalesManager::GetDefaultScale();
	ccHObject::Container container_find = GetEnabledObjFromGroup(hypothesis_group, CC_TYPES::FACET, true, true);
	for (auto & child : container_find)	{
		ccFacet* facet = ccHObjectCaster::ToFacet(child);
		double relativePos = (facet->getConfidence() - min_conf) / diag_conf;
		relativePos = relativePos >= 1 ? 1 : relativePos;
		relativePos = relativePos <= 0 ? 0 : relativePos;
		const ccColor::Rgb* col = colorScale->getColorByRelativePos(relativePos);
		facet->setColor(*col);
	}

	hypothesis_group->prepareDisplayForRefresh_recursive();
}

vector<String_String> CollectValidFacet(ccHObject::Container planeObjs)
{
	vector<String_String> valid_facet;
	for (auto & planeObj : planeObjs) {
		ccHObject::Container facets_;
		planeObj->filterChildren(facets_, false, CC_TYPES::FACET, true);

		for (auto & f : facets_) {
			if (f->isEnabled()) {
				valid_facet.push_back({ planeObj->getName().toStdString(), f->getName().toStdString() });
			}
		}
	}

	return valid_facet;
}

void UpdateConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj)
{
	ccHObject::Container planeObjs = GetEnabledObjFromGroup(hypothesis_group, CC_TYPES::PLANE, true, true);
	polyfit_obj->valid_group_facet_name = CollectValidFacet(planeObjs);
	polyfit_obj->UpdateConfidence(planeObjs);
}

ccHObject* PolyfitFaceSelection(ccHObject* hypothesis_group, PolyFitObj * polyfit_obj)
{
	ccHObject::Container planeObjs = GetEnabledObjFromGroup(hypothesis_group, CC_TYPES::PLANE, true, true);
	vector<String_String> name_group_facet = CollectValidFacet(planeObjs);
	polyfit_obj->UpdateValidFacet(name_group_facet);
	polyfit_obj->UpdateConfidence(planeObjs);
	polyfit_obj->FacetOptimization();

	ccHObject* polyfit_model = nullptr;
	if (!polyfit_obj->optimized_mesh_) return nullptr;
	polyfit_model = new ccHObject(GetBaseName(hypothesis_group->getName()) + BDDB_POLYFITOPTM_SUFFIX);
	
	// TODO: display
	// Plane, and sub facet
	// find subfacet by plane

	Map* mesh = Geom::duplicate(polyfit_obj->optimized_mesh_);
	Attribute<Map::Vertex, int>	vertex_id(mesh->vertex_attribute_manager());
	MapEnumerator::enumerate_vertices(const_cast<Map*>(mesh), vertex_id, 0);

	Map::Vertex_const_iterator begin = mesh->vertices_begin();

	vector<Contour3d> all_contour_points;
	//! subfacet
	FOR_EACH_FACET_CONST(Map, mesh, it) {
		Map::Halfedge* jt = it->halfedge();
		it->label();
		Contour3d contour_points;
		do {
			vec3 pt = (begin + vertex_id[jt->vertex()])->point();
			contour_points.push_back({ pt.data()[0],pt.data()[1],pt.data()[2] });
			jt = jt->next();
		} while (jt != it->halfedge());
	}

	if (hypothesis_group->getParent()) {
		hypothesis_group->getParent()->addChild(polyfit_model);
	}
	return polyfit_model;
}

PolyFitObj::PolyFitObj() :
	status(STT_prepared)
{
}

PolyFitObj::~PolyFitObj()
{
}

void PolyFitObj::clear()
{
// 	if (point_set_)
// 		point_set_.forget();

	if (hypothesis_mesh_)
		hypothesis_mesh_.forget();

	if (optimized_mesh_)
		optimized_mesh_.forget();

	if (hypothesis_) {
		delete hypothesis_;
		hypothesis_ = 0;
	}

	status = STT_prepared;
}

void PolyFitObj::initGenerator(ccHObject::Container planeObjs)
{
	PointSet* pset = GetPointSetFromPlaneObjs(planeObjs);
	hypothesis_ = new HypothesisGenerator(pset);
}

void PolyFitObj::GenerateHypothesis()
{
	if (!hypothesis_) {
		throw std::runtime_error("no hypothesis");
		return;
	}
	if (hypothesis_mesh_) {
		hypothesis_mesh_.forget();
	}
	hypothesis_->strict_intersect = strict_intersection;
	hypothesis_->strict_ints_snap_squared = snap_intersection * snap_intersection;
	hypothesis_mesh_ = hypothesis_->generate();
}

void PolyFitObj::ComputeConfidence()
{
	Method::UpdateGlobalDataFitting(data_fitting);
	Method::UpdateGlobalModelCoverage(model_coverage);
	Method::UpdateGlobalModelComplexity(model_complexity);
	hypothesis_->compute_confidences(hypothesis_mesh_, use_confidence);
}

void PolyFitObj::FacetOptimization()
{
	if (status < STT_confidence){
		throw std::runtime_error("no available hypothesis and confidence");
		return;
	}
	Map* mesh = Geom::duplicate(hypothesis_mesh_);
	PointSet* point_set_ = hypothesis_->point_set();
	const HypothesisGenerator::Adjacency& adjacency = hypothesis_->extract_adjacency(mesh);
	FaceSelection selector(point_set_, mesh);
	selector.optimize_cc(adjacency, valid_group_facet_name);
	optimized_mesh_ = mesh;
}

void PolyFitObj::UpdateValidFacet(std::vector<stocker::String_String> valid_update)
{
	vector<String_String> valid_for_selection;
	for (auto & f : valid_group_facet_name) {
		if (find(valid_update.begin(), valid_update.end(), f) != valid_update.end()) {
			valid_for_selection.push_back(f);
		}
	}
	swap(valid_group_facet_name, valid_for_selection);
}

bool PolyFitObj::OutputResultToObjFile(BDBaseHObject* baseObj, std::string & file_path)
{
	if (status < STT_optimized) {
		return false;
	}
	auto& bd = baseObj->block_prj.m_builder.sbuild.find(stocker::BuilderBase::BuildNode::Create(building_name));
	file_path = baseObj->GetPathModelObj(building_name);
	
	std::ofstream out(file_path.c_str());
	if (out.fail()) {
		std::string error_info = "cannot open file: " + file_path;
		throw std::runtime_error(error_info.c_str());
		return false;
	}
	out.precision(16);
	// Obj files numbering starts with 1
	Map* mesh = Geom::duplicate(optimized_mesh_);
	Attribute<Map::Vertex, int>	vertex_id(mesh->vertex_attribute_manager());
	MapEnumerator::enumerate_vertices(const_cast<Map*>(mesh), vertex_id, 1);

	// Output Vertices
	
	FOR_EACH_VERTEX_CONST(Map, mesh, it) {
		vec3 pt = it->point();
		
		Vec3d pt_ = baseObj->ToGlobal(Vec3d(pt.data()[0], pt.data()[1], pt.data()[2]));
		out << "v " << pt_ << std::endl;
	}

	// Output facets
	FOR_EACH_FACET_CONST(Map, mesh, it) {
		Map::Halfedge* jt = it->halfedge();
		out << "f ";
		do {
			out << vertex_id[jt->vertex()] << " ";
			jt = jt->next();
		} while (jt != it->halfedge());
		out << std::endl;
	}

	// Output outlines
	FOR_EACH_FACET_CONST(Map, mesh, it) {
		Map::Halfedge* jt = it->halfedge();
		out << "l ";
		do {
			out << vertex_id[jt->vertex()] << " ";
			jt = jt->next();
		} while (jt != it->halfedge());
		out << std::endl;
	}

	MapVertexLock is_locked(const_cast<Map*>(mesh));
	FOR_EACH_VERTEX_CONST(Map, mesh, it) {
		if (is_locked[it]) {
			out << "# anchor " << vertex_id[it] << std::endl;
		}
	}
	out.close();

	std::cout << "[BDRecon] model file saved to: " << file_path << std::endl;
	return true;
}

bool PolyFitObj::FindValidFacet(std::string name_plane, std::string name_facet)
{
	return find(valid_group_facet_name.begin(),	valid_group_facet_name.end(),
		String_String(name_plane, name_facet)) != valid_group_facet_name.end();
}

void PolyFitObj::UpdateConfidence(ccHObject::Container PlaneObjs)
{
	MapFacetAttribute<VertexGroup*> facet_attrib_supporting_vertex_group_(hypothesis_mesh_, Method::Get_facet_attrib_supporting_vertex_group());

	MapFacetAttribute<double> facet_attrib_confidence_(hypothesis_mesh_, Method::Get_facet_attrib_confidence());
	vector<String_String> valid_facet;
	for (auto & planeObj : PlaneObjs) {
		ccHObject::Container facets_;
		planeObj->filterChildren(facets_, false, CC_TYPES::FACET, true);

		for (auto & f_ : facets_) {
			if (f_->isEnabled()) {
				if (!FindValidFacet(planeObj->getName().toStdString(), f_->getName().toStdString())) 
					continue;
				
				ccFacet* facet = ccHObjectCaster::ToFacet(f_);
				double confidence = facet->getConfidence();

				FOR_EACH_FACET(Map, hypothesis_mesh_, it) {
					Map::Facet* f = it;
					VertexGroup* g = facet_attrib_supporting_vertex_group_[f];

					if (f->label() == f_->getName().toStdString() &&
						g->label() == planeObj->getName().toStdString()) {
						facet_attrib_confidence_[f] = confidence;
						break;
					}					
				}
			}
		}
	}
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

#include "vcg/space/distance3.h"
bool SamePoint(Vec3d pt1, Vec3d pt2) {
	return (pt1 - pt2).Norm() < 0.000001;
}
bool DistPoint(Vec3d pt1, Vec3d pt2) {
	return (pt1 - pt2).Norm() > 0.01;
}
bool SegmentSegmentShouldMerge(Seg3d ln_1, Seg3d ln_2, double dist_thre = 1, double angle_thre = 0.9)
{
	if (SamePoint(ln_1.P0(), ln_2.P0()) && DistPoint(ln_1.P0(), ln_2.P1()))	{
		return false;
	}
	if (SamePoint(ln_1.P1(), ln_2.P0()) && DistPoint(ln_1.P1(), ln_2.P1())) {
		return false;
	}

	bool parallel; double dist;
	vcg::Point3d pt1, pt2;
	vcg::SegmentSegmentDistance(ln_1, ln_2, dist, parallel, pt1, pt2);
	if (dist < 0.000001) {
		return true;
	}
	Vec3d dir_1(ln_1.P1() - ln_1.P0());
	Vec3d dir_2(ln_2.P1() - ln_2.P0());

	// < 25 degree
	if (dir_1*dir_2 > 0.9 && dist < 1) {
		return true;
	}
	
	return false;
}
bool SegmentCanAddToPolyline(Polyline3d lines, Seg3d ln, double dist_thre = 1, double angle_thre = 0.9)
{
	for (auto & ln_ : lines) {
		if (SegmentSegmentShouldMerge(ln_, ln, dist_thre, angle_thre)) {
			return false;
		}
	}
	return true;
}
ccHObject* ConstrainedMesh(ccHObject* planeObj)
{
	ccHObject* plane_cloud_obj = planeObj->getParent();
	if (!planeObj->isA(CC_TYPES::PLANE) || !plane_cloud_obj) {
		throw std::runtime_error("invalid planar entity");
		return nullptr;
	}

	Contour3d plane_points = GetPointsFromCloud(plane_cloud_obj);
	PlaneUnit plane_unit = FormPlaneUnit(plane_points, "temp", true);
	
	Polyline3d plane_sharps;
	Contour3d alpha_shape;
	GLMesh mesh_out;

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
	Polyline3d outline_poly = MakeLoopPolylinefromContour3d(outline_points);

	Polyline3d line_pool;
// 	for (auto & ln : boundary_lines) {
// 		if (SegmentCanAddToPolyline(line_pool, ln)) {
// 			line_pool.push_back(ln);
// 		}
// 	}
	for (auto & ln : outline_poly) {
		if (/*SegmentCanAddToPolyline(line_pool, ln)*/1) {
			line_pool.push_back(ln);
		}
	}
	Contour3d point_pool;
	for (auto pt : boundary_points) {		
		if (DistancePointPolygon(pt,line_pool) < 1)	continue;
		bool add = true;
		for (auto & pt_ : point_pool) {
			if (vcg::Distance(pt, pt_) < 1) {
				add = false;
				break;
			}			
		}
		if (!add) continue;
		
		point_pool.push_back(pt);
	}

	PlaneConstrainedDelaunayTriangulation(plane_unit, point_pool, line_pool, mesh_out, false);

	ccPointCloud* vertices = new ccPointCloud("vertices");
	for (auto & pt : mesh_out.vert) {		
		vertices->addPoint(CCVector3(vcgXYZ(pt.P())));
	}
	if (vertices->reserveTheNormsTable()) {
		for (auto & pt : mesh_out.vert) {
//			vertices->addNorm(CCVector3(vcgXYZ(pt.N())));
			vertices->addNorm(CCVector3(vcgXYZ(plane_unit.plane.Direction())));
		}
	}

	Polyline2d outlines = Line3dToPlline2d(plane_unit, outline_poly);
	ccMesh* mesh = new ccMesh(vertices);
	mesh->setName(GetBaseName(planeObj->getParent()->getName() + ".cdt"));
	mesh->addChild(vertices);
	ccPointCloud* point_cloud_entity = ccHObjectCaster::ToPointCloud(plane_cloud_obj);
	vertices->setGlobalShift(point_cloud_entity->getGlobalShift());
	vertices->setGlobalScale(point_cloud_entity->getGlobalScale());	

	for (auto & face : mesh_out.face) {
		Vec2d pt = plane_unit.Point3dPrjtoPlpoint2d( (face.P(0) + face.P(1) + face.P(2)) / 3);
		if (vcg::PointInsidePolygon(pt, outlines)) {
			mesh->addTriangle(
				vcg::tri::Index(mesh_out, face.cV(0)),
				vcg::tri::Index(mesh_out, face.cV(1)),
				vcg::tri::Index(mesh_out, face.cV(2)));
		}		
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
		points = GetPointsFromCloudInsidePolygonXY(point_cloud, polygon, DBL_MAX);
	}
	else if (primitive) {
		points = GetPointsFromCloudInsidePolygonXY(primitive, polygon, DBL_MAX);
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
		compo_temp.push_back(0);
		components.push_back(compo_temp);
	}
	return components;
}

ccHObject* LoD1FromFootPrint(ccHObject* buildingObj)
{	
	BDBaseHObject* baseObj = GetRootBDBase(buildingObj);
	if (!baseObj) {
		return nullptr;
	}

	QString building_name = GetBaseName(buildingObj->getName());
	BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
	ccHObject* cloudObj = baseObj->GetOriginPointCloud(building_name, true);
	ccHObject* prim_group_obj = baseObj->GetPrimitiveGroup(building_name, true);

	StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(building_name, true);
	ccHObject::Container footprintObjs = blockgroup_obj->getFootPrints();

	for (size_t i = 0; i < footprintObjs.size(); i++) {
		if (!footprintObjs[i]->isEnabled()) continue;

		StFootPrint* foot_print = ccHObjectCaster::ToStFootPrint(footprintObjs[i]);
		//! get height
		Contour3d points_inside;
		double height = foot_print->getHeight();
		double ground = foot_print->getGround();
		if (fabs(height - ground) < 1e-6) {
			if (!DeduceFootPrintHeight(cloudObj, prim_group_obj, footprintObjs[i], points_inside, height)) {
				std::cout << "cannot deduce height from footprint " << foot_print->getName().toStdString() << std::endl;
				continue;
			}			
		}		
		
		std::vector<CCVector3> foot_print_points = foot_print->getPoints(false);

		std::vector<CCVector3> top_points;
		std::vector<CCVector3> bottom_points;
		for (auto & pt : foot_print_points) {
			top_points.push_back(CCVector3(pt.x, pt.y, height));
			bottom_points.push_back(CCVector3(pt.x, pt.y, ground));
		}
		StBlock* block_entity = new StBlock(top_points, bottom_points);
		int biggest = GetMaxNumberExcludeChildPrefix(blockgroup_obj, BDDB_BLOCK_PREFIX);
		block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(biggest + 1));
		foot_print->addChild(block_entity);
	}
	return blockgroup_obj;

#if 0
	std::vector<std::vector<int>> components;

	CCVector3d loadCoordinatesShift(0, 0, 0);
	bool loadCoordinatesTransEnabled = false;
	FileIOFilter::LoadParameters parameters; {
		parameters.alwaysDisplayLoadDialog = false;
		parameters.shiftHandlingMode = ccGlobalShiftManager::NO_DIALOG_AUTO_SHIFT;
	}
	CC_FILE_ERROR result = CC_FERR_NO_ERROR;

	// deprecated
	std::vector<Polyline3d> polygons;
	std::vector<double> ft_heights;

	{
		// TODO: get the relationships of the polygons
		// now the polygons should not be overlapped
		// if more than one polygons are given, will create multiple models
		for (size_t i = 0; i < polygonObjs.size(); i++) {
			StFootPrint* polyObj = ccHObjectCaster::ToStFootPrint(polygonObjs[i]);
			assert(polyObj);
			Polyline3d polygon = GetPolygonFromPolyline(polyObj);
			double footprint_height = polyObj->getHeight();
			polygons.push_back(polygon);
			ft_heights.push_back(footprint_height);
		}
	}

	QString model_group_name = building_name + BDDB_LOD1MODEL_SUFFIX;
	ccHObject* model_group = nullptr;
	for (size_t i = 0; i < buildingObj->getChildrenNumber(); i++) {
		if (buildingObj->getChild(i)->getName() == model_group_name) {
			model_group = buildingObj->getChild(i);
		}
	}
	if (!model_group) {
		model_group = new ccHObject(model_group_name);
	}

	for (size_t i = 0; i < polygons.size(); i++) {

		int biggest = GetMaxNumberExcludeChildPrefix(model_group, BDDB_LOD1MODEL_SUFFIX);
		QString model_name = BDDB_LOD1MODEL_PREFIX + QString::number(biggest + 1);

		char output_path[256];
		sprintf(output_path, "%s%s%s%s%s",
			build_unit.file_path.model_dir.c_str(),
			building_name.toStdString().c_str(), ".",
			model_name.toStdString().c_str(), ".obj");

		if (!LoD1FromFootPrintAndHeight(polygons[i], ft_heights[i], output_path)) {
			continue;
		}

		if (!QFile::exists(QString(output_path))) return nullptr;

		ccHObject* cur_model = FileIOFilter::LoadFromFile(output_path, parameters, result, QString());

		model_group->addChild(cur_model);
	}
	buildingObj->addChild(model_group);
	return model_group;
#endif
}

//! 3D4EM	.lod2.model
ccHObject* LoD2FromFootPrint(ccHObject* buildingObj, ccHObject::Container footprintObjs, double ground_height)
{
	if (!buildingObj || footprintObjs.empty()) { return nullptr; }
	BDBaseHObject* baseObj = GetRootBDBase(buildingObj);
	if (!baseObj) {
		return nullptr;
	}
	QString building_name = buildingObj->getName();
	BuildUnit build_unit = baseObj->GetBuildingUnit(building_name.toStdString());
	ccHObject* cloudObj = baseObj->GetOriginPointCloud(building_name, true);
	ccHObject* prim_group_obj = baseObj->GetPrimitiveGroup(building_name, true);

	StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName(), true);

	std::vector<std::vector<int>> components = GroupFootPrint(footprintObjs);

	for (size_t i = 0; i < components.size(); i++) {
		stocker::BuilderLOD2 builder_3d4em(true);

		std::vector<Contour3d> contours;
		std::vector<int> cur_component = components[i];
		Polyline3d first_polygon; double footprint_height(-999999);
		bool valid = false;
		ccHObject* first_footprint = nullptr;
		for (size_t j = 0; j < cur_component.size(); j++) {
			first_footprint = footprintObjs[cur_component[j]];
			if (!first_footprint->isEnabled()) {
				continue;
			}
			Polyline3d polygon = GetPolygonFromPolyline(first_footprint);
			if (j == 0) {
				valid = true;
				first_polygon = polygon;
				footprint_height = ccHObjectCaster::ToStFootPrint(first_footprint)->getHeight();
			}
			contours.push_back(ToContour(polygon, 0));
		}
		if (!valid) {
			continue;
		}
		builder_3d4em.SetFootPrint(contours);

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

		// primitive first
		if (prim_group_obj) {
			ccHObject::Container prim_objs = GetEnabledObjFromGroup(prim_group_obj, CC_TYPES::PLANE, true, true);
			ccHObject::Container point_cloud_objs;
			for (ccHObject* obj : prim_objs) {
				if (obj->getParent() && obj->getParent()->isEnabled()) { point_cloud_objs.push_back(obj->getParent()); }
			}
			std::vector<Contour3d> points = GetPointsFromCloudInsidePolygonXY(point_cloud_objs, first_polygon, footprint_height);
			builder_3d4em.SetSegmentedPoints(points);
		}
		else if (cloudObj) {
			Contour3d points = GetPointsFromCloudInsidePolygonXY(cloudObj, first_polygon, footprint_height);
			builder_3d4em.SetBuildingPoints(points);
			if (!builder_3d4em.PlaneSegmentation()) continue;;
		}
		else {
			throw std::runtime_error("cannot find original points or primitive group");
			continue;
		}

		if (!builder_3d4em.BuildingReconstruction()) continue;

		if (!QFile::exists(QString(output_path))) continue;

		std::vector<Contour3d> roof_polygons = builder_3d4em.GetRoofPolygons();
		for (Contour3d roof_points : roof_polygons) {
			std::vector<CCVector3> top_points;
			std::vector<CCVector3> bottom_points;
			for (auto & pt : roof_points) {
				top_points.push_back(CCVector3(pt.X(), pt.Y(), footprint_height));
				bottom_points.push_back(CCVector3(pt.X(), pt.Y(), ground_height));
			}
			StBlock* block_entity = new StBlock(top_points, bottom_points);
			int biggest = GetMaxNumberExcludeChildPrefix(blockgroup_obj, BDDB_BLOCK_PREFIX);
			block_entity->setName(BDDB_BLOCK_PREFIX + QString::number(biggest + 1));
			first_footprint->addChild(block_entity);
		}

		// 		ccHObject* cur_model = FileIOFilter::LoadFromFile(output_path, parameters, result, QString());
		// 		cur_model->setName(model_name);
		// 		model_group->addChild(cur_model);
	}
	return blockgroup_obj;
}

ccHObject* LoD2FromFootPrint(ccHObject* buildingObj, double ground_height)
{	
	if (buildingObj->isA(CC_TYPES::ST_FOOTPRINT)) {
		ccHObject::Container footprintObjs;
		footprintObjs.push_back(buildingObj);
		return LoD2FromFootPrint(GetParentBuilding(buildingObj), footprintObjs, ground_height);
	}
	else if (buildingObj->isA(CC_TYPES::ST_BUILDING)) {
		BDBaseHObject* baseObj = GetRootBDBase(buildingObj);
		if (!baseObj) {
			return nullptr;
		}
		StBlockGroup* blockgroup_obj = baseObj->GetBlockGroup(buildingObj->getName(), true);
		ccHObject::Container footprintObjs = blockgroup_obj->getFootPrints();
		return LoD2FromFootPrint(buildingObj, footprintObjs, ground_height);
	}
}