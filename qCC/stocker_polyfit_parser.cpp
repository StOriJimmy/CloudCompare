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
#include "polyfit/model/map_enumerator.h"
#include "polyfit/model/map_serializer.h"
#endif // USE_STOCKER

#include "stockerDatabase.h"

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

//	bd00000000.hypothesis
//	-Plane0						point cloud
//	 --Plane					Plane
//	  ---vertices				(Plane accessory)
//	  ---compressed normals		(Plane accessory)
//	  ---Facet0					Facet
//	   ----Contour points		(Facet accessory)

	StPrimGroup* hypoObj;
	BDBaseHObject* baseObj = GetRootBDBase(primitive_group);
	QString building_name = GetBaseName(primitive_group->getName());
	CCVector3d global_shift(0, 0, 0);
	double global_scale(0);
	stocker::Polyline2d building_convex_hull_2d;
	if (baseObj) {
		global_shift = CCVector3d(vcgXYZ(baseObj->global_shift));
		global_scale = baseObj->global_scale;
		hypoObj = baseObj->GetHypothesisGroup(building_name);
		stocker::BuildUnit* bd = baseObj->GetBuildingSp(building_name.toStdString());
		if (!bd) return nullptr;
		stocker::Contour2d bd_cvx = bd->convex_hull_xy;
		assert(!bd_cvx.empty());
		building_convex_hull_2d = stocker::MakeLoopPolylinefromContour(bd_cvx);
	}
	else {
		hypoObj = new StPrimGroup(building_name + BDDB_POLYFITHYPO_SUFFIX);
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
	
	int facet_count = 0;
	FOR_EACH_FACET(Map, polyfit_obj->hypothesis_mesh_, it) {
		Map::Facet* f = it;

		//! add to the plane it belongs
		/// get plane by name
		std::string support_plane_name = facet_attrib_supporting_vertex_group_[f]->label();
		ccHObject* plane_entity = GetPlaneEntityFromPrimGroup(hypoObj, support_plane_name.c_str());
		if (!plane_entity) throw std::runtime_error("cannot find plane");

		Polygon3d contour_polygon = Geom::facet_polygon(f);
		std::vector<CCVector3> ccv_poly;
		for (auto & pt : contour_polygon) {
			ccv_poly.push_back(CCVector3(pt.data()[0], pt.data()[1], pt.data()[2]));
		}
		//! check if the facet inside the building's convex hull
		
		if (polyfit_obj->auto_filter && !building_convex_hull_2d.empty()) {
			bool facet_inside_convex = false;			
			for (auto pt : ccv_poly) {
				if (vcg::PointInsidePolygon(ToVec2d(baseObj->ToGlobal(stocker::parse_xyz(pt))), building_convex_hull_2d)) {
					facet_inside_convex = true;
					break;
				}
			}
			if (!facet_inside_convex) {
				continue;
			}
		}
		
		//! each facet is a facet entity under plane
		PointCoordinateType plane_equation[4];
		ccPlane* cc_plane = ccHObjectCaster::ToPlane(plane_entity);
		CCVector3 N; PointCoordinateType dis; cc_plane->getEquation(N, dis);
		plane_equation[0] = N.x; plane_equation[1] = N.y; plane_equation[2] = N.z; plane_equation[3] = dis;
		ccFacet* facet_entity = ccFacet::CreateFromContour(ccv_poly, f->label().c_str(), false, plane_equation);
		
		ccPolyline* contour_entity = facet_entity->getContour();
		if (contour_entity) {
			contour_entity->setGlobalShift(global_shift);
			contour_entity->setGlobalScale(global_scale);

			//! get the distance
			{
				CCLib::Neighbourhood YK(facet_entity->getContourVertices());
				stocker::Contour2d contour_points_2d;
				CCVector3 O, X, Y;
				YK.projectPointsOn2DPlane<stocker::Vec2d>(contour_points_2d, plane_equation, &O, &X, &Y, CCLib::Neighbourhood::None);
				stocker::Polyline2d facet_contour = stocker::MakeLoopPolylinefromContour(contour_points_2d);

				stocker::Contour2d contour_points_2d_plane;
				YK.projectPointsOn2DPlane<stocker::Vec2d, CCVector3>(contour_points_2d_plane, cc_plane->getProfile(), plane_equation, &O, &X, &Y, false);
				stocker::Polyline2d plane_contour = stocker::MakeLoopPolylinefromContour(contour_points_2d_plane);

				double distance = stocker::DistancePolygonPolygon(facet_contour, plane_contour);
				facet_entity->setDistance(distance);
			}
		}
		else {
			facet_entity->setDistance(-1);
			std::string error_info = "error contour: plane-" + support_plane_name + " facet-" + f->label();
			ccLog::Warning(error_info.c_str());
		}
		plane_entity->addChild(facet_entity);
		facet_count++;
	}
	std::cout << facet_count << " facets generated!" << std::endl;
	hypoObj->setDisplay_recursive(primitive_group->getDisplay());
	if (primitive_group->getParent()) {
		if (!hypoObj->getParent()) {
			primitive_group->getParent()->addChild(hypoObj);
		}
		primitive_group->setEnabled(false);
	}
	return hypoObj;
}

std::vector<stocker::String_String> CollectValidFacet(ccHObject::Container planeObjs)
{
	std::vector<stocker::String_String> valid_facet;
	for (auto & planeObj : planeObjs) {
		ccHObject::Container facets_;
		planeObj->filterChildren(facets_, false, CC_TYPES::FACET, true);

		for (auto & f : facets_) {
			if (f->isEnabled()) {
				valid_facet.push_back({ planeObj->getParent()->getName().toStdString(), f->getName().toStdString() });
			}
		}
	}

	return valid_facet;
}

void PolyfitComputeConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj)
{
	if (polyfit_obj->building_name != GetBaseName(hypothesis_group->getName()).toStdString() || polyfit_obj->status < PolyFitObj::STT_hypomesh) {
		throw std::runtime_error("please generate hypothesis firstly");
		return;
	}
	ccHObject::Container planeObjs = GetEnabledObjFromGroup(hypothesis_group, CC_TYPES::PLANE, true, true);

	std::vector<stocker::String_String> name_group_facet = CollectValidFacet(planeObjs);
	polyfit_obj->UpdateValidFacet(name_group_facet);
	polyfit_obj->ComputeConfidence();

	MapFacetAttribute<VertexGroup*> facet_attrib_supporting_vertex_group_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_supporting_vertex_group());

	MapFacetAttribute<double> facet_attrib_supporting_point_num_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_supporting_point_num());
	MapFacetAttribute<double> facet_attrib_facet_area_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_facet_area());
	MapFacetAttribute<double> facet_attrib_covered_area_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_covered_area());
	MapFacetAttribute<double> facet_attrib_confidence_(polyfit_obj->hypothesis_mesh_, Method::Get_facet_attrib_confidence());

	std::vector<double> all_conf;

	//////////////////////////////////////////////////////////////////////////AUTOFILTER
#if 0	
	std::map<ccHObject*, PlaneUnit*> plane_data;
	std::map<ccHObject*, stocker::Polyline2d> plane_convexhull;
	{
		for (auto & planeObj : planeObjs) {
			std::string plane_name = GetBaseName(planeObj->getParent()->getName()).toStdString();
			stocker::Contour3d cur_plane_points = GetPointsFromCloud3d(planeObj->getParent());
			PlaneUnit plane_unit_ = FormPlaneUnit(plane_name, GetVcgPlane(planeObj), cur_plane_points, true);
			PlaneUnit* plane_unit = new PlaneUnit(plane_name, GetVcgPlane(planeObj), plane_unit_.convex_hull_prj); // TODO: delete or add to planedata in primitivegroup info

			plane_data[planeObj] = plane_unit;
			stocker::Polyline2d plane_convex_hull = MakeLoopPolylinefromContour(Point3dToPlpoint2d(plane_unit_, plane_unit_.convex_hull_prj));
			plane_convexhull[planeObj] = plane_convex_hull;
		}
	}
#endif
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

#if 0
			//! convex_hull, 
			PlaneUnit plane_unit = *plane_data[plane_entity];
			stocker::Polyline2d plane_ch = plane_convexhull[plane_entity];
			ccPolyline* contour_entity = facet->getContour();
			if (!contour_entity) {
				facet->setDistance(-1);
				break;
			}
			vector<CCVector3>ccv_poly = contour_entity->getPoints(true);
			Contour3d facet_contour_temp; for (auto & pt : ccv_poly) { facet_contour_temp.push_back(parse_xyz(pt)); }
			stocker::Polyline2d facet_contour = MakeLoopPolylinefromContour(Point3dToPlpoint2d(plane_unit, facet_contour_temp));
			//! check overlap
			double distance = DistancePolygonPolygon(facet_contour, plane_ch);
			facet->setDistance(distance);
			if (polyfit_obj->auto_filter) {
				if (distance > 2 && facet->getFitting() < 1) {
					facet->setEnabled(false);
				}
			}
#endif

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

void UpdateConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj)
{
	ccHObject::Container planeObjs = GetEnabledObjFromGroup(hypothesis_group, CC_TYPES::PLANE, true, true);
	polyfit_obj->valid_group_facet_name = CollectValidFacet(planeObjs);
	polyfit_obj->UpdateConfidence(planeObjs);
}

ccHObject* PolyfitFaceSelection(ccHObject* hypothesis_group, PolyFitObj * polyfit_obj)
{
	ccHObject::Container planeObjs = GetEnabledObjFromGroup(hypothesis_group, CC_TYPES::PLANE, true, true);
	std::vector<stocker::String_String> name_group_facet = CollectValidFacet(planeObjs);
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

	std::vector<stocker::Contour3d> all_contour_points;
	//! subfacet
	FOR_EACH_FACET_CONST(Map, mesh, it) {
		Map::Halfedge* jt = it->halfedge();
		it->label();
		stocker::Contour3d contour_points;
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
	hypothesis_->compute_confidences_cc(hypothesis_mesh_, valid_group_facet_name, use_confidence);
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
// 	vector<String_String> valid_for_selection;
// 	for (auto & f : valid_group_facet_name) {
// 		if (find(valid_update.begin(), valid_update.end(), f) != valid_update.end()) {
// 			valid_for_selection.push_back(f);
// 		}
// 	}
//	swap(valid_group_facet_name, valid_for_selection);
	valid_group_facet_name.clear(); valid_group_facet_name.shrink_to_fit();
	valid_group_facet_name.assign(valid_update.begin(), valid_update.end());
}

bool PolyFitObj::OutputResultToObjFile(BDBaseHObject* baseObj, std::string & file_path)
{
	if (status < STT_optimized) {
		return false;
	}
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
		
		stocker::Vec3d pt_ = baseObj->ToGlobal(stocker::Vec3d(pt.data()[0], pt.data()[1], pt.data()[2]));
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
		stocker::String_String(name_plane, name_facet)) != valid_group_facet_name.end();
}

void PolyFitObj::UpdateConfidence(ccHObject::Container PlaneObjs)
{
	MapFacetAttribute<VertexGroup*> facet_attrib_supporting_vertex_group_(hypothesis_mesh_, Method::Get_facet_attrib_supporting_vertex_group());

	MapFacetAttribute<double> facet_attrib_confidence_(hypothesis_mesh_, Method::Get_facet_attrib_confidence());
	std::vector<stocker::String_String> valid_facet;
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
