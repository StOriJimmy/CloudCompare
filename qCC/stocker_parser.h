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

#ifndef __STOCKER_PARSER_HEADER__
#define __STOCKER_PARSER_HEADER__

#include "ccHObject.h"
#include "mainwindow.h"

#include "stockerDatabase.h"

#ifdef USE_STOCKER
#include "builderlod3/builderlod3.h"
#include "builderlod2/builderlod2.h"
#include "buildercore/StBuilder.h"
#include "ioctrl/StFileOperator.hpp"

#include "polyfit/method/hypothesis_generator.h"
#include "polyfit/method/face_selection.h"
#include "polyfit/method/method_global.h"
#include "polyfit/model/point_set_io.h"
#include "polyfit/model/point_set.h"
#include "polyfit/model/map_geometry.h"
#include "polyfit/model/map_io.h"
#endif // USE_STOCKER

//class BDBaseHObject;
class PolyFitObj;


template <typename T = stocker::Vec3d>
auto GetPointsFromCloud3d(ccHObject* entity, bool global = false)->std::vector<T>;
template <typename T = vcg::Point3f>
auto GetPointsFromCloud3f(ccHObject* entity, bool global = false)->std::vector<T>;
stocker::Contour3f GetPointsFromCloud3f(ccHObject* entity, bool global = false);
bool GetPointsFromCloud(ccHObject * entity, stocker::Contour3d & global, stocker::Contour3f & local);

double GetPointsAverageSpacing(ccHObject* pc);

stocker::Contour3d GetPointsFromCloudInsidePolygonXY(ccHObject * entity, stocker::Polyline3d polygon, double height);
std::vector<stocker::Contour3d> GetPointsFromCloudInsidePolygonsXY(ccHObject::Container entities, stocker::Polyline3d polygon, double height, bool skip_empty = true);
std::vector<stocker::Contour3d> GetPointsFromCloudInsidePolygonsXY(ccHObject* entity, stocker::Polyline3d polygon, double height, bool skip_empty = true);
stocker::Contour3d GetPointsFromCloudInsidePolygon3d(ccHObject * entity, stocker::Polyline3d polygon, stocker::Contour3d & remained, double distance_threshold);
stocker::Polyline3d GetPolygonFromPolyline(ccHObject * entity);
stocker::Polyline3d GetPolylineFromEntities(ccHObject::Container entities);
vector<vector<stocker::Contour3d>> GetOutlinesFromOutlineParent(ccHObject * entity);

ccHObject::Container GetPlaneEntitiesBySelected(ccHObject * select);
ccHObject::Container GetBuildingEntitiesBySelected(ccHObject * select);
ccPlane * GetPlaneFromCloud(ccHObject * entity);
ccPlane * GetPlaneFromPlaneOrCloud(ccHObject * entity);
ccHObject * GetPlaneEntityFromPrimGroup(ccHObject * prim, QString name);
vcg::Plane3d GetVcgPlane(ccHObject * planeObj);

ccPlane* FitPlaneAndAddChild(ccPointCloud* cloud, const vcg::Plane3d* plane_para = nullptr);

ccPointCloud* AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col);

void AddSegmentsToVertices(ccPointCloud * cloud, stocker::Polyline3d lines, QString Prefix, ccColor::Rgb col);

template <typename T = stocker::Vec3d>
ccPointCloud * AddPointsAsPlane(std::vector<T> points, QString name, ccColor::Rgb col, const vcg::Plane3d* plane_para = nullptr);

ccPointCloud* AddSegmentsAsPlane(stocker::Polyline3d lines, QString lines_prefix, ccColor::Rgb col, ccHObject* _exist_cloud = nullptr);

ccPolyline * AddPolygonAsPolyline(stocker::Contour3d points, QString name, ccColor::Rgb col, bool close);

ccPolyline * AddPolygonAsPolyline(stocker::Polyline3d polygon, QString name, ccColor::Rgb col, bool close);

StPrimGroup * LoadPlaneParaAsPrimtiveGroup(ccPointCloud * entity_cloud, ccPointCloud * todo_cloud);

ccHObject* PlaneSegmentationRgGrow(ccHObject* entity, bool overwrite,
	int min_pts, double distance_epsilon, double seed_raius, double growing_radius,
	double merge_threshold = -1, double split_threshold = -1);

ccHObject* PlaneSegmentationRansac(ccHObject* entity, bool overwrite, ccPointCloud* todo_cloud,
	int min_pts, double distance_epsilon, double seed_raius, double normal_threshold, double ransac_probability,
	double merge_threshold = -1, double split_threshold = -1);

ccHObject * PlaneSegmentationATPS(ccHObject * entity, bool overwrite, ccPointCloud * todo_cloud, 
	bool* iter_times = nullptr,
	int* kappa_t = nullptr, double* delta_t = nullptr, double* tau_t = nullptr, 
	double* gamma_t = nullptr, double* epsilon_t = nullptr, double* theta_t = nullptr);

//! mode = 0, area, mode = 1, perimeter of convex hull, 
void CalculatePlaneQuality(ccHObject::Container primObjs, int mode);

void RetrieveUnassignedPoints(ccHObject * original_cloud, ccHObject * prim_group, ccPointCloud * todo_point);
void RetrieveAssignedPoints(ccPointCloud * todo_cloud, ccPointCloud * plane_cloud, double distance_threshold);

ccHObject::Container CalcPlaneIntersections(ccHObject::Container entity_planes, double distance);
ccHObject* CalcPlaneBoundary(ccHObject* planeObj, double distance, double minpts, double radius);
ccHObject * DetectLineRansac(ccHObject * entity, double distance, double minpts, double radius);
ccHObject* CalcPlaneOutlines(ccHObject* planeObj, double alpha);
void ShrinkPlaneToOutline(ccHObject* planeObj, double alpha, double distance_epsilon);
void CreateIntersectionPoint(ccHObject * p1, ccHObject * p2);
ccHObject* PlaneFrameOptimization(ccHObject* planeObj, stocker::FrameOption option);
ccHObject* PlaneFrameLineGrow(ccHObject* planeObj, double alpha, double intersection, double min_area, bool skip_small_area, bool regularize);

bool FastPlanarTextureMapping(ccHObject * planeObj);

bool TextureMappingBuildings(ccHObject::Container buildings, ccHObject::Container cameras, stocker::IndexVector* task_indices = nullptr,
	double refine_length = 3, double sampling_grid = 0.5f, int max_view = 3, bool skip_nonexist = true);

bool TextureMappingPlanes(ccHObject::Container primObjs, stocker::IndexVector* task_indices = nullptr, 
	double refine_length = 3, double sampling_grid = -1.0f, int max_view = 3);

ccHObject * ConstrainedMesh(ccHObject * planeObj, int rare_pts = -1);

ccHObject::Container GenerateFootPrints_PP(ccHObject * prim_group, double ground);

ccHObject::Container GenerateBuildingFootPrints(ccHObject * prim_group, 
	double ground, double alpha,
	double max_intersection, double min_area,
	bool regularize, double reg_angle,
	bool scdt);

ccHObject::Container GenerateFootPrints(ccHObject * prim_group, double ground, double alpha, 
	double compo_distance, double compo_minpts,
	double max_intersection, double min_area);

ccHObject * LoD1FromFootPrint(ccHObject * buildingObj, bool flat = true);

ccHObject * LoD2FromFootPrint(ccHObject * entity);

ccHObject::Container PackPolygons(ccHObject::Container polygonEntites, int sample);

bool PackPlaneFrames(ccHObject * buildingObj, int max_iter, bool cap_hole, double ptsnum_ratio, double data_ratio,
	double ints_thre, double cluster_hori, double cluster_verti);

bool PackFootprints_PPP(ccHObject * buildingObj, int max_iter, bool cap_hole, double ptsnum_ratio, double data_ratio, double sharp_weight);

ccHObject* LoD2FromFootPrint_PPP(ccHObject * entity, int max_iter, bool cap_hole, bool fit_footprint, double ptsnum_ratio, double data_ratio,
	double ints_thre, double alpha, double min_area, double max_intersection,
	double cluster_hori, double cluster_verti);

bool PackFootprints_PPRepair(ccHObject * buildingObj);

//! settings.x - xybias, y - zbias, z - minPts
void GetPlanesInsideFootPrint(ccHObject * footprint, ccHObject * prim_group, CCVector3 settings, bool bVertical, bool clearExisting);

ccHObject::Container LoadMeshAsBlock(QString filename);

StBuilding * GetParentBuilding(ccHObject * obj);

ccPointCloud * GetPlaneCloud(ccHObject * planeObj);

std::vector<stocker::Outline3d> GetPlanarOutlines(ccHObject * entity, QString prefix);

bool SetGlobalShiftAndScale(ccHObject * obj);

void filterCameraByName(ccHObject* camera_group, QStringList name_list);

//////////////////////////////////////////////////////////////////////////
//! polyfit
ccHObject* PolyfitGenerateHypothesis(ccHObject* primitive_group, PolyFitObj* polyfit_obj);

void PolyfitComputeConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj);

void UpdateConfidence(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj);

ccHObject * PolyfitFaceSelection(ccHObject * hypothesis_group, PolyFitObj * polyfit_obj);

class PolyFitObj
{
public:
	PolyFitObj();
	~PolyFitObj();

	void clear();

	void initGenerator(ccHObject::Container planeObjs);

	void GenerateHypothesis();

	//! all the facets are calculated
	void ComputeConfidence();

	void FacetOptimization();

	void AutoFilterByDistanceAndFitting();

	void UpdateValidFacet(std::vector<stocker::String_String> valid);

	//! only update valid planes
	void UpdateConfidence(ccHObject::Container facetObjs);

	bool OutputResultToObjFile(BDBaseHObject* baseObj, std::string & file_path);
	
private:

	bool FindValidFacet(std::string name_plane, std::string name_facet);

public:
//	PointSet::Ptr			point_set_;
	HypothesisGenerator*	hypothesis_;
	Map::Ptr				hypothesis_mesh_;
	Map::Ptr				optimized_mesh_;
	std::vector<stocker::String_String> valid_group_facet_name;

	std::string building_name;

	double data_fitting;
	double model_coverage;
	double model_complexity;
	bool use_confidence;
	bool strict_intersection;
	double snap_intersection;
	bool auto_filter;
	enum POLYFIT_STUTAS
	{
		STT_prepared,
		STT_hypomesh,
		STT_confidence,
		STT_optimized,
	};
	POLYFIT_STUTAS status;	// 0-prepared, 1-hypomesh, 2-confidence, 3-optimized
};

#include <concurrent_vector.h>
#include <ppl.h>
#define conc_index concurr_index
#ifndef DISABLE_PARRALLEL_FOR
#define ConcPair(x) std::pair<size_t, x>
#define ConcVector(x) Concurrency::concurrent_vector<ConcPair(x)>
#define ConcParForBegin(x) Concurrency::parallel_for((size_t)0, (size_t)x, [&](size_t conc_index)
#define ConcParForEnd );
#define ConcPairObj(x) { conc_index, x }
#define ConcSort(x, v) sort(begin(v), end(v), [](ConcPair(x) _l, ConcPair(x) _r) {return _l.first < _r.first; });
#define GetConcObj(x) x.second
#else
#define ConcVector(x) std::vector<x>
#define ConcParForBegin(x) for (size_t conc_ind = 0; conc_ind < x; conc_ind++)
#define ConcParForEnd 
#define ConcPairObj(x) x
#define ConcSort(x, v)
#define GetConcObj(x) x
#endif // USE_PARRALEL_FOR

#endif