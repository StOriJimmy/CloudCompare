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
#include "ccHObjectCaster.h"

#ifdef USE_STOCKER
using namespace stocker;
#endif // USE_STOCKER
stocker::Contour3d GetPointsFromCloud(ccHObject* entity) {
	stocker::Contour3d points;
	if (!entity->isEnabled()) {
		return points;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);
	if (!cloud) return points;

	for (unsigned i = 0; i < cloud->size(); i++) {
		CCVector3 pt = *cloud->getPoint(i);
		points.push_back({ pt.x, pt.y, pt.z });
	}
	return points;
}

stocker::Polyline3d GetPolylineFromEntities(ccHObject::Container entities)
{
	stocker::Polyline3d polyline;
	for (auto & polyline_entity : entities) {
		ccPolyline* ccpolyline = ccHObjectCaster::ToPolyline(polyline_entity);
		unsigned lastvert = ccpolyline->isClosed() ? ccpolyline->size() : ccpolyline->size() - 1;
		for (size_t i = 0; i < lastvert; i++) {
			stocker::Seg3d seg;
			CCVector3 P0 = *(ccpolyline->getPoint(i));
			CCVector3 P1 = *(ccpolyline->getPoint((i + 1) % ccpolyline->size()));
			seg.P0() = stocker::parse_xyz(P0);
			seg.P1() = stocker::parse_xyz(P1);
			polyline.push_back(seg);
		}
	}
	return polyline;
}

ccHObject::Container GetEnabledObjFromGroup(ccHObject* entity, CC_CLASS_ENUM type)
{
	ccHObject::Container group, group_enabled;
	if (entity) {
		entity->filterChildren(group, true, type, true, entity->getDisplay());
		for (auto & gp : group) {
			if ((gp->getParent()) && (!gp->getParent()->isEnabled())) {
				continue;
			}
			if (gp->isEnabled())
				group_enabled.push_back(gp);
		}
	}
	return group_enabled;
}

void AddSegmentsAsChildVertices(ccHObject* entity, stocker::Polyline3d lines, QString name, ccColor::Rgb col)
{
	if (lines.empty()) {
		return;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(entity);

	ccPointCloud* line_vert = new ccPointCloud(name);
	int i(0);
	for (auto & ln : lines) {
		ccPolyline* cc_polyline = new ccPolyline(line_vert);
		cc_polyline->setDisplay(entity->getDisplay());
		cc_polyline->setColor(col);
		cc_polyline->showColors(true);
		char ln_name[32];
		sprintf(ln_name, "%s%d", name.toStdString().c_str(), i);
		cc_polyline->setName(ln_name);
		cc_polyline->setWidth(1);
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
}

void CalcPlaneIntersections(ccHObject::Container entity_planes, double distance)
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
	for (size_t i = 0; i < plane_units.size(); i++) {
		int num;
		sscanf(plane_units[i].GetName().Str().c_str(), "%d", &num);
		AddSegmentsAsChildVertices(entity_planes[num]->getParent(), ints_per_plane[i], "Intersection", ccColor::red);
	}
#endif // USE_STOCKER
}

void CalcPlaneBoundary(ccHObject* planeObj)
{

}

void CalcPlaneOutlines(ccHObject* planeObj)
{
#ifdef USE_STOCKER
	if (!planeObj->isEnabled()) return;

	stocker::Contour3d cur_plane_points = GetPointsFromCloud(planeObj->getParent());
	if (cur_plane_points.size() < 3) {
		return;
	}
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud(planeObj->getParent());

	//! get boundary
	vector<vector<stocker::Contour3d>> contours_points = stocker::GetPlanePointsOutline(cur_plane_points, 5, false, 1);
	for (vector<stocker::Contour3d> & component : contours_points) {
		for (stocker::Contour3d & st_contours : component) {
			ccPointCloud* line_vert = new ccPointCloud("Vertices");
			ccPolyline* cc_polyline = new ccPolyline(line_vert);
			cc_polyline->setDisplay(planeObj->getDisplay());
			cc_polyline->setColor(ccColor::green);
			cc_polyline->showColors(true);
			cc_polyline->addChild(line_vert);
			cc_polyline->setName("Outline");
			cc_polyline->setWidth(2);
			cc_polyline->setGlobalShift(cloud->getGlobalShift());
			cc_polyline->setGlobalScale(cloud->getGlobalScale());
			cc_polyline->reserve(static_cast<unsigned>(st_contours.size() + 1));
			for (auto & pt : st_contours) {
				line_vert->addPoint(CCVector3(pt.X(), pt.Y(), pt.Z()));
				cc_polyline->addPointIndex(line_vert->size() - 1);
			}
			cc_polyline->setClosed(true);
			cloud->addChild(cc_polyline);
		}
	}
#endif // USE_STOCKER

}

void PlaneFrameOptimization(ccHObject* planeObj)
{
#ifdef USE_STOCKER
	ccPlane* ccPlane = ccHObjectCaster::ToPlane(planeObj);
	if (!ccPlane) return;

	CCVector3 N; float constVal;
	ccPlane->getEquation(N, constVal);

	vcg::Plane3d vcgPlane;
	vcgPlane.SetDirection({ N.x, N.y, N.z });
	vcgPlane.SetOffset(constVal);


#endif
}
