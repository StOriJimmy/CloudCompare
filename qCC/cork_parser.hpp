#pragma once
#include "mesh/corkMesh.h"
#include "ccPointCloud.h"
#include "ccMesh.h"
#include "bdr3DGeometryEditPanel.h"
#include <iostream>

struct CSGBoolOpParameters
{
	CSGBoolOpParameters()
		: operation(CSG_UNION)
		, corkA(0)
		, corkB(0)
		, meshesAreOk(false)
	{}

	CSG_OPERATION operation;
	CorkMesh* corkA;
	CorkMesh* corkB;
	QString nameA;
	QString nameB;
	bool meshesAreOk;
};

inline bool ToCorkMesh(const ccMesh* in, CorkMesh& out)
{
	if (!in || !in->getAssociatedCloud()) {
		return false;
	}

	ccGenericPointCloud* vertices = in->getAssociatedCloud();
	assert(vertices);

	unsigned triCount = in->size();
	unsigned vertCount = vertices ? vertices->size() : 0;

	std::vector<CorkMesh::Tri>& outTris = out.getTris();
	std::vector<CorkVertex>& outVerts = out.getVerts();
	try {
		outVerts.resize(vertCount);
		outTris.resize(triCount);
	}
	catch (const std::bad_alloc&) {
		return false;
	}

	if (outVerts.empty() || outTris.empty()) {
		return false;
	}

	//import triangle indexes
	{
		for (unsigned i = 0; i < triCount; i++)
		{
			const CCLib::VerticesIndexes* tsi = in->getTriangleVertIndexes(i);
			CorkTriangle corkTri;
			corkTri.a = tsi->i1;
			corkTri.b = tsi->i2;
			corkTri.c = tsi->i3;
			outTris[i].data = corkTri;
			//DGM: it seems that Cork doubles this information?!
			outTris[i].a = tsi->i1;
			outTris[i].b = tsi->i2;
			outTris[i].c = tsi->i3;
		}
	}

	//import vertices
	{
		for (unsigned i = 0; i < vertCount; i++)
		{
			const CCVector3* P = vertices->getPoint(i);
			outVerts[i].pos.x = static_cast<double>(P->x);
			outVerts[i].pos.y = static_cast<double>(P->y);
			outVerts[i].pos.z = static_cast<double>(P->z);
		}
	}

	return true;
}

inline ccMesh* FromCorkMesh(const CorkMesh& in)
{
	const std::vector<CorkMesh::Tri>& inTris = in.getTris();
	const std::vector<CorkVertex>& inVerts = in.getVerts();

	if (inTris.empty() || inVerts.empty()) {
		return 0;
	}

	unsigned triCount = static_cast<unsigned>(inTris.size());
	unsigned vertCount = static_cast<unsigned>(inVerts.size());

	ccPointCloud* vertices = new ccPointCloud("vertices");
	if (!vertices->reserve(vertCount)) {
		delete vertices;
		return 0;
	}

	ccMesh* mesh = new ccMesh(vertices);
	mesh->addChild(vertices);
	if (!mesh->reserve(triCount)) {
		delete mesh;
		return 0;
	}

	//import vertices
	{
		for (unsigned i = 0; i < vertCount; i++)
		{
			const CorkVertex& P = inVerts[i];
			CCVector3 Pout(static_cast<PointCoordinateType>(P.pos.x),
				static_cast<PointCoordinateType>(P.pos.y),
				static_cast<PointCoordinateType>(P.pos.z));
			vertices->addPoint(Pout);
		}
	}

	//import triangle indexes
	{
		for (unsigned i = 0; i < triCount; i++)
		{
			const CorkMesh::Tri& tri = inTris[i];
			mesh->addTriangle(tri.a, tri.b, tri.c);
		}
	}

	mesh->setVisible(true);
	vertices->setEnabled(false);

	return mesh;
}
