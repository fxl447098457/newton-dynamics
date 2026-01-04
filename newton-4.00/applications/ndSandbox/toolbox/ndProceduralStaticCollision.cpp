/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

//#define D_TERRAIN_WIDTH			1024
//#define D_TERRAIN_HEIGHT			1024
//#define D_TERRAIN_WIDTH			512
//#define D_TERRAIN_HEIGHT			512
#define D_TERRAIN_WIDTH				256
#define D_TERRAIN_HEIGHT			256

#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f
#define D_TERRAIN_NOISE_GRID_SCALE  (1.0f / 500.0f)
//#define D_TERRAIN_NOISE_GRID_SCALE  1.0f / (ndFloat32 (D_TERRAIN_WIDTH) / 5)

#define D_TERRAIN_GRID_SIZE			2.0f
#define D_TERRAIN_TILE_SIZE			128
#define D_TERRAIN_ELEVATION_SCALE	32.0f

class ndProceduralTerrainShape : public ndShapeStaticProceduralMesh
{
	public:
	D_CLASS_REFLECTION(ndProceduralTerrainShape, ndShapeStaticProceduralMesh)

	ndProceduralTerrainShape()
		:ndShapeStaticProceduralMesh()
	{
		MakeNoiseHeightfield();

		ndFloat32 minY = 1.0e20f;
		ndFloat32 maxY = -1.0e20f;
		for (ndInt32 i = ndInt32(m_heightfield.GetCount()) - 1; i >= 0; --i)
		{
			minY = ndMin(minY, m_heightfield[i]);
			maxY = ndMax(maxY, m_heightfield[i]);
		}

		ndVector p0(ndFloat32 (0.0f), minY, ndFloat32(0.0f), ndFloat32(1.0f));
		ndVector p1(ndFloat32(D_TERRAIN_WIDTH * D_TERRAIN_GRID_SIZE), maxY, ndFloat32(D_TERRAIN_WIDTH * D_TERRAIN_GRID_SIZE), ndFloat32(1.0f));
		SetAABB(p0, p1);
	}

	void MakeNoiseHeightfield()
	{
		m_heightfield.SetCount(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);

		const ndInt32 octaves = D_TERRAIN_NOISE_OCTAVES;
		//const ndFloat32 cellSize = D_TERRAIN_GRID_SIZE;
		const ndFloat32 persistance = D_TERRAIN_NOISE_PERSISTANCE;
		const ndFloat32 noiseGridScale = D_TERRAIN_NOISE_GRID_SCALE;

		ndFloat32 minHeight = ndFloat32(1.0e10f);
		ndFloat32 maxHight = ndFloat32(-1.0e10f);
		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT; z++)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH; x++)
			{
				ndFloat32 noiseVal = BrownianMotion(octaves, persistance, noiseGridScale * ndFloat32(x), noiseGridScale * ndFloat32(z));
				m_heightfield[z * D_TERRAIN_WIDTH + x] = noiseVal;
				minHeight = ndMin(minHeight, noiseVal);
				maxHight = ndMax(maxHight, noiseVal);
			}
		}

		ndFloat32 scale = D_TERRAIN_ELEVATION_SCALE / (maxHight - minHeight);
		for (ndInt32 i = 0; i < m_heightfield.GetCapacity(); ++i)
		{
			ndFloat32 y = m_heightfield[i];
			y = scale * (y - minHeight);
			m_heightfield[i] = y;
		}
	}

	//virtual void DebugShape(const ndMatrix&, ndShapeDebugNotify& notify) const override
	virtual void DebugShape(const ndMatrix&, ndShapeDebugNotify&) const override
	{
		//ndDebugNotify& debugDraw = (ndDebugNotify&)notify;
		//ndBodyKinematic* const body = debugDraw.m_body;
		//
		//// this demo will iterate over the all the body contact pairs drawing the face in aabb.
		//ndArray<ndVector> vertex;
		//ndArray<ndInt32> faceList;
		//ndArray<ndInt32> faceMaterial;
		//ndArray<ndInt32> indexListList;
		//
		//ndMatrix myMatrix(body->GetMatrix());
		//ndBodyKinematic::ndContactMap& contactJoints = body->GetContactMap();
		//ndBodyKinematic::ndContactMap::Iterator it(contactJoints);
		//for (it.Begin(); it; it++)
		//{
		//	const ndContact* const contact = it.GetNode()->GetInfo();
		//	if (contact->IsActive())
		//	{
		//		ndBodyKinematic* const body0 = contact->GetBody0();
		//		ndShapeInstance& collision = body0->GetCollisionShape();
		//
		//		ndVector minP;
		//		ndVector maxP;
		//		ndMatrix matrix(body0->GetMatrix());
		//		//collision.CalculateAabb(matrix.Inverse(), minP, maxP);
		//		collision.CalculateAabb(matrix * myMatrix.OrthoInverse(), minP, maxP);
		//
		//		vertex.SetCount(0);
		//		faceList.SetCount(0);
		//		faceMaterial.SetCount(0);
		//		indexListList.SetCount(0);
		//		GetCollidingFaces(minP, maxP, vertex, faceList, faceMaterial, indexListList);
		//
		//		ndInt32 index = 0;
		//		ndVector color(50.0f / 255.0f, 100.0f / 255.0f, 200.0f / 255.0f, 1.0f);
		//
		//		for (ndInt32 i = 0; i < faceList.GetCount(); ++i)
		//		{
		//			ndVector points[32];
		//			ndInt32 vCount = faceList[i];
		//			for (ndInt32 j = 0; j < vCount; ++j)
		//			{
		//				ndInt32 k = indexListList[index + j];
		//				points[j] = myMatrix.TransformVector(vertex[k]);
		//			}
		//
		//			// I do not know why this is not rendering
		//			//RenderPolygon(debugDraw.m_manager, points, vCount, color);
		//			index += vCount;
		//		}
		//	}
		//}
	}

	virtual ndFloat32 RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32, const ndBody* const, ndContactPoint& contactOut) const override
	{
		//ndAssert(0);
		// TO DO 
		return 1.0f;
	}

	virtual void GetCollidingFaces(const ndVector& minBox, const ndVector& maxBox, ndArray<ndVector>& vertex, ndArray<ndInt32>& faceList, ndArray<ndInt32>& faceMaterial, ndArray<ndInt32>& indexList) const
	{
		//ndAssert(0);
		// TO DO 
		// generate the point cloud
		//ndVector p0(minBox.Scale(m_invGridSize).Floor());
		//ndVector p1(maxBox.Scale(m_invGridSize).Floor() + ndVector::m_one);
		//ndVector origin(p0.Scale(m_gridSize) & ndVector::m_triplexMask);
		//ndInt32 count_x = ndInt32(p1.m_x - p0.m_x);
		//ndInt32 count_z = ndInt32(p1.m_z - p0.m_z);
		
		//origin.m_y = 0.0f;
		//for (ndInt32 iz = 0; iz <= count_z; iz++)
		//{
		//	ndVector point(origin);
		//	for (ndInt32 ix = 0; ix <= count_x; ix++)
		//	{
		//		vertex.PushBack(point);
		//		point.m_x += m_gridSize;
		//	}
		//	origin.m_z += m_gridSize;
		//}
		//
		//// generate the face array
		//const ndInt32 stride = count_x + 1;
		//for (ndInt32 iz = 0; iz < count_z; iz++)
		//{
		//	for (ndInt32 ix = 0; ix < count_x; ix++)
		//	{
		//		faceList.PushBack(4);
		//		indexListList.PushBack((iz + 0) * stride + ix + 0);
		//		indexListList.PushBack((iz + 1) * stride + ix + 0);
		//		indexListList.PushBack((iz + 1) * stride + ix + 1);
		//		indexListList.PushBack((iz + 0) * stride + ix + 1);
		//
		//		faceMaterial.PushBack(0);
		//	}
		//}
	}

	virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const override
	{
		ndAssert(0);
		// TO DO 
		return 0;
		//hash = ndCRC64(&m_planeEquation[0], sizeof(ndVector), hash);
		//hash = ndCRC64(&m_gridSize, sizeof(ndFloat32), hash);
		//return hash;
	}

	ndArray<ndReal> m_heightfield;
};

class ndHeightfieldMesh : public ndRenderSceneNode
{
	public:
	ndHeightfieldMesh(ndRender* const render, const ndProceduralTerrainShape* const shape, const ndSharedPtr<ndRenderTexture>& texture, const ndMatrix& location)
		:ndRenderSceneNode(location)
	{
		ndMatrix uvMapping(ndGetIdentityMatrix());
		uvMapping[0][0] = 1.0f / 20.0f;
		uvMapping[1][1] = 1.0f / 20.0f;
		uvMapping[2][2] = 1.0f / 20.0f;

		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z += D_TERRAIN_TILE_SIZE)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
			{
				ndSharedPtr<ndShapeInstance> tileShape(BuildTile(shape, x, z));
				const ndShapeHeightfield* const heightfield = tileShape->GetShape()->GetAsShapeHeightfield();

				ndMatrix tileMatrix(ndGetIdentityMatrix());
				tileMatrix.m_posit += heightfield->GetLocation(0, 0);
				tileMatrix.m_posit.m_y = ndFloat32(0.0f);
				tileMatrix.m_posit.m_x += ndFloat32(x) * heightfield->GetWithScale();
				tileMatrix.m_posit.m_z += ndFloat32(z) * heightfield->GetHeightScale();

				ndSharedPtr<ndRenderSceneNode> tileNode(new ndRenderSceneNode(tileMatrix));
				AddChild(tileNode);

				ndRenderPrimitive::ndDescriptor descriptor(render);
				descriptor.m_collision = tileShape;
				descriptor.m_stretchMaping = false;
				descriptor.m_uvMatrix = uvMapping;
				descriptor.m_mapping = ndRenderPrimitive::m_box;
				ndRenderPrimitiveMaterial& material = descriptor.AddMaterial(texture);
				material.m_castShadows = false;

				ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));
				tileNode->SetPrimitive(mesh);
			}
		}
	}

	private:
	virtual void Render(const ndRender* const owner, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const override
	{
		// make a tiled rendered node.
		// the terrain is a array of tile subtable for colling,
		// but in this demo we are just rendering the map brute force
		ndRenderSceneNode::Render(owner, parentMatrix, renderMode);
	}

	ndSharedPtr<ndShapeInstance> BuildTile(const ndProceduralTerrainShape* const shape, ndInt32 x0, ndInt32 z0)
	{
		const ndInt32 xMax = ((x0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_WIDTH) ? D_TERRAIN_TILE_SIZE : D_TERRAIN_TILE_SIZE + 1;
		const ndInt32 zMax = ((z0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_HEIGHT) ? D_TERRAIN_TILE_SIZE : D_TERRAIN_TILE_SIZE + 1;

		// build a collision subtile
		const ndArray<ndReal>& heightMap = shape->m_heightfield;
		ndSharedPtr<ndShapeInstance> tileInstance(new ndShapeInstance(new ndShapeHeightfield(xMax, zMax,
			ndShapeHeightfield::m_invertedDiagonals,
			D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE)));

		ndArray<ndReal>& tileHeightMap = tileInstance->GetShape()->GetAsShapeHeightfield()->GetElevationMap();
		for (ndInt32 z = 0; z < zMax; z++)
		{
			for (ndInt32 x = 0; x < xMax; x++)
			{
				ndReal h = heightMap[(z0 + z) * D_TERRAIN_WIDTH + x0 + x];
				tileHeightMap[z * xMax + x] = h;
			}
		}
		tileInstance->GetShape()->GetAsShapeHeightfield()->UpdateElevationMapAabb();
		return tileInstance;
	}
};

ndSharedPtr<ndBody> BuildProceduralTerrain(ndDemoEntityManager* const scene, const char* const textureName, const ndMatrix& location)
{
	ndShapeInstance proceduralInstance(new ndProceduralTerrainShape());

	ndProceduralTerrainShape* const heighfield = (ndProceduralTerrainShape*)proceduralInstance.GetShape()->GetAsShapeStaticProceduralMesh();
	
	ndMatrix heighfieldLocation(location);
	ndVector origin (heighfield->GetObbOrigin());
	heighfieldLocation.m_posit.m_x -= origin.m_x;
	heighfieldLocation.m_posit.m_z -= origin.m_z;
	heighfieldLocation.m_posit.m_y -= origin.m_y;
	
	// add tile base sence node
	ndRender* const render = *scene->GetRenderer();
	ndSharedPtr<ndRenderTexture> texture(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));
	ndSharedPtr<ndRenderSceneNode> entity(new ndHeightfieldMesh(render, heighfield, texture, heighfieldLocation));
	
	// generate a rigibody and added to the scene and world
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(heighfieldLocation);
	body->GetAsBodyDynamic()->SetCollisionShape(proceduralInstance);
	
	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}