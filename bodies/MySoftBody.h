/*
 *  MySoftBody.h
 *  akiraOF23ofxBulletBasics
 *
 *  Created by 荳句恍 鮗ｻ雋ｴ on 10/02/12.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#pragma once

#include "ofMain.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "ofxBulletStaticUtil.h"

class MySoftBody {

public:
	MySoftBody(btBroadphaseInterface* m_broadphase,
			   btCollisionDispatcher* m_dispatcher,
			   btVector3 gravity);
	~MySoftBody(){};
	
	void remove(btSoftRigidDynamicsWorld* m_dynamicsWorld);
	
	void createRopeShape(btVector3 from, btVector3 len,
					int res, int fixed,
					int mass,
					ofVec4f color = ofVec4f(0.6, 0.6, 0.1, 0.5));
	void createEllipsoidShape(btVector3 center, btVector3 radius, int res);
	void createClothShape(ofVec3f clothShape[4],
						  int resolution, int fix);
	void createConvexHullShape(const btVector3* vertices, int nVerts);
	void createTriMeshShape(const btScalar* vertices, const int* triangles, int ntriangles)	;
	
	void render();
	void update();
	
	btSoftBody* getSoftBody();
	void setSoftBody(btSoftBody* _psb);
	ofVec3f getBodyCentroid();	
	vector<ofVec3f> getAllFacesAsVerts();
	vector<ofVec3f> getFaceAsVerts(int faceIdx);
	ofVec3f getFaceNormal(int faceIdx);
	float getFaceDistanceBetween(int face1Idx, int face2Idx);
	ofVec3f getFaceCentroid(int faceIdx);
	vector<int> sortFaceByDistance(int faceIdx);
	vector<int> sortFaceByPosition(ofVec3f pos);
	btSoftBodyWorldInfo getSoftWI();
	
	void pinchFace(int faceIdx, float force);
	void blowUp(float force);
	void shrink(float force);
	
	int ballSize;	
	// edge ball position
	ofVec3f ballPos;
	
	ofVec4f bodyColor;	
	

	btSoftBody* psb;
	btSoftBodyWorldInfo softBodyWI;
	
private:
	
			
};