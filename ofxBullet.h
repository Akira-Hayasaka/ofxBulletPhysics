/*
 *  ofxBullet.h
 *  akiraOF17Kemushi
 *
 *  Created by Makira on 10/09/16.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#pragma once

#include "ofMain.h"
#include <btBulletDynamicsCommon.h>
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "bulletBaseApp.h"
#include "MySoftBody.h"
#include "MyRigidBody.h"
#include "Ragdoll.h"
#include "ofxBulletStaticUtil.h"
#include "LinearMath/btConvexHull.h"


class ofxBullet : public bulletBaseApp {
	
public:
	
	// "pure virtual" member function overrides.
	ofxBullet(){};
	~ofxBullet() { exitPhysics(); };
	
	void					initPhysics(){ initPhysics(ofVec3f(0,100,0), false); };
	void					initPhysics(ofVec3f gravity, bool _bEnableCollisionNotification = false);	
	void					exitPhysics();
	void					stepPhysicsSimulation(float framerate);	
	void					render();	
	void					swapBuffers(){};
	void					clientMoveAndDisplay(){};									
	void					updateModifierKeys(){};
	
	void					enableRayCastingMouseInteraction(ofCamera* _cam, ofVec3f camPosOffset = ofVec3f(0,0,0));
	bool					bMousePressed;
	void					mouseDragged(ofMouseEventArgs& event);
	void					mousePressed(ofMouseEventArgs& event);
	void					mouseReleased(ofMouseEventArgs& event);
	void					enableRayCastingTouchInteraction(ofCamera* _cam, ofVec3f camPosOffset = ofVec3f(0,0,0));
	bool					bTouched;
	void					touchDown(ofTouchEventArgs& touch);
	void					touchUp(ofTouchEventArgs& touch);
	void					touchMoved(ofTouchEventArgs& touch);
	ofVec3f				cameraPosiotionOffset;
	
	inline btDynamicsWorld* getWorld() { return m_dynamicsWorld; };

	// just make it a btSoftRigidDynamicsWorld please
	virtual const btSoftRigidDynamicsWorld*	getSoftDynamicsWorld() const { return (btSoftRigidDynamicsWorld*) m_dynamicsWorld; }
	virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld() { return (btSoftRigidDynamicsWorld*) m_dynamicsWorld; }	
	
	
	MyRigidBody*			createGround(ofVec3f startTrans, ofVec3f shape, int mass = 0,
										 ofVec4f color = ofVec4f(1.0, 1.0, 1.0, 0.5),
										 int bodyType = KINEMATIC_BODY);	
	MyRigidBody*			createBackWall(ofVec3f startTrans, ofVec3f shape, int mass = 0,
											ofVec4f color = ofVec4f(0.1, 0.1, 0, 0.5),
											int bodyType = KINEMATIC_BODY);	
	vector<MyRigidBody*>	createBoundingBox(ofVec3f centerPos = ofVec3f(ofGetWidth()/2, ofGetHeight()/2, 0),
											  ofVec3f dimention = ofVec3f(ofGetWidth(), ofGetHeight(), ofGetWidth()),
											  ofVec4f color = ofVec4f(0.3, 0.0, 3, 0.5), 
											  int mass = 0, int bodyType = KINEMATIC_BODY);
	MyRigidBody*			createStaticPlane(ofVec3f startTrans, ofVec3f shape, int mass = 0,
										 ofVec4f color = ofVec4f(1.0, 1.0, 1.0, 0.5),
										 int bodyType = KINEMATIC_BODY);	
	MyRigidBody*			createBox(ofVec3f startTrans, ofVec3f boxShape, int mass,
									  ofVec4f color = ofVec4f(0.1, 0.1, 0, 0.5),
									  int bodyType = DYNAMIC_BODY, 
									  ofVec3f rot = ofVec3f(0, 0, 0));
	MyRigidBody*			createSphere(ofVec3f startTrans, int radius, int mass,
										  ofVec4f color = ofVec4f(0, 0.1, 0.1, 0.5),
										  int bodyType = DYNAMIC_BODY);	
	MyRigidBody*			createCapsule(ofVec3f startTrans, int radius, int height, int mass,
										  ofVec4f color = ofVec4f(0, 0.1, 0.1, 0.5),
										 int bodyType = DYNAMIC_BODY);
	RagDoll*				createRagdoll(const btVector3& startOffset, int scale = 60);
	MySoftBody*				createRope(ofVec3f from, ofVec3f len,
									   int res, int fixed,
									   int mass,
									   ofVec4f color = ofVec4f(0.6, 0.6, 0.1, 0.5),
									   ofVec3f gravity = ofVec3f(0,10,0),
									   MyRigidBody* anchorTgt = NULL);
	MySoftBody*				createEllipsoid(ofVec3f gravity, ofVec3f center, ofVec3f radius, int res);
	MySoftBody*				createCloth(ofVec3f gravity, ofVec3f clothShape[4], int resolution, int fix);	
	MySoftBody*				createSoftConvexHull(ofVec3f gravity, const btVector3* vertices, int nVerts);	
	MySoftBody*				createSoftTriMesh(ofVec3f gravity, const btScalar* vertices, const int* triangles, int ntriangles);
	
	ofVec3f				getSceneCenter();
	void					setSceneCenter(ofVec3f _center);	
	
	inline void				enableCollisionNotifycation() { bEnableCollisionNotification = true; }
	inline void				disableCollisionNotifycation() { bEnableCollisionNotification = false; }
	vector<ofVec3f>		getCollisionPointsA();
	vector<ofVec3f>		getCollisionPointsB(); // how different A and B??
						
	ofCamera*				cam;
	MyRigidBody*			ground;
	MyRigidBody*			backWall;
	vector<MyRigidBody*>	boudingBox;
	vector<MyRigidBody*>	myRigidVec;			
	vector<MySoftBody*>		mySoftVec;
	vector<RagDoll*>		myRagdollVec;
	
	
protected:
	
	btVector3 getRayTo(int x,int y);
	btTypedConstraint*						m_pickConstraint;		// constraint for mouse picking
	btRigidBody*							pickedBody;						// for deactivation state		
	
	btSequentialImpulseConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration*		m_collisionConfiguration;
	btCollisionDispatcher*					m_dispatcher;
	btBroadphaseInterface*					m_broadphase;
	
	ofVec3f								sceneCenter;
	
	bool									bEnableCollisionNotification;
	vector<ofVec3f>						contactsA;
	vector<ofVec3f>						contactsB;
	
};
