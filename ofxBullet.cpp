/*
 *  ofxBullet.h
 *  akiraOF17Kemushi
 *
 *  Created by Makira on 10/09/16.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "ofxBullet.h"

void ofxBullet::initPhysics(ofxVec3f gravity, bool _bEnableCollisionNotification) {
	
	bEnableCollisionNotification = _bEnableCollisionNotification;
	
	ground = NULL;
	backWall = NULL;
	
	sceneCenter.set(ofGetWidth()/2, ofGetHeight()/2, 0);
	
    btVector3 worldAabbMin(-1000,-1000,-1000);
    btVector3 worldAabbMax(1000,1000,1000);
    int maxProxies = 1024;
    m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
	
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	
    m_solver = new btSequentialImpulseConstraintSolver;
	
    m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	
    m_dynamicsWorld->setGravity(ofxBulletStaticUtil::ofxVec3ToBtVec3(gravity));
	
	bMousePressed = false;
	bTouched = false;
}

void ofxBullet::exitPhysics() {
	
	// cleanup in the reverse order of creation/initialization	
	// remove the bodies from the dynamics world and delete them
	for (int i = m_dynamicsWorld->getNumCollisionObjects()-1; i >= 0 ; i--) {
		
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	for (int i = 0; i < myRigidVec.size(); i++) {
		
		btCollisionShape* shape = myRigidVec[i]->getRigidShape();
		delete shape;
		
	}
	
	// cannot delete softBody shape because softBody dont have shape
	for (int i = 0; i < mySoftVec.size(); i++) {
		
		btSoftBody* sb = mySoftVec[i]->getSoftBody();
		delete sb;
		
	}
	
	delete m_dynamicsWorld;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;
	delete m_collisionConfiguration;	
	
}


void ofxBullet::enableRayCastingMouseInteraction(ofxCamera* _cam, ofxVec3f camPosOffset) {
	
	cam = _cam;
	cameraPosiotionOffset = camPosOffset;
	ofAddListener(ofEvents.mousePressed, this, &ofxBullet::mousePressed);
	ofAddListener(ofEvents.mouseReleased, this, &ofxBullet::mouseReleased);
	ofAddListener(ofEvents.mouseDragged, this, &ofxBullet::mouseDragged);
	
}

void ofxBullet::enableRayCastingTouchInteraction(ofxCamera* _cam, ofxVec3f camPosOffset) {
	
	cam = _cam;
	cameraPosiotionOffset = camPosOffset;	
	ofAddListener(ofEvents.touchDown, this, &ofxBullet::touchDown);
	ofAddListener(ofEvents.touchUp, this, &ofxBullet::touchUp);
	ofAddListener(ofEvents.touchMoved, this, &ofxBullet::touchMoved);	
}

//--------------------------------------------------------------
/*
 #define GLUT_LEFT_BUTTON		0
 #define GLUT_MIDDLE_BUTTON		1
 #define GLUT_RIGHT_BUTTON		2
 #define GLUT_DOWN			0
 #define GLUT_UP				1
 */
btScalar mousePickClamp = 30.f;
btVector3 hitPos(-1,-1,-1);
float oldPickingDist  = 0.f;
btVector3 oldPickingPos;
void ofxBullet::mousePressed(ofMouseEventArgs& event){
	
	int x = event.x;
	int y = event.y;
	int button = event.button;
	
	if (button == 0) {
		if (getWorld()) {
			
			btVector3 rayTo = getRayTo(x,y);
			ofxVec3f campos = cam->getPosition();
			btVector3 rayFrom = btVector3(campos.x, campos.y, campos.z);
			
			btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
			getWorld()->rayTest(rayFrom,rayTo,rayCallback);
			if (rayCallback.hasHit()) {
				btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
				if (body) {
					//other exclusions?
					if (!(body->isStaticObject() || body->isKinematicObject())) {
						
						pickedBody = body;
						pickedBody->setActivationState(DISABLE_DEACTIVATION);
						
						
						btVector3 pickPos = rayCallback.m_hitPointWorld;
						printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
						
						
						btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
						
						btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
						p2p->m_setting.m_impulseClamp = oldPickingDist;
						
						getWorld()->addConstraint(p2p);
						m_pickConstraint = p2p;
						
						//save mouse position for dragging
						oldPickingPos = rayTo;
						hitPos = pickPos;
						
						oldPickingDist  = (pickPos-rayFrom).length();
						
						//very weak constraint for picking
						p2p->m_setting.m_tau = 0.1f;
						
						bMousePressed = true;
					}
				}
			}	
		}
	}
}
//--------------------------------------------------------------
void ofxBullet::mouseReleased(ofMouseEventArgs& event){
	
	int x = event.x;
	int y = event.y;
	int button = event.button;	
	
	if (button == 0) {
		if (m_pickConstraint && getWorld() && bMousePressed) {
			getWorld()->removeConstraint(m_pickConstraint);
			delete m_pickConstraint;
			m_pickConstraint = 0;
			pickedBody->forceActivationState(ACTIVE_TAG);
			pickedBody->setDeactivationTime( 0.f );
			pickedBody = 0;
			
			bMousePressed = false;
		}
	}
}
//--------------------------------------------------------------
void ofxBullet::mouseDragged(ofMouseEventArgs& event){
	
	int x = event.x;
	int y = event.y;
	int button = event.button;	
	
	if (m_pickConstraint) {
		//move the constraint pivot
		btPoint2PointConstraint* p2p = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
		if (p2p) {
			//keep it at the same picking distance
			
			btVector3 newRayTo = getRayTo(x,y);
			btVector3 rayFrom;
			btVector3 oldPivotInB = p2p->getPivotInB();
			btVector3 newPivotB;
			
			ofxVec3f campos = cam->getPosition();
			rayFrom = btVector3(campos.x, campos.y, campos.z);
			btVector3 dir = newRayTo-rayFrom;
			dir.normalize();
			dir *= oldPickingDist;
			
			newPivotB = rayFrom + dir;
			
			p2p->setPivotB(newPivotB);
		}
		
	}
}

//--------------------------------------------------------------
void ofxBullet::touchDown(ofTouchEventArgs& event){
	
	int x = event.x;
	int y = event.y;
//	int button = event.button;
	
//	if (button == 0) {
		if (getWorld()) {
			
			btVector3 rayTo = getRayTo(x,y);
			ofxVec3f campos = cam->getPosition()+cameraPosiotionOffset;
			btVector3 rayFrom = btVector3(campos.x, campos.y, campos.z);
			
			btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
			getWorld()->rayTest(rayFrom,rayTo,rayCallback);
			if (rayCallback.hasHit()) {
				btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
				if (body) {
					//other exclusions?
					if (!(body->isStaticObject() || body->isKinematicObject())) {
						
						pickedBody = body;
						pickedBody->setActivationState(DISABLE_DEACTIVATION);
						
						
						btVector3 pickPos = rayCallback.m_hitPointWorld;
						printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
						
						
						btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
						
						btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
						p2p->m_setting.m_impulseClamp = oldPickingDist;
						
						getWorld()->addConstraint(p2p);
						m_pickConstraint = p2p;
						
						//save mouse position for dragging
						oldPickingPos = rayTo;
						hitPos = pickPos;
						
						oldPickingDist  = (pickPos-rayFrom).length();
						
						//very weak constraint for picking
						p2p->m_setting.m_tau = 0.1f;
						
						bTouched = true;
					}
				}
			}	
		}
//	}
}
//--------------------------------------------------------------
void ofxBullet::touchUp(ofTouchEventArgs& event){
	
	int x = event.x;
	int y = event.y;
//	int button = event.button;	
	
//	if (button == 0) {
		if (m_pickConstraint && getWorld() && bTouched) {
			getWorld()->removeConstraint(m_pickConstraint);
			delete m_pickConstraint;
			//printf("removed constraint %i",gPickingConstraintId);
			m_pickConstraint = 0;
			pickedBody->forceActivationState(ACTIVE_TAG);
			pickedBody->setDeactivationTime( 0.f );
			pickedBody = 0;
			
			bTouched = false;
		}
//	}
}
//--------------------------------------------------------------
void ofxBullet::touchMoved(ofTouchEventArgs& event){
	
	int x = event.x;
	int y = event.y;
//	int button = event.button;	
	
	if (m_pickConstraint) {
		//move the constraint pivot
		btPoint2PointConstraint* p2p = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
		if (p2p) {
			//keep it at the same picking distance
			
			btVector3 newRayTo = getRayTo(x,y);
			btVector3 rayFrom;
			btVector3 oldPivotInB = p2p->getPivotInB();
			btVector3 newPivotB;
			
			ofxVec3f campos = cam->getPosition()+cameraPosiotionOffset;
			rayFrom = btVector3(campos.x, campos.y, campos.z);
			btVector3 dir = newRayTo-rayFrom;
			dir.normalize();
			dir *= oldPickingDist;
			
			newPivotB = rayFrom + dir;
			
			p2p->setPivotB(newPivotB);
		}
		
	}
}



MyRigidBody* ofxBullet::createGround(ofxVec3f startTrans, ofxVec3f shape, int mass,
									  ofxVec4f color,
									  int bodyType) {

	btTransform startTransform;
	startTransform.setIdentity();
//	startTransform.setRotation(btQuaternion(btVector3(0.0,0.0,1.0), ofxBulletStaticUtil::degToRad(20)));
	startTransform.setOrigin(btVector3(ofxBulletStaticUtil::ofxVec3ToBtVec3(startTrans)));
	
	MyRigidBody* mrb = new MyRigidBody(bodyType);
	
	mrb->createBoxShape(startTransform, ofxBulletStaticUtil::ofxVec3ToBtVec3(shape), mass, color);
	
	ground = mrb;
	m_dynamicsWorld->addRigidBody(mrb->getRigidBody());	
	
	return mrb;

}

MyRigidBody* ofxBullet::createBackWall(ofxVec3f startTrans, ofxVec3f shape, int mass,
										 ofxVec4f color,
										 int bodyType) {
	
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(ofxBulletStaticUtil::ofxVec3ToBtVec3(startTrans)));

	btQuaternion q;
	q.setRotation(btVector3(1,0,0), ofxBulletStaticUtil::degToRad(90));
	startTransform.setRotation(q);	
	
	MyRigidBody* mrb = new MyRigidBody(bodyType);
	
	mrb->createBoxShape(startTransform, ofxBulletStaticUtil::ofxVec3ToBtVec3(shape), mass, color);
	
	backWall = mrb;
	m_dynamicsWorld->addRigidBody(mrb->getRigidBody());	
	
	return mrb;	
	
}

vector<MyRigidBody*> ofxBullet::createBoundingBox(ofxVec3f centerPos, ofxVec3f dimention,
												  ofxVec4f color, int mass, int bodyType) {
	
	// gournd
	btTransform groundTrans;
	groundTrans.setIdentity();
	ofxVec3f groundVec = ofxVec3f(centerPos.x, centerPos.y+dimention.y, centerPos.z);
	btVector3 groundBVec = ofxBulletStaticUtil::ofxVec3ToBtVec3(groundVec);
	groundTrans.setOrigin(groundBVec);
	MyRigidBody* ground = new MyRigidBody(bodyType);
	ofxVec3f groundShape = ofxVec3f(1500,0,1600);
	btVector3 groundBShape = ofxBulletStaticUtil::ofxVec3ToBtVec3(groundShape);
	ground->createBoxShape(groundTrans, groundBShape, mass, color);
	m_dynamicsWorld->addRigidBody(ground->getRigidBody());	
	boudingBox.push_back(ground);
	
	// left
	
	// right
	
	// front							  
	
	// back
	
	// top
	
	return boudingBox;
}

MyRigidBody* ofxBullet::createStaticPlane(ofxVec3f startTrans, ofxVec3f shape, int mass,
									 ofxVec4f color,
									 int bodyType) {
	
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(ofxBulletStaticUtil::ofxVec3ToBtVec3(startTrans)));
	
	MyRigidBody* mrb = new MyRigidBody(bodyType);
	
	mrb->createBoxShape(startTransform, ofxBulletStaticUtil::ofxVec3ToBtVec3(shape), mass, color);
	
	myRigidVec.push_back(mrb);
	m_dynamicsWorld->addRigidBody(mrb->getRigidBody());	
	
	return mrb;
	
}

MyRigidBody* ofxBullet::createBox(ofxVec3f startTrans, ofxVec3f boxShape, int mass,
								  ofxVec4f color,
								  int bodyType, ofxVec3f rot) {

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setRotation(btQuaternion(btVector3(1.0,0.0,0.0), ofxBulletStaticUtil::degToRad(rot.x)));	
	startTransform.setRotation(btQuaternion(btVector3(0.0,1.0,0.0), ofxBulletStaticUtil::degToRad(rot.y)));	
	startTransform.setRotation(btQuaternion(btVector3(0.0,0.0,1.0), ofxBulletStaticUtil::degToRad(rot.z)));		
	startTransform.setOrigin(btVector3(ofxBulletStaticUtil::ofxVec3ToBtVec3(startTrans)));
	
	MyRigidBody* mrb = new MyRigidBody(bodyType);
	
	mrb->createBoxShape(startTransform, ofxBulletStaticUtil::ofxVec3ToBtVec3(boxShape), mass, color);
	
	myRigidVec.push_back(mrb);
	m_dynamicsWorld->addRigidBody(mrb->getRigidBody());	

	return mrb;
	
}

MyRigidBody* ofxBullet::createSphere(ofxVec3f startTrans, int radius, int mass,
									  ofxVec4f color, 
									  int bodyType) {
	
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(ofxBulletStaticUtil::ofxVec3ToBtVec3(startTrans)));
	
	MyRigidBody* mrb = new MyRigidBody(bodyType);
	
	mrb->createSphereShape(startTransform, radius, mass, color);
	
	myRigidVec.push_back(mrb);

	m_dynamicsWorld->addRigidBody(mrb->getRigidBody());	
//	m_dynamicsWorld->addCollisionObject(mrb->getRigidBody(), 0, 0);
	
	return mrb;
}

MyRigidBody* ofxBullet::createCapsule(ofxVec3f startTrans, int radius, int height, int mass,
									  ofxVec4f color, 
									  int bodyType) {
	
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(ofxBulletStaticUtil::ofxVec3ToBtVec3(startTrans)));
	
	MyRigidBody* mrb = new MyRigidBody(bodyType);
	
	mrb->createCapsuleShape(startTransform, radius, height, mass, color);
	
	myRigidVec.push_back(mrb);
	m_dynamicsWorld->addRigidBody(mrb->getRigidBody());	
	
	return mrb;
}

RagDoll* ofxBullet::createRagdoll(const btVector3& startOffset, int scale) {
	
	RagDoll* ragDoll = new RagDoll(m_dynamicsWorld, startOffset, scale);
	myRagdollVec.push_back(ragDoll);
	return ragDoll;
	
}

MySoftBody* ofxBullet::createRope(ofxVec3f from, ofxVec3f len,
								  int res, int fixed,
								  int mass, 
								  ofxVec4f color,
								  ofxVec3f gravity,
								  MyRigidBody* ancchorTgt) {
	
//	MySoftBody* msb = new MySoftBody(m_broadphase, m_dispatcher,
//									 ofxBulletStaticUtil::ofxVec3ToBtVec3(gravity));
//	
//	msb->createRopeShape(ofxBulletStaticUtil::ofxVec3ToBtVec3(from), 
//						 ofxBulletStaticUtil::ofxVec3ToBtVec3(len), 
//						 res, fixed,
//						 mass, color);
//	
//	mySoftVec.push_back(msb);
//	getSoftDynamicsWorld()->addSoftBody(msb->getSoftBody());
//	
//	if (ancchorTgt) {
//		msb->getSoftBody()->appendAnchor(0, ancchorTgt->getRigidBody());		
//	}
//	
//	return msb;
	
}

MySoftBody* ofxBullet::createEllipsoid(ofxVec3f gravity, ofxVec3f center, ofxVec3f radius, int res) {
	
	MySoftBody* msb = new MySoftBody(m_broadphase, m_dispatcher, btVector3(gravity.x,gravity.y,gravity.z));
	msb->createEllipsoidShape(btVector3(center.x, center.y, center.z),
							  btVector3(radius.x, radius.y, radius.z),
							  res);
	mySoftVec.push_back(msb);
	getSoftDynamicsWorld()->addSoftBody(msb->getSoftBody());
	
	return msb;
	
}

MySoftBody* ofxBullet::createSoftConvexHull(ofxVec3f gravity, const btVector3* vertices, int nVerts) {
	
	MySoftBody* msb = new MySoftBody(m_broadphase, m_dispatcher, btVector3(gravity.x,gravity.y,gravity.z));
	
	msb->createConvexHullShape(vertices, nVerts);
	
	mySoftVec.push_back(msb);
	getSoftDynamicsWorld()->addSoftBody(msb->getSoftBody());
	
	return msb;
}

MySoftBody* ofxBullet::createSoftTriMesh(ofxVec3f gravity, const btScalar* vertices, const int* triangles, int ntriangles) {
	
	MySoftBody* msb = new MySoftBody(m_broadphase, m_dispatcher, btVector3(gravity.x,gravity.y,gravity.z));
	
	msb->createTriMeshShape(vertices, triangles, ntriangles);
	
	//mySoftVec.push_back(msb);
	getSoftDynamicsWorld()->addSoftBody(msb->getSoftBody());
	
	return msb;
}

MySoftBody* ofxBullet::createCloth(ofxVec3f gravity, ofxVec3f clothShape[4], int resolution, int fix) {
	
	MySoftBody* msb = new MySoftBody(m_broadphase, m_dispatcher, btVector3(gravity.x,gravity.y,gravity.z));
	msb->createClothShape(clothShape, resolution, fix);
	mySoftVec.push_back(msb);
	getSoftDynamicsWorld()->addSoftBody(msb->getSoftBody());	
	
	return msb;
	
}

// step the world
void ofxBullet::stepPhysicsSimulation(float framerate) {
	
	m_dynamicsWorld->stepSimulation(1.0f/framerate, 10);
	
	
	if (bEnableCollisionNotification) {
		contactsA.clear();
		contactsB.clear();
		int numManifolds = m_dynamicsWorld->getDispatcher()->getNumManifolds();
		for (int i = 0; i<numManifolds; i++) {
			btPersistentManifold* contactManifold =  m_dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
			
			int numContacts = contactManifold->getNumContacts();
			for (int j = 0; j<numContacts; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (pt.getDistance() < 0.f) {
					const btVector3& ptA = pt.getPositionWorldOnA();
					const btVector3& ptB = pt.getPositionWorldOnB();
					const btVector3& normalOnB = pt.m_normalWorldOnB;
					
					contactsA.push_back(ofxBulletStaticUtil::btVec3ToOfxVec3(ptA));
					contactsB.push_back(ofxBulletStaticUtil::btVec3ToOfxVec3(ptB));
				}
			}
		}	
	}
}

// render
void ofxBullet::render() {

	if (ground) 
		 ground->render(m_dynamicsWorld);
	
	if (backWall) 
		backWall->render(m_dynamicsWorld);
	
	
	for (int i = 0; i < myRigidVec.size(); i++) {
		myRigidVec[i]->render(m_dynamicsWorld);
		
	}
	
	for (int i = 0; i < myRagdollVec.size(); i++) {
		myRagdollVec[i]->render(m_dynamicsWorld);
		
	}
	
	
//	for(int i=0; i < mySoftVec.size(); i++){
//		mySoftVec[i]->render();
//	}
		

	ofSetColor(255, 255, 255);
	glDisable(GL_DEPTH_TEST);

}


btVector3 ofxBullet::getRayTo(int x, int y) {
	
	// why upside-down??
	//y = ofMap(y, 0, ofGetHeight(), ofGetHeight(), 0);
	
	// oF adjustment
	float adjustx = fabs(ofGetWidth()/2-x);
	float adjusty = fabs(ofGetHeight()/2-y);
	adjustx = ofMap(adjustx, 0, ofGetWidth()/2, 0, ofGetWidth()/5);
	adjusty = ofMap(adjusty, 0, ofGetHeight()/2, 0, ofGetHeight()/5);
	if (x > ofGetWidth()/2) adjustx = -adjustx;
	if (y > ofGetHeight()/2) adjusty = -adjusty;
	x += adjustx; y+= adjusty;
	
	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = 2.0 * atanf (tanFov);
	
	ofxVec3f campos = cam->getPosition()+cameraPosiotionOffset;
	btVector3 rayFrom = btVector3(campos.x, campos.y, campos.z);
	ofxVec3f dir = cam->getDir();
	btVector3 rayForward = btVector3(dir.x, dir.y, dir.z);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward *= farPlane;
	
	btVector3 rightOffset;
	ofxVec3f camup = cam->getUp();
	btVector3 vertical = btVector3(camup.x, camup.y, camup.z);
	
	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();
	vertical = -vertical;
	
	float tanfov = tanf(0.5f*fov);
	
	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;
	
	btScalar aspect;
	
	if (ofGetWidth() > ofGetHeight()) {
		aspect = ofGetWidth() / (btScalar)ofGetHeight();
		hor*=aspect;
	} else 	{
		aspect = ofGetHeight() / (btScalar)ofGetWidth();
		vertical*=aspect;
	}
	
	
	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/float(ofGetWidth());
	btVector3 dVert = vertical * 1.f/float(ofGetHeight());
	
	
	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += x * dHor;
	rayTo -= y * dVert;
	return rayTo;
}

vector<ofxVec3f> ofxBullet::getCollisionPointsA() {
	return contactsA;
}

vector<ofxVec3f> ofxBullet::getCollisionPointsB() {
	return contactsB;
}
