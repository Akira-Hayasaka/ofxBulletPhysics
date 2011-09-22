
#pragma once

#include "ofMain.h"
#include "ofxBulletStaticUtil.h"
#include "btBulletDynamicsCommon.h"
#include "MyKinematicMotionState.h"
#include "GL_ShapeDrawer.h"

class MyRigidBody {
	
public:
	
	MyRigidBody(int bt){ bodyType = bt; bodyColor = ofVec4f(0.1, 0.1, 0, 1); };
	~MyRigidBody(){};
	
	void						remove(btDynamicsWorld* m_dynamicsWorld);
	
	void						render(btDynamicsWorld* m_dynamicsWorld);
	
	void						createBoxShape(btTransform startTrans, btVector3 shape, int mass, 
											   ofVec4f color = ofVec4f(0.1, 0, 0.1, 0.5));	
	void						createSphereShape(btTransform startTrans, int _radius, int mass, 
												   ofVec4f color = ofVec4f(0, 0.1, 0.1, 0.5));	
	void						createCapsuleShape(btTransform startTrans, int radius, int height, int mass, 
												   ofVec4f color = ofVec4f(0, 0.1, 0.1, 0.5));
	
	void						translateBody(ofVec3f pos, ofVec3f rotDir, float degree);
	ofPoint						getBodyPos();
	ofVec3f					getBodyRotDegree();
	ofQuaternion				getBodyRotQuat();
	ofVec4f					getBodyColor();
	int							getSphereRadius(); // sphere only
	ofVec3f					getBoxSize(); // box only
	int							getBodyMass() { return bodyMass; };	
	vector<ofVec3f>			getVertsPos();
	
	btRigidBody*				getRigidBody(){ return psb; };	
	btConvexInternalShape*		getRigidShape() { return shape; };

	
	
		int							ID;
		float						deceleration;
		ofVec3f					destination;
		float						distance;
		float						currentRadius;
		ofVec3f					rotDir;
		int						age;
		bool						delAnimFlag;
		bool						delFlag;

		
protected:

	void						createRigidBody(int mass, const btTransform startTrans);		
	
	btRigidBody*				psb;
	btConvexInternalShape*		shape;
	GL_ShapeDrawer				drawer;
	
private:
	
	int							bodyMass;
	int							bodyType;
	int							sphereRadius; // sphere only
	btVector3					boxSize; // box only
	ofVec4f					bodyColor;

	
};