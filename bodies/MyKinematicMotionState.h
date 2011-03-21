#pragma once

/*
 *  MyKinematicMotionState.h
 *  akiraOF16BulletRope
 *
 *  Created by Makira on 10/09/15.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 *	http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=MotionStates
 *
 */
#include "btMotionState.h"

class MyKinematicMotionState : public btMotionState {
public:
    MyKinematicMotionState(const btTransform &initialpos) {
        mPos1 = initialpos;	
    }
	
    virtual ~ MyKinematicMotionState() {
    }
	
//    void setNode(Ogre::SceneNode *node) {
//        mVisibleobj = node;
//    }
	
    virtual void getWorldTransform(btTransform &worldTrans) const {
        worldTrans = mPos1;
    }
	
    void setKinematicPos(btTransform &currentPos) {
        mPos1 = currentPos;
    }
	
    virtual void setWorldTransform(const btTransform &worldTrans) {
    }
	
protected:
    btTransform mPos1;
};