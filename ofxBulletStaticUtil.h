#pragma once

/*
 *  ofxBulletStaticUtil.h
 *  akiraOF17Kemushi
 *
 *  Created by Makira on 10/09/16.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "ofMain.h"
#include "ofxVec3f.h"
#include "ofxVec4f.h"
#include "btVector3.h"


#define STATIC_BODY 0
#define DYNAMIC_BODY 1
#define KINEMATIC_BODY 2

class ofxBulletStaticUtil {
	
public:
	
	static inline btVector3 ofxVec3ToBtVec3(ofxVec3f in) {
		
		btVector3 ret;
		ret.setX(in.x);
		ret.setY(in.y);
		ret.setZ(in.z);
		return ret;
		
	}	
	
	static inline ofxVec3f btVec3ToOfxVec3(btVector3 in) {
		
		ofxVec3f ret;
		ret.x = in.getX();
		ret.y = in.getY();
		ret.z = in.getZ();
		return ret;
		
	}
	
	static inline btVector4 ofxVec4ToBtVec4(ofxVec4f in) {
	
		btVector4 ret;
		ret.setX(in.x);
		ret.setY(in.y);
		ret.setZ(in.z);
		ret.setW(in.w);
		return ret;
		
	}
	
	static inline double radToDeg(double radian) {
		
		double degree = 0;
		degree = radian * (180/PI);
		return degree;
		
	}	
	
	static inline double degToRad(double degree) {
		
		double radian = 0;
		radian = degree/(180/PI);
		return radian;
		
	}	
	
};