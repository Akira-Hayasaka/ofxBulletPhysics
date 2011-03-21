#pragma once

/*
 *  bulletApp.h
 *  akiraOF16BulletRope
 *
 *  Created by Makira on 10/09/14.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "GlutStuff.h"
#include "GL_ShapeDrawer.h" 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAlignedObjectArray.h"

class	btCollisionShape;
class	btDynamicsWorld;
class	btRigidBody;
class	btTypedConstraint;



class bulletBaseApp
{
	void	displayProfileString(int xOffset,int yStart,char* message);
	class CProfileIterator* m_profileIterator;
	
protected:
#ifdef USE_BT_CLOCK
	btClock m_clock;
#endif //USE_BT_CLOCK
	
	///this is the most important class
	btDynamicsWorld*		m_dynamicsWorld;
	
	///constraint for mouse picking
	btTypedConstraint*		m_pickConstraint;
	
	btCollisionShape*	m_shootBoxShape;
	
	float	m_cameraDistance;
	int	m_debugMode;
	
	float m_ele;
	float m_azi;
	btVector3 m_cameraPosition;
	btVector3 m_cameraTargetPosition;//look at
	
	int	m_mouseOldX;
	int	m_mouseOldY;
	int	m_mouseButtons;
public:
	int	m_modifierKeys;
protected:
	
	float m_scaleBottom;
	float m_scaleFactor;
	btVector3 m_cameraUp;
	int	m_forwardAxis;
	
	int m_glutScreenWidth;
	int m_glutScreenHeight;
	
	int	m_ortho;
	
	float	m_ShootBoxInitialSpeed;
	
	bool	m_stepping;
	bool m_singleStep;
	bool m_idle;
	int m_lastKey;
	
	void showProfileInfo(int& xOffset,int& yStart, int yIncr);
	void renderscene(int pass);
	
	GL_ShapeDrawer*	m_shapeDrawer;
	bool			m_enableshadows;
	btVector3		m_sundirection;
	
public:
	
	bulletBaseApp();
	
	virtual ~bulletBaseApp();
	
	btDynamicsWorld*		getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}
	
	virtual	void initPhysics() = 0;
	
	virtual	void setDrawClusters(bool drawClusters)
	{
		
	}
	
	void overrideGLShapeDrawer (GL_ShapeDrawer* shapeDrawer);
	
	void setOrthographicProjection();
	void resetPerspectiveProjection();
	
	bool	setTexturing(bool enable) { return(m_shapeDrawer->enableTexture(enable)); }
	bool	setShadows(bool enable)	{ bool p=m_enableshadows;m_enableshadows=enable;return(p); }
	bool	getTexturing() const
	{
		return m_shapeDrawer->hasTextureEnabled();
	}
	bool	getShadows() const
	{
		return m_enableshadows;
	}
	
	
	int		getDebugMode()
	{
		return m_debugMode ;
	}
	
	void	setDebugMode(int mode);
	
	void	setAzi(float azi)
	{
		m_azi = azi;
	}
	
	void	setCameraUp(const btVector3& camUp)
	{
		m_cameraUp = camUp;
	}
	void	setCameraForwardAxis(int axis)
	{
		m_forwardAxis = axis;
	}
	
	void myinit();
	
	void toggleIdle();
	
	virtual void updateCamera();
	
	btVector3	getCameraPosition()
	{
		return m_cameraPosition;
	}
	btVector3	getCameraTargetPosition()
	{
		return m_cameraTargetPosition;
	}
	
	btScalar	getDeltaTimeMicroseconds()
	{
#ifdef USE_BT_CLOCK
		btScalar dt = m_clock.getTimeMicroseconds();
		m_clock.reset();
		return dt;
#else
		return btScalar(16666.);
#endif
	}
	
	///glut callbacks
	
	float	getCameraDistance();
	void	setCameraDistance(float dist);	
	void	moveAndDisplay();
	
	virtual void clientMoveAndDisplay() = 0;
	
	virtual void	clientResetScene();
	
	///Demo functions
	virtual void setShootBoxShape ();
	virtual void	shootBox(const btVector3& destination);
	
	
	btVector3	getRayTo(int x,int y);
	
	btRigidBody*	localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape);
	
	///callback methods by glut	
	
	virtual void keyboardCallback(unsigned char key, int x, int y);
	
	virtual void keyboardUpCallback(unsigned char key, int x, int y) {}
	
	virtual void specialKeyboard(int key, int x, int y){}
	
	virtual void specialKeyboardUp(int key, int x, int y){}
	
	virtual void reshape(int w, int h);
	
	virtual void mouseFunc(int button, int state, int x, int y);
	
	virtual void	mouseMotionFunc(int x,int y);
	
	virtual void displayCallback();
	
	virtual 	void renderme();
	
	virtual		void swapBuffers() = 0;
	
	virtual		void	updateModifierKeys() = 0;
	
	void stepLeft();
	void stepRight();
	void stepFront();
	void stepBack();
	void zoomIn();
	void zoomOut();
	
	bool	isIdle() const
	{
		return	m_idle;
	}
	
	void	setIdle(bool idle)
	{
		m_idle = idle;
	}
	
	btVector3 cameraPosition;
	btVector3 cameraTarget;	
	
};

