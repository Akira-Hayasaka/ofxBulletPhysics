#include "MySoftBody.h"

/*
 
 btScalar				m_kLST;			// Linear stiffness coefficient [0,1]
 btScalar				m_kAST;			// Area/Angular stiffness coefficient [0,1]
 btScalar				m_kVST;			// Volume stiffness coefficient [0,1]
 int						m_flags;		// Flags
 
 eAeroModel::_			aeromodel;		// Aerodynamic model (default: V_Point)
 btScalar				kVCF;			// Velocities correction factor (Baumgarte)
 btScalar				kDP;			// Damping coefficient [0,1]
 btScalar				kDG;			// Drag coefficient [0,+inf]
 btScalar				kLF;			// Lift coefficient [0,+inf]
 btScalar				kPR;			// Pressure coefficient [-inf,+inf]
 btScalar				kVC;			// Volume conversation coefficient [0,+inf]
 btScalar				kDF;			// Dynamic friction coefficient [0,1]
 btScalar				kMT;			// Pose matching coefficient [0,1]		
 btScalar				kCHR;			// Rigid contacts hardness [0,1]
 btScalar				kKHR;			// Kinetic contacts hardness [0,1]
 btScalar				kSHR;			// Soft contacts hardness [0,1]
 btScalar				kAHR;			// Anchors hardness [0,1]
 btScalar				kSRHR_CL;		// Soft vs rigid hardness [0,1] (cluster only)
 btScalar				kSKHR_CL;		// Soft vs kinetic hardness [0,1] (cluster only)
 btScalar				kSSHR_CL;		// Soft vs soft hardness [0,1] (cluster only)
 btScalar				kSR_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
 btScalar				kSK_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
 btScalar				kSS_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
 btScalar				maxvolume;		// Maximum volume ratio for pose
 btScalar				timescale;		// Time scale
 int						viterations;	// Velocities solver iterations
 int						piterations;	// Positions solver iterations
 int						diterations;	// Drift solver iterations
 int						citerations;	// Cluster solver iterations
 int						collisions;		// Collisions flags
 tVSolverArray			m_vsequence;	// Velocity solvers sequence
 tPSolverArray			m_psequence;	// Position solvers sequence
 tPSolverArray			m_dsequence;	// Drift solvers sequence		 
 
*/


MySoftBody::MySoftBody(btBroadphaseInterface* m_broadphase,
					   btCollisionDispatcher* m_dispatcher,
					   btVector3 gravity) {

    softBodyWI.m_broadphase = m_broadphase;
    softBodyWI.m_dispatcher = m_dispatcher;
	softBodyWI.m_gravity.setValue(gravity.x(), gravity.y(), gravity.z()); // too much gravity will penetrate ground	
	softBodyWI.air_density = 10;
	softBodyWI.water_density = 10;
	softBodyWI.water_offset = 0;
	softBodyWI.water_normal	= btVector3(0,0,0);
	softBodyWI.m_sparsesdf.Initialize();
	//softBodyWI.m_sparsesdf.GarbageCollect();

}

void MySoftBody::remove(btSoftRigidDynamicsWorld* m_dynamicsWorld) {
		
	m_dynamicsWorld->removeSoftBody(psb);
	delete psb;	
	
}

btSoftBody * MySoftBody::getSoftBody(){
	return psb;
}	
void MySoftBody::setSoftBody(btSoftBody* _psb) {
	psb = _psb;
}
btSoftBodyWorldInfo MySoftBody::getSoftWI() {
	return softBodyWI;
}

void MySoftBody::createRopeShape(btVector3 from, btVector3 len,
							int res, int fixed,
							int mass,
							ofxVec4f color) {
	
	bodyColor = color;
	
	psb = btSoftBodyHelpers::CreateRope(softBodyWI, 
										from,
										//to+len,
										from+len,
										res, fixed);
	psb->setTotalMass(mass);
	psb->setPose(true,true);	
	psb->m_materials[0]->m_kLST   =   1;
	psb->m_materials[0]->m_kAST   =   1;
	psb->m_materials[0]->m_kVST   =   1;
	psb->m_cfg.kMT = 0.002;
	psb->m_cfg.kDP = 0.05;
	psb->m_cfg.kKHR = 1; //*
	psb->m_cfg.kDF = 1;	//*
	
}

void MySoftBody::createEllipsoidShape(btVector3 center, btVector3 radius, int res) {
	
    psb = btSoftBodyHelpers::CreateEllipsoid(softBodyWI,
											 center,
											 radius,
											 res);
//	psb->m_materials[0]->m_kLST   =   0.1; // Linear stiffness coefficient [0,1]
//	psb->m_materials[0]->m_kAST   =   0.1; // Area/Angular stiffness coefficient [0,1]
//	psb->m_materials[0]->m_kVST   =   0.1; // Volume stiffness coefficient [0,1]
//	
//	// 粘り気
//	psb->m_cfg.kDP            =   0.1;//0.001; 
//	// 地面との間で巻き込まれたのが回復する力
//	psb->m_cfg.kDG			  =   1;
//	// わからない。何かへの抵抗。
//	psb->m_cfg.kLF			  =   0;
//	// ???
//	psb->m_cfg.kDF            =   1;//1;
//	// 元に戻る力、弾力
//	psb->m_cfg.kPR            =   500;//2500;
//	// ぐちゃぐちゃになる度。中身の量。元に戻らない度。
//	psb->m_cfg.kVC            =   0;//2500;
//	// 地面と滑るかどうか
//	psb->m_cfg.kDF            =   0;//1;
//	// ???
//	psb->m_cfg.kMT            =   0;//1;	
//	// ???
//	psb->m_cfg.kCHR            =   1;//1;	
//	// ???
//	psb->m_cfg.kSHR            =   1;//1;
//	
//	psb->setTotalMass(1,true);
//	psb->setPose( false, false );
	
	psb->m_materials[0]->m_kLST   =   0.1;
	psb->m_materials[0]->m_kAST   =   0.1; // Area/Angular stiffness coefficient [0,1]
	psb->m_materials[0]->m_kVST   =   0.1; // Volume stiffness coefficient [0,1]	
	
	// 粘り気
	psb->m_cfg.kDP            =   0.1;//0.001; 
	// 地面との間で巻き込まれたのが回復する力
	psb->m_cfg.kDG			  =   0;
	// わからない。何かへの抵抗。
	psb->m_cfg.kLF			  =   0;
	// ???
	psb->m_cfg.kDF            =   1;//1;
	// 元に戻る力、弾力
	psb->m_cfg.kPR            =   2250;//2500;
	// ぐちゃぐちゃになる度。中身の量。元に戻らない度。
	psb->m_cfg.kVC            =   0;//2500;
	// 地面と滑るかどうか
	psb->m_cfg.kDF            =   1;//1;
	// ???
	psb->m_cfg.kMT            =   0;//1;	
	// ???
	psb->m_cfg.kCHR            =   1;//1;	
	// ???
	psb->m_cfg.kSHR            =   1;//1;
	
	psb->setTotalMass(20,true);
	psb->setPose( false, false );	
	
}

void MySoftBody::createConvexHullShape(const btVector3* vertices, int nVerts) {
	
	 psb = btSoftBodyHelpers::CreateFromConvexHull(softBodyWI, vertices, nVerts);											
	
	psb->m_materials[0]->m_kLST   =   0.1;
	psb->m_materials[0]->m_kAST   =   0.1; // Area/Angular stiffness coefficient [0,1]
	psb->m_materials[0]->m_kVST   =   0.1; // Volume stiffness coefficient [0,1]	
	
	// 粘り気
	psb->m_cfg.kDP            =   0.1;//0.001; 
	// 地面との間で巻き込まれたのが回復する力
	psb->m_cfg.kDG			  =   0;
	// わからない。何かへの抵抗。
	psb->m_cfg.kLF			  =   0;
	// ???
	psb->m_cfg.kDF            =   1;//1;
	// 元に戻る力、弾力
	psb->m_cfg.kPR            =   2500;//2500;
	// ぐちゃぐちゃになる度。中身の量。元に戻らない度。
	psb->m_cfg.kVC            =   0;//2500;
	// 地面と滑るかどうか
	psb->m_cfg.kDF            =   1;//1;
	// ???
	psb->m_cfg.kMT            =   0;//1;	
	// ???
	psb->m_cfg.kCHR            =   1;//1;	
	// ???
	psb->m_cfg.kSHR            =   1;//1;
	
	psb->setTotalMass(20,true);
	psb->setPose( false, false );	
		
}

void MySoftBody::createTriMeshShape(const btScalar* vertices, const int* triangles, int ntriangles) {
	
	psb = btSoftBodyHelpers::CreateFromTriMesh(softBodyWI, vertices, triangles, ntriangles);

	psb->m_materials[0]->m_kLST   =   0.99;
	psb->m_materials[0]->m_kAST   =   0.99; // Area/Angular stiffness coefficient [0,1]
	psb->m_materials[0]->m_kVST   =   0.99; // Volume stiffness coefficient [0,1]	

	
	// 粘り気
	psb->m_cfg.kDP            =   0.1;//0.001; 
	// 地面との間で巻き込まれたのが回復する力
	psb->m_cfg.kDG			  =   0;
	// わからない。何かへの抵抗。
	psb->m_cfg.kLF			  =   0;
	// ???
	psb->m_cfg.kDF            =   1;//1;
	// 元に戻る力、弾力
	psb->m_cfg.kPR            =   70050;//2500;
	// ぐちゃぐちゃになる度。中身の量。元に戻らない度。
	psb->m_cfg.kVC            =   0;//2500;
	// 地面と滑るかどうか
	psb->m_cfg.kDF            =   1;//1;
	// ???
	psb->m_cfg.kMT            =   0;//1;	
	// ???
	psb->m_cfg.kCHR            =   1;//1;	
	// ???
	psb->m_cfg.kSHR            =   1;//1;
	
	psb->setTotalMass(20,true);
	psb->setPose( false, false );		
	
//	btSoftBody::Material*	pm=psb->appendMaterial();
//	pm->m_kLST				=	0.5;
////	pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
//	psb->generateBendingConstraints(2,pm);
//	psb->m_cfg.piterations	=	2;
//	psb->m_cfg.kDF			=	0.5;
//	psb->randomizeConstraints();
//	psb->scale(btVector3(6,6,6));
//	psb->setTotalMass(100,true);		
	
}

void MySoftBody::createClothShape(ofxVec3f clothShape[4], 
								  int resolution, int fix) {
	
	/*
	 btSoftBodyWorldInfo& worldInfo,
	 const btVector3& corner00,
	 const btVector3& corner10,
	 const btVector3& corner01,
	 const btVector3& corner11,
	 int resx,
	 int resy,
	 int fixeds,
	 bool gendiags	 
	 */
	psb = btSoftBodyHelpers::CreatePatch(softBodyWI,
										 btVector3(clothShape[0].x, clothShape[0].y, clothShape[0].z),
										 btVector3(clothShape[1].x, clothShape[1].y, clothShape[1].z),
										 btVector3(clothShape[2].x, clothShape[2].y, clothShape[2].z),
										 btVector3(clothShape[3].x, clothShape[3].y, clothShape[3].z),
										 resolution,
										 resolution,
										 fix,
										 false);
	
	psb->getCollisionShape()->setMargin(0.5);
	btSoftBody::Material* pm = psb->appendMaterial();
	pm->m_kLST		=	0.04;	
	psb->generateBendingConstraints(1,pm);
	psb->setTotalMass(150);
	psb->m_cfg.kDP = 0.01;
	psb->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSided;
	psb->generateClusters(1024);
	psb->setTotalMass(100);
	
}

void MySoftBody::render() {

	btSoftBody::tNodeArray& nodes(psb->m_nodes);
	btSoftBody::tLinkArray& links(psb->m_links);
	btSoftBody::tFaceArray& faces(psb->m_faces);

	glColor4f(0.0, 0.0, 0.0, 1.0);
	
	for(int i = 0; i < nodes.size() ; i++) {
		ofxPoint(nodes[i].m_x.getX(), nodes[i].m_x.getY(), nodes[i].m_x.getZ());
	}
	
	ofSetColor(0, 0, 255);
	for(int i = 0; i < links.size(); i++) {
		btSoftBody::Node* node_0 = links[i].m_n[0];
		btSoftBody::Node* node_1 = links[i].m_n[1];
		ofxLine(node_0->m_x.getX(), node_0->m_x.getY(), node_0->m_x.getZ(), 
				node_1->m_x.getX(), node_1->m_x.getY(), node_1->m_x.getZ());
	}		
	
	
	ofSetColor(100, 100, 0.5);
	for(int i = 0; i < faces.size(); i++) {
		btSoftBody::Node* node_0 = faces[i].m_n[0];
		btSoftBody::Node* node_1 = faces[i].m_n[1];
		btSoftBody::Node* node_2 = faces[i].m_n[2];
		ofxTriangleShape(node_0->m_x.getX(), node_0->m_x.getY(), node_0->m_x.getZ(),
						 node_1->m_x.getX(), node_1->m_x.getY(), node_1->m_x.getZ(),
						 node_2->m_x.getX(), node_2->m_x.getY(), node_2->m_x.getZ());	
		
	}	
	
}

ofxVec3f MySoftBody::getBodyCentroid() {

	btSoftBody::tNodeArray& nodes(psb->m_nodes);

	float x = 0.0; 
	float y = 0.0;
	float z = 0.0;
	for (int i = 0; i < nodes.size(); i++) {
		x += nodes[i].m_x.getX();		
		y += nodes[i].m_x.getY();		
		z += nodes[i].m_x.getZ();
	}
	return ofxVec3f(x/nodes.size(), y/nodes.size(), z/nodes.size());
}

vector<ofxVec3f> MySoftBody::getAllFacesAsVerts() {
	
	vector<ofxVec3f> rtn;
	btSoftBody::tFaceArray& faces(psb->m_faces);
	for (int i = 0; i < faces.size(); i++) {
		ofxVec3f a = ofxBulletStaticUtil::btVec3ToOfxVec3(faces[i].m_n[0]->m_x);
		ofxVec3f b = ofxBulletStaticUtil::btVec3ToOfxVec3(faces[i].m_n[1]->m_x);
		ofxVec3f c = ofxBulletStaticUtil::btVec3ToOfxVec3(faces[i].m_n[2]->m_x);
		rtn.push_back(a);
		rtn.push_back(b);
		rtn.push_back(c);
	}
	return rtn;
	
}

vector<ofxVec3f> MySoftBody::getFaceAsVerts(int faceIdx) {
	
	vector<ofxVec3f> rtn;
	btSoftBody::tFaceArray& faces(psb->m_faces);
	ofxVec3f a = ofxBulletStaticUtil::btVec3ToOfxVec3(faces[faceIdx].m_n[0]->m_x);
	ofxVec3f b = ofxBulletStaticUtil::btVec3ToOfxVec3(faces[faceIdx].m_n[1]->m_x);
	ofxVec3f c = ofxBulletStaticUtil::btVec3ToOfxVec3(faces[faceIdx].m_n[2]->m_x);
	rtn.push_back(a);
	rtn.push_back(b);
	rtn.push_back(c);
	return rtn;	
	
}

ofxVec3f MySoftBody::getFaceNormal(int faceIdx) {
	
	btSoftBody::tFaceArray& faces(psb->m_faces);
	btVector3 btNormal = faces[faceIdx].m_normal;
	return ofxVec3f(btNormal.getX(), btNormal.getY(), btNormal.getZ());
	
}

float MySoftBody::getFaceDistanceBetween(int face1Idx, int face2Idx) {
	
	btSoftBody::tFaceArray& faces(psb->m_faces);
	ofxVec3f face1Centroid = getFaceCentroid(face1Idx);
	ofxVec3f face2Centroid = getFaceCentroid(face2Idx);
	return face1Centroid.distance(face2Centroid);
	
}

ofxVec3f MySoftBody::getFaceCentroid(int faceIdx) {
	
	btSoftBody::tFaceArray& faces(psb->m_faces);
	
	btSoftBody::Node* node_0 = faces[faceIdx].m_n[0];
	btSoftBody::Node* node_1 = faces[faceIdx].m_n[1];
	btSoftBody::Node* node_2 = faces[faceIdx].m_n[2];		
	
	return ofxVec3f((node_0->m_x.getX()+node_1->m_x.getX()+node_2->m_x.getX())/3,
					(node_0->m_x.getY()+node_1->m_x.getY()+node_2->m_x.getY())/3,
					(node_0->m_x.getZ()+node_1->m_x.getZ()+node_2->m_x.getZ())/3);
	
}

vector<int> MySoftBody::sortFaceByDistance(int faceIdx) {
	
	btSoftBody::tFaceArray& faces(psb->m_faces);	
	
	map<float, int, greater<float> > faceSize;
	ofxVec3f tgtCentroid = getFaceCentroid(faceIdx);
	for (int i = 0; i < faces.size(); i++) {
		ofxVec3f compCentroid1 = getFaceCentroid(i);
		float d1 = tgtCentroid.distance(compCentroid1);
		faceSize.insert(map<float, int, greater<float> >::value_type(d1, i));
	}
	
	vector<int> faceIDVec;
	map<float, int, greater<float> >::iterator it = faceSize.begin();
	while (it != faceSize.end()) {
		faceIDVec.push_back((*it).second);
		++it;
	}	
	
	return faceIDVec;
}

vector<int> MySoftBody::sortFaceByPosition(ofxVec3f pos) {
	
	btSoftBody::tFaceArray& faces(psb->m_faces);
	
	// get the face distances from pos
	map<float, int, greater<float> > distSort;
	for (int i = 0; i < faces.size(); i++) {
		ofxVec3f cen = getFaceCentroid(i);
		float dist = cen.distance(pos);
		distSort.insert(map<float, int, greater<float> >::value_type(dist, i));
	}
	
	// sort dist desc
	vector<int> faceIDVec;
	map<float, int, greater<float> >::iterator it = distSort.begin();
	while (it != distSort.end()) {
		faceIDVec.push_back((*it).second);
		++it;
	}	
	
	return faceIDVec;
	
}


void MySoftBody::pinchFace(int faceIdx, float force) {
	
	// get face and nodes
	btSoftBody::tFaceArray& faces(psb->m_faces);
	btSoftBody::Node* node_0 = faces[faceIdx].m_n[0];
	btSoftBody::Node* node_1 = faces[faceIdx].m_n[1];
	btSoftBody::Node* node_2 = faces[faceIdx].m_n[2];
	
	// pinch face
	int pinchFaceFactor = force;
	btVector3 normal = faces[faceIdx].m_normal;
	btVector3 pinchFaceForce = normal*pinchFaceFactor;
	
	if (node_0->m_im > 0) node_0->m_f += pinchFaceForce;
	if (node_1->m_im > 0) node_1->m_f += pinchFaceForce;
	if (node_2->m_im > 0) node_2->m_f += pinchFaceForce;	
	
}

void MySoftBody::blowUp(float force) {
	
	// get face and nodes
	btSoftBody::tFaceArray& faces(psb->m_faces);
	int fSize = faces.size();
	
	for (int i = 0; i < fSize; i++) {
		
		btVector3 normal = faces[i].m_normal;
		btVector3 pinchFaceForce = normal*force;
		
		btSoftBody::Node* node_0 = faces[i].m_n[0];
		btSoftBody::Node* node_1 = faces[i].m_n[1];
		btSoftBody::Node* node_2 = faces[i].m_n[2];	
		if (node_0->m_im > 0) node_0->m_f += pinchFaceForce;
		if (node_1->m_im > 0) node_1->m_f += pinchFaceForce;
		if (node_2->m_im > 0) node_2->m_f += pinchFaceForce;	
		
	}	
}

void MySoftBody::shrink(float force) {
	
	// get face and nodes
	btSoftBody::tFaceArray& faces(psb->m_faces);
	int fSize = faces.size();
	
	for (int i = 0; i < fSize; i++) {
		
		btSoftBody::Node* node_0 = faces[i].m_n[0];
		btSoftBody::Node* node_1 = faces[i].m_n[1];
		btSoftBody::Node* node_2 = faces[i].m_n[2];	
		
		// pinch face
		btVector3 normal = -faces[i].m_normal;
		btVector3 pinchFaceForce = normal*force;
		
		if (node_0->m_im > 0) node_0->m_f += pinchFaceForce;
		if (node_1->m_im > 0) node_1->m_f += pinchFaceForce;
		if (node_2->m_im > 0) node_2->m_f += pinchFaceForce;		
	}	
}