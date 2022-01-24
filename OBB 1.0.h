#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <ostream>
#include <vector>
#include <utility>
//#include <GLFW/glfw3.h>
#include <gl/glut.h>

using namespace Eigen;
using namespace std;

class OBB 
{
public:
	/*typedef struct CollisionManifold {
		bool colliding;
		Vector3f normal;
		float depth;
		std::vector<Vector3f> contacts;
	};*/

	bool colliding;
	Vector3f normal;
	float depth;
	vector<Vector3f> contacts;


	float Length;
	float Width;
	float Height;
	Vector3f position1;
	Quaternionf quaternion1;
	Matrix3f orientation1;
	Vector3f position2;
	Quaternionf quaternion2;
	Matrix3f orientation2;

	
	
	//Plane
	Vector3f normal_P;
	float distance;

	Vector3f start;
	Vector3f end;
	MatrixXf Line;

	
	//coordinates
	Vector3f x;
	Vector3f y;
	Vector3f z;

	//Validation of distance, connect the points, which is the smallest distance
	Vector3f Point1;
	Vector3f Point2;
	

	
	OBB(Vector3f CP1, Vector3f CP2, Quaternionf q1, Quaternionf q2, float l, float w, float h);

	bool PointInOBB(const Vector3f& point, Vector3f &p, Matrix3f &o, float &l, float &w, float &h);
	VectorXf GetInterval(Vector3f &p1, Matrix3f &o1, float &l1, float &w1, float &h1, 
						 Vector3f &p2, Matrix3f &o2, float &l2, float &w2, float &h2, const Vector3f& axis);

	vector<Vector3f> GetVertices(Vector3f &p, Matrix3f &o, float &l, float &w, float &h);
	vector<MatrixXf> GetEdges(Vector3f &p, Matrix3f &o, float &l, float &w, float &h);
	vector<VectorXf> GetPlanes(Vector3f &p, Matrix3f &o, float &l, float &w, float &h);
	bool ClipToPlane(const VectorXf& plane, const MatrixXf& line, Vector3f* outPoint);
	
	vector<Vector3f> ClipEdgesToOBB(const vector<MatrixXf>& edges, Vector3f &p, Matrix3f &o, float &l, float &w, float &h);
	float PenetrationDepth(vector<Vector3f> &contact1, vector<Vector3f> &contact2, Vector3f &p1, Matrix3f &o1, float &l1, float &w1, float &h1, 
						   Vector3f &p2, Matrix3f &o2, float &l2, float &w2, float &h2, const Vector3f &axis, bool* outShouldFlip);

	void FindCollisionFeatures();

	VectorXf result();
	bool CorrectNormal(Vector3f &aixs);
	void projectToNormal(Vector3f &p, Matrix3f &o, Vector3f &normal, Vector3f &pointColor);
	void drawProjectPoint();

	void drawBox(Vector3f &color1, Vector3f &color2, float &lineWidth);

	//verification depth and calculate the real distance of boxes(after separate)

	Matrix3f PointToSegment(Vector3f &A, Vector3f &B, Vector3f &C);// Distance of point to segment

	void  DistanceOfBoxesVerification(); //Moller - Trumbore intersection algorithm and SegmentSegment distance Algorithm

};

