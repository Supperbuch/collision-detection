#pragma once

#include <Eigen/Eigen>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <set>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <fstream>
#include<cstdlib>
#include <cstdio>
#include<cstring>
#include <gl/glut.h>

#include <time.h>
//#include"omp.h"

using namespace Eigen;
using namespace std;

class LineSegmentsIntersection3D
{
public:

	Vector3f line_Point0; //create two lines with 4 ponits
	Vector3f line_Point1;
	Vector3f line_Point2;
	Vector3f line_Point3;
	Vector3f vel_P0;
	Vector3f vel_P1;
	Vector3f vel_P2;
	Vector3f vel_P3;
	Vector3f P_linePoint;

	Vector3f centerPoint1;
	Vector3f Vel_1;
	Quaternionf quaternion_Pre1;
	Quaternionf quaternion_1;
	//Matrix3f rotation_matrix_Pre1;
	//Matrix3f rotation_matrix_1;

	Vector3f centerPoint2;
	Vector3f Vel_2;
	Quaternionf quaternion_Pre2;
	Quaternionf quaternion_2;
	//Matrix3f rotation_matrix_Pre2;
	//Matrix3f rotation_matrix_2;

	float Length;
	float Width;
	float Height;

	Vector3f c0; //quadratic coefficients
	Vector3f c1;
	Vector3f c2;

	float c_0; // Cubic coefficients
	float c_1;
	float c_2;
	float c_3;

	float epsilon_Dis = 1.0e-6;
	float epsilon_coQuadratic = 2.2204460492503131e-14;
	float epsilon_coCubic = 2.2204460492503131e-14;
	float epsilon_Dis_1D = 1.0e-6;
	float epsilon_t = 1.0e-9;

	Vector3f StartP;
	Vector3f EndP;

	float Tk; //parallel co-linear case A11,parallel case A13, 2D case A18, A19
	float t_Previous ;
	
	//float t_k; // parallel case
	vector <float> pre_solutionSet; //A12
	Vector3f ParallelP; // intersect point for co-linear case,A10, A13

	//float tk; //2D case
	multiset <float> solutionSet; //A17

	float u;
	float v;
	
	vector<float> X123;  //SolveCubicEquation

	
	Vector3f Inter2DP;  //A18, intersect point for 2D case.
	Vector3f NonDegenerateP; //A20 , intersect point for Non-degenerate case.

	//bool movingPointoverlap;
	bool staticColinearoverlap; //A9
	bool movingIntersection;
	bool foundAnysolutions; //A12
	bool existSolution; //A13
	bool Intersection2D; //A18
	bool isNonDegenerate; //A20


	LineSegmentsIntersection3D(Vector3f CP1, Vector3f V1, Quaternionf QP1, Quaternionf Q1, Vector3f CP2, Vector3f V2, Quaternionf QP2, Quaternionf Q2, float L, float W, float H);

	//~LineSegmentsIntersection3D();

	bool LineSegmentsdegenerate(); //case1

	bool StartPointIdentical();  //case2

	void ComputeCoefficientsCrossProduct(); //A3

	bool AreVectorsPermanentlyParallel(); //algorithm 4

	void ComputeCubicCoefficient();   //algorithm 6

	bool MovingCommenPlane();  //algorithm 7, case4

	void ComputeCoefficientsCrossProduct_Coline();

	bool IsPointPermanentlyOnLine();  //algorithm 8

	void StaticCoLinearOverlap(const float &t_k); //algorithm 9

	void MovingPointsIntersection(Vector3f &p1, Vector3f &p2, Vector3f &v1, Vector3f &v2); //algorithm 10

	void HandleParallelCoLinear(); //A11

	void SolveThreeQuadraticEquations(); //A12

	void HandleParallelNonDegenerateCase(); //A13

	bool StaticPointSegmentIntersection(const Vector3f &p_0, const Vector3f &p_1, const Vector3f &v_0, const Vector3f &v_1,
		const Vector3f &p_2, const Vector3f &p_3, const Vector3f &v_2, const Vector3f &v_3, const float &t); // A15

	//float * GaussianElimination(float (&Am)[2][3]);
	void BinaryLinearEquations(float &a, float &b, float &c, float &d, float &e, float &f);

	bool StaticSegmentSegmentIntersection(const float &T_k);  // A16

	void MovingPointSegmentIntersection(const Vector3f &P_0, const Vector3f &P_1, const Vector3f &V_0, const Vector3f &V_1,
		const Vector3f &P_2, const Vector3f &P_3, const Vector3f &V_2, const Vector3f &V_3);  //A17

	void Handle2DCase(const Vector3f &P0, const Vector3f &P1, const Vector3f &V0, const Vector3f &V1,
		const Vector3f &P2, const Vector3f &P3, const Vector3f &V2, const Vector3f &V3); //A18

	void SolveCubicEquation(float &a, float &b, float &c, float &d);

	void HandleNonDegenerateCase();  //A19

	void HandleGeneralCase();  //A20 


	//typedef struct CollisionManifold {
	//	bool colliding;
	//	Vector3f normal;
	//	float depth;
	//	std::vector<Vector3f> contacts;
	//};

	//void ResetCollisionManifold(CollisionManifold* result);
};


