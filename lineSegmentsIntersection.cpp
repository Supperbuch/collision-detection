#include "lineSegmentsIntersection.h"

#include <algorithm>
//#include <Eigen/Eigen>
//#include <algorithm>
//#include <stdio.h>
//#include <set>
//#include<cmath>

LineSegmentsIntersection3D::LineSegmentsIntersection3D(Vector3f CP1, Vector3f V1, Quaternionf QP1, Quaternionf Q1, Vector3f CP2, Vector3f V2, Quaternionf QP2, Quaternionf Q2, float L, float W, float H)
{
	Vector3f P0_Pre(L / 2, 0, 0);
	Vector3f P1_Pre(-L / 2, 0, 0);
	Vector3f P2_Pre(L / 2, 0, 0);
	Vector3f P3_Pre(-L / 2, 0, 0);

	line_Point0 = QP1 * P0_Pre + CP1;
	line_Point1 = QP1 * P1_Pre + CP1;
	line_Point2 = QP2 * P2_Pre + CP2;
	line_Point3 = QP2 * P3_Pre + CP2;
	vel_P0 = Q1 * line_Point0 + V1 - line_Point0;// Rotationmatrix * P + V = P + V'-->V'=....
	vel_P1 = Q1 * line_Point1 + V1 - line_Point1;
	vel_P2 = Q2 * line_Point2 + V2 - line_Point2;
	vel_P3 = Q2 * line_Point3 + V2 - line_Point3;

	float epsilon_Dis = 1.0e-6;
	float epsilon_coQuadratic = 2.2204460492503131e-14;
	float epsilon_coCubic = 2.2204460492503131e-14;
	float epsilon_Dis_1D = 1.0e-6;
	float epsilon_t = 1.0e-9;


	//bool movingPointoverlap = false;
	bool staticColinearoverlap = false;
	bool movingIntersection = false;
	bool foundAnysolutions = false;
	bool existSolution = false;
	bool Intersection2D = false;
	bool isNonDegenerate = false;

}

//LineSegmentsIntersection3D::~LineSegmentsIntersection3D()
//{
//}

bool LineSegmentsIntersection3D::LineSegmentsdegenerate() // case1
{
	//Determine whether the line segment degenerates to a point
	Vector3f Pk_1 = line_Point0 - line_Point1;
	Vector3f Vk_1 = vel_P0 - vel_P1;
	Vector3f Pk_2 = line_Point2 - line_Point3;
	Vector3f Vk_2 = vel_P2 - vel_P3;
	if ((Pk_1.norm() <= epsilon_Dis && Vk_1.norm() <= epsilon_Dis) || (Pk_2.norm() <= epsilon_Dis && Vk_2.norm() <= epsilon_Dis))
		return true;
	else
		return false;
}

bool LineSegmentsIntersection3D::StartPointIdentical()
{
	//Determine whether the start point of the lines is the same, e.g P0 = P2
	Vector3f Pk1 = line_Point0 - line_Point2;
	Vector3f Vk1 = vel_P0 - vel_P2;
	if (Pk1.norm() <= epsilon_Dis && Vk1.norm() <= epsilon_Dis)
	{
		StartP = line_Point0;
		EndP = line_Point0 + vel_P0;
		return true;
	}
	Vector3f Pk2 = line_Point0 - line_Point3;
	Vector3f Vk2 = vel_P0 - vel_P3;
	if (Pk2.norm() <= epsilon_Dis && Vk2.norm() <= epsilon_Dis)
	{
		StartP = line_Point0;
		EndP = line_Point0 + vel_P0;
		return true;
	}
	Vector3f Pk3 = line_Point1 - line_Point2;
	Vector3f Vk3 = vel_P1 - vel_P2;
	if (Pk3.norm() <= epsilon_Dis && Vk3.norm() <= epsilon_Dis)
	{
		StartP = line_Point1;
		EndP = line_Point1 + vel_P1;
		return true;
	}
	Vector3f Pk4 = line_Point1 - line_Point3;
	Vector3f Vk4 = vel_P1 - vel_P3;
	if (Pk4.norm() <= epsilon_Dis && Vk4.norm() <= epsilon_Dis)
	{
		StartP = line_Point1;
		EndP = line_Point1 + vel_P1;
		return true;
	}

	return false;

	/*vector<Vector3f> Sum = { Pk1, Vk1, Pk2, Vk2, Pk3, Vk3, Pk4, Vk4 };
	for (int i = 0; i < Sum.size(); i=i+2)
	{
		if (Sum[i].norm() <= epsilon_Dis && Sum[i+1].norm() <= epsilon_Dis)

			return true;
	}

	return false;*/
}

void LineSegmentsIntersection3D::ComputeCoefficientsCrossProduct()
{
	// Calculate the coefficients of the equation after the cross product of two line segments
	/*
	Vector3f c0 ;
	Vector3f c1 ;
	Vector3f c2 ;

	P = P1 - P0;
	Q = P3 - p2;
	R = V1 - V0;
	S = V3 - V2;
	*/
	Vector3f P = line_Point1 - line_Point0;
	Vector3f Q = line_Point3 - line_Point2;
	Vector3f R = vel_P1 - vel_P0;
	Vector3f S = vel_P3 - vel_P2;
	c0(0) = R(1) * S(2) - R(2) * S(1);
	c0(1) = R(2) * S(0) - R(0) * S(2);
	c0(2) = R(0) * S(1) - R(1) * S(0);

	c1(0) = P(1) * S(2) - P(2) * S(1) + R(1) * Q(2) - R(2) * Q(1);
	c1(1) = P(2) * S(0) - P(0) * S(2) + R(2) * Q(0) - R(0) * Q(2);
	c1(2) = P(0) * S(1) - P(1) * S(0) + R(0) * Q(1) - R(1) * Q(0);

	c2(0) = P(1) * Q(2) - P(2) * Q(1);
	c2(1) = P(2) * Q(0) - P(0) * Q(2);
	c2(2) = P(0) * Q(1) - P(1) * Q(0);

}

bool LineSegmentsIntersection3D::AreVectorsPermanentlyParallel()
{

	ComputeCoefficientsCrossProduct();
	for (int i = 0; i < 3; ++i)
	{
		if (fabs(c0(i)) > epsilon_coQuadratic || fabs(c1(i)) > epsilon_coQuadratic || fabs(c2(i)) > epsilon_coQuadratic) //test every value of c0,c1,c2
			return false;
	}
	return true;
}

void LineSegmentsIntersection3D::ComputeCubicCoefficient()
{
	//compute the coefficents of cubic equations. 
	/*
	Vector3f c_0;
	Vector3f c_1;
	Vector3f c_2;
	Vector3f c_3;
	*/
	ComputeCoefficientsCrossProduct();
		Vector3f U = line_Point2 - line_Point0;
	Vector3f W = vel_P2 - vel_P0;
	c_0 = c0.dot(W);
	c_1 = c0.dot(U) + c1.dot(W);
	c_2 = c1.dot(U) + c2.dot(W);
	c_3 = U.dot(c2);
}

bool LineSegmentsIntersection3D::MovingCommenPlane()  // A7
{
	//Determine whether the two line segments are moving on the plane
	ComputeCubicCoefficient();	
	if (fabs(c_0) > epsilon_coCubic || fabs(c_1) > epsilon_coCubic || fabs(c_2) > epsilon_coCubic || fabs(c_3) > epsilon_coCubic)
		return false;
	else
		return true;
		
	
}

void LineSegmentsIntersection3D::ComputeCoefficientsCrossProduct_Coline()
{
	// calculate the coefficients of cross product for one point with one line (collinear)

	Vector3f P = line_Point1 - line_Point0;
	Vector3f Q = line_Point2 - line_Point0;
	Vector3f R = vel_P1 - vel_P0;
	Vector3f S = vel_P2 - vel_P0;

	c0(0) = R(1) * S(2) - R(2) * S(1);
	c0(1) = R(2) * S(0) - R(0) * S(2);
	c0(2) = R(0) * S(1) - R(1) * S(0);

	c1(0) = P(1) * S(2) - P(2) * S(1) + R(1) * Q(2) - R(2) * Q(1);
	c1(1) = P(2) * S(0) - P(0) * S(2) + R(2) * Q(0) - R(0) * Q(2);
	c1(2) = P(0) * S(1) - P(1) * S(0) + R(0) * Q(1) - R(1) * Q(0);

	c2(0) = P(1) * Q(2) - P(2) * Q(1);
	c2(1) = P(2) * Q(0) - P(0) * Q(2);
	c2(2) = P(0) * Q(1) - P(1) * Q(0);

}

bool LineSegmentsIntersection3D::IsPointPermanentlyOnLine()
{
	//Calculate whether a point on a line segment is collinear with another line segment
	/*
	Vector3f c0 ;
	Vector3f c1 ;
	Vector3f c2 ;
	(B(t) - A(t)) x (C(t) - D(t));
	*/
	ComputeCoefficientsCrossProduct_Coline();

		for (int i = 0; i < 3; ++i)
		{
			if (fabs(c0(i)) > epsilon_coQuadratic || fabs(c1(i)) > epsilon_coQuadratic || fabs(c2(i)) > epsilon_coQuadratic) //test every value of c0,c1,c2
				return false;
		}
	return true;

}

void LineSegmentsIntersection3D::StaticCoLinearOverlap(const float &t_k)
{
	// For a given time t. We can judge whether the two lines overlap according to this function(collinear)
	Vector3f line_P;   //P
	Vector3f line_P_vector;  //R

	if ((line_Point0 - line_Point1 + vel_P0 * t_k - vel_P1 * t_k).norm() <= epsilon_Dis) //first line is a point
	{
		if ((line_Point2 - line_Point3 + vel_P2 * t_k - vel_P3 * t_k).norm() <= epsilon_Dis)  //second line is a point
		{
			if ((line_Point0 - line_Point2 + vel_P0 * t_k - vel_P2 * t_k).norm() <= epsilon_Dis)  //A,C overlap
				staticColinearoverlap = true;
			else
				staticColinearoverlap = false;
		}
			

		else
		{
			line_P_vector = line_Point3 + vel_P3 * t_k - (line_Point2 + vel_P2 * t_k);
			line_P = line_Point2 + vel_P2 * t_k;
		}
	}
	else
	{
		line_P_vector = line_Point1 + vel_P1 * t_k - (line_Point0 + vel_P0 * t_k);
		line_P = line_Point0 + vel_P0 * t_k;
	}

	float Al = (line_P - line_Point0 - vel_P0 * t_k).dot(line_P_vector);
	float Bl = (line_P - line_Point1 - vel_P1 * t_k).dot(line_P_vector);
	float Cl = (line_P - line_Point2 - vel_P2 * t_k).dot(line_P_vector);
	float D1 = (line_P - line_Point3 - vel_P3 * t_k).dot(line_P_vector);

	if (max(Al, Bl) < min(Cl, D1) || min(Al, Bl) > max(Cl, D1))
		staticColinearoverlap = false;
	else
		staticColinearoverlap = true;
}

void LineSegmentsIntersection3D::MovingPointsIntersection(Vector3f &p1, Vector3f &p2, Vector3f &v1, Vector3f &v2)
{

	//Calculate the time when two points collide in motion (collinear)

	/*Tk = 0, Project to a line, then sort, take the middle two points
	select P0 point as a point on commen line, and P1 - P0 as direction
	*/
	Vector3f Alpha;
	Vector3f Beta;
	bool foundPreviousT = false;
	float t;

	Alpha = v1 - v2;
	Beta = p2 - p1;
	//float t_Previous = 0;
	for (int i = 0; i < 3; ++i)
	{
		if (fabs(Alpha(i)) <= epsilon_Dis_1D)
		{
			continue;
		}
		else
		{
			t = Beta(i) / Alpha(i);
			//if(t < 0 || t > 1)
			//	return false;
			if (foundPreviousT)
			{
				if (fabs(t - t_Previous) > epsilon_t)
					break;
			}
			else
			{
				foundPreviousT = true;
				t_Previous = t;
			}
		}
	}

}

void LineSegmentsIntersection3D::HandleParallelCoLinear()
{
	// Under collinear conditions, We can use this function to get the time of the first collision
	StaticCoLinearOverlap(0);
	if (staticColinearoverlap)
	{
		movingIntersection = true;
		Tk = 0;
		Vector3f SecondP;
		Vector3f ThirdP;

		for (int n = 0; n < 3; ++n)
		{
			vector <float> Points{ line_Point0(n), line_Point1(n), line_Point2(n), line_Point3(n) };
			for (int i = 0; i < Points.size() - 1; ++i)
			{
				for (int j = 0; j < Points.size() - 1 - i; ++j)
				{
					if (Points[j] > Points[j + 1])
						swap(Points[j], Points[j + 1]);
				}
			}
			SecondP(n) = Points[1];
			ThirdP(n) = Points[2];
			Points.clear();
		}
		ParallelP = (SecondP + ThirdP) / 2;
		return;
	}		 
	else
	{
		MovingPointsIntersection(line_Point0, line_Point2, vel_P0, vel_P2);
		float t0 = t_Previous;	
		MovingPointsIntersection(line_Point0, line_Point3, vel_P0, vel_P3);
		float t1 = t_Previous;
		MovingPointsIntersection(line_Point1, line_Point2, vel_P1, vel_P2);
		float t2 = t_Previous;
		MovingPointsIntersection(line_Point1, line_Point3, vel_P1, vel_P3);
		float t3 = t_Previous;
	
		multiset<float> listTime = {t0, t1, t2, t3};
		for (auto element = listTime.begin(); element != listTime.end(); ++element)
		{
			if (*element > 0 && *element <= 1)
			{
				movingIntersection = true;
				Tk = *element;
				break;
			}
			movingIntersection = false;
		}
		if (Tk > 0 && Tk <= 1)
		{
			//movingIntersection = true;
			Vector3f p0 = line_Point0 + Tk * vel_P0;
			Vector3f p1 = line_Point1 + Tk * vel_P1;
			Vector3f p2 = line_Point2 + Tk * vel_P2;
			Vector3f p3 = line_Point3 + Tk * vel_P3;
			vector<Vector3f> InterPoint = { p0, p1, p2, p3 };
			for (int i = 0; i < InterPoint.size(); ++i)
			{
				for (int j = i + 1; j < InterPoint.size(); ++j)
				{
					if (fabs(InterPoint[i](0) - InterPoint[j](0)) < epsilon_Dis &&
						fabs(InterPoint[i](1) - InterPoint[j](1)) < epsilon_Dis && 
						fabs(InterPoint[i](2) - InterPoint[j](2) < epsilon_Dis))
						ParallelP = InterPoint[i];
				}
			}
		}	
	}
}

void LineSegmentsIntersection3D::SolveThreeQuadraticEquations()
{
	/* This is for Parallel Non-Degenerate Case. We can calculate the
	time corresponding to the cross product of one point to another line segment to zero.
	*/
	pre_solutionSet.clear();
	ComputeCoefficientsCrossProduct_Coline();

	for (int i = 0; i < 3; ++i)
	{
		if (fabs(c0(i)) < epsilon_coQuadratic && fabs(c1(i)) < epsilon_coQuadratic && fabs(c2(i)) < epsilon_coQuadratic)
			continue;

		float t1, t2;

		if (fabs(c0(i)) < epsilon_coQuadratic)
		{
			if (-c2(i) / c1(i) >= 0 && -c2(i) / c1(i) <= 1)
				pre_solutionSet.push_back(-c2(i) / c1(i));
		}
		else
		{
			float d = c1(i) * c1(i) - 4 * c0(i) * c2(i);
			if (d < 1.0e-6)
			{
				foundAnysolutions = false;
				pre_solutionSet.clear();
				break;
			}
			if (fabs(d) < 1.0e-6)
			{
				t1 = -c1(i) / (2 * c0(i));
				if (t1 >= 0 && t1 <= 1)
					pre_solutionSet.push_back(t1);
			}
			if (d > 0)
			{
				t1 = (-c1(i) + sqrt(d)) / (2 * c0(i));
				t2 = (-c1(i) - sqrt(d)) / (2 * c0(i));
				if (t1 >= 0 && t1 <= 1)
					pre_solutionSet.push_back(t1);
				if (t2 >= 0 && t2 <= 1)
					pre_solutionSet.push_back(t2);
			}

		}
	}
	if (!pre_solutionSet.empty())
		foundAnysolutions = true;
}

void LineSegmentsIntersection3D::HandleParallelNonDegenerateCase()
{
	/*
	Put the time obtained above into this equation,
	If it is satisfied that the line segments overlap at this moment,
	the smallest time is the solution we want
	*/
	SolveThreeQuadraticEquations();
	multiset <float>::iterator tkLocator;
	multiset <float> solutionSet;

	if (!pre_solutionSet.empty())
	{
		for (auto tkLocator = pre_solutionSet.begin();
			tkLocator != pre_solutionSet.end();
			++tkLocator)
		{
			StaticCoLinearOverlap(*tkLocator);
			if (staticColinearoverlap)
			{
				solutionSet.insert(*tkLocator);
			}
		}	
	}
	if (!solutionSet.empty())
	{
		existSolution = true;
		Tk = *solutionSet.begin();
		Vector3f p0 = line_Point0 + vel_P0 * Tk;
		Vector3f p1 = line_Point1 + vel_P1 * Tk;
		Vector3f p2 = line_Point2 + vel_P2 * Tk;
		Vector3f p3 = line_Point3 + vel_P3 * Tk;

		Vector3f SecondP;
		Vector3f ThirdP;

		for (int n = 0; n < 3; ++n)
		{
			vector <float> Points{ p0(n), p1(n), p2(n), p3(n) };
			for (int i = 0; i < Points.size() - 1; ++i)
			{
				for (int j = 0; j < Points.size() - 1 - i; ++j)
				{
					if (Points[j] > Points[j + 1])
						swap(Points[j], Points[j + 1]);
				}
			}
			SecondP(n) = Points[1];
			ThirdP(n) = Points[2];
			Points.clear();
		}
		ParallelP = (SecondP + ThirdP) / 2;
	}
	else
		existSolution = false;
}


bool LineSegmentsIntersection3D::StaticPointSegmentIntersection(const Vector3f &p_0, const Vector3f &p_1, const Vector3f &v_0, const Vector3f &v_1,
	const Vector3f &p_2, const Vector3f &p_3, const Vector3f &v_2, const Vector3f &v_3, const float &t)
	// For a given time, judge whether a point is on the another line segment

{
	if ((p_3 - p_2 + v_3 * t - v_2 * t).norm() <= epsilon_Dis)
	{
		if ((p_0 - p_2 + v_0 * t - v_2 * t).norm() <= epsilon_Dis)
			return true;
		else
			return false;
	}

	u = 0;
	v = 0;

	Vector3f S = p_3 + v_3 * t - p_2 - v_2 * t;	 
	Vector3f B = p_0 + v_0 * t - p_2 - v_2 * t;  //P - Q
	bool foundU = false;
	//float u = 0;
	for (int i = 0; i < 3; ++i)
	{
		if (fabs(S(i)) <= epsilon_Dis_1D)
		{
			if (fabs(B(i)) > epsilon_Dis_1D)
				return false;
			else
				continue;
		}

		float ui = B(i) / S(i);
		if (foundU)
		{
			if (fabs(u - ui) > epsilon_Dis_1D)
				return false;
		}
		else
		{
			foundU = true;
			u = ui;
		}		
	}
	if (u >= 0 && u <= 1)
		return true;
	else
		return false;

}

//float * LineSegmentsIntersection3D::GaussianElimination(float (&Am)[2][3])
//{
//	float * X0_1 = new float[2];
//
//	for (int i = 0; i < 2; i++) {
//		for (int j = i + 1; j < 2; j++) {
//			float t = -Am[j][i] / Am[i][i];
//			float l[3];
//			for (int k = 0; k < 3; k++) {
//				l[k] = Am[i][k] * t + Am[j][k];
//			}
//			for (int k = 0; k < 3; k++) {
//				Am[j][k] = l[k];
//			}
//		}
//	}
//	for (int i = 1; i >= 0; i--) {
//		//X0_1[i] = Am[i][2]/Am[i][1];
//		float t = Am[i][2];
//		for (int j = 1; j > i; j--) {
//
//			t -= (Am[i][j] * X0_1[j]);
//		}
//		X0_1[i] = t / Am[i][i];
//	}
//	return X0_1;
//}

void LineSegmentsIntersection3D::BinaryLinearEquations(float &a, float &b, float &c, float &d, float &e, float &f)
{

	v = (c*d - f * a) / (b*d - e * a);
	u = (c*e - f * b) / (e*a - b * d);
}

bool LineSegmentsIntersection3D::StaticSegmentSegmentIntersection(const float &T_k)
{
	/*There are 3 situations here
	input is tk
	1. point line interection
	2. parallel and co-linear(static Co-linear overlap)
	3. given t of intersection, which satisfy equation(2.6), then calculate the unknows u and
		v ------> ¡Ê [0,1]
	*/
	Vector3f R = line_Point1 - line_Point0 + vel_P1 * T_k - vel_P0 * T_k;
	Vector3f S = line_Point3 - line_Point2 + vel_P3 * T_k - vel_P2 * T_k;
	Vector3f P = line_Point0 + vel_P0 * T_k;
	Vector3f Q = line_Point2 + vel_P2 * T_k;

	//first line segment degenerate to point
	if (R.norm() <= epsilon_Dis)
		return StaticPointSegmentIntersection(line_Point0, line_Point1, vel_P0, vel_P1, line_Point2, line_Point3, vel_P2, vel_P3, T_k);

	//second line segment degenerate to point
	if (S.norm() <= epsilon_Dis)
		return StaticPointSegmentIntersection(line_Point2, line_Point3, vel_P2, vel_P3, line_Point0, line_Point1, vel_P0, vel_P1, T_k);

	//judge whether line segments are parallel and co-linear
	ComputeCoefficientsCrossProduct();
	Vector3f equation_Vale1 = T_k * T_k * c0 + T_k * c1 + c2;

	ComputeCoefficientsCrossProduct_Coline();
	Vector3f equation_Vale2 = T_k * T_k * c0 + T_k * c1 + c2;

	if (equation_Vale1.norm() <= 10.0e-6 && equation_Vale2.norm() <= 10.0e-6)
	{
		StaticCoLinearOverlap(T_k);
		return staticColinearoverlap;
	}
	if (equation_Vale1.norm() <= epsilon_Dis)
		return false;
	

	Vector3f B = Q - P;
	/*
		|M00 M01|
		|M01 M11|
	*/
	float M00 = R.dot(R);
	float M01 = -R.dot(S);
	float M11 = S.dot(S);
	float L0 = R.dot(B);
	float L1 = -S.dot(B);

	//float augMatrix[2][3] = { {M00, M01, L0}, {M01, M11 , L1} };

	//float * result = GaussianElimination(augMatrix);
	//u = result[0];
	//v = result[1];
	BinaryLinearEquations(M00, M01, L0, M01, M11, L1);

	//we should consider calculation errors.
	if ((u > 1+epsilon_Dis || u < -epsilon_Dis) || (v > 1+ epsilon_Dis || v < -epsilon_Dis))
		return false;

	for (int i = 0; i < 3; ++i)
	{
		float delta = R(i) * u - S(i) * v - B(i);
		if (fabs(delta) > 10.0e-6)
			return false;
	}
	return true;
}

void LineSegmentsIntersection3D::MovingPointSegmentIntersection(const Vector3f &P_0, const Vector3f &P_1, const Vector3f &V_0, const Vector3f &V_1,
	const Vector3f &P_2, const Vector3f &P_3, const Vector3f &V_2, const Vector3f &V_3)

	// We can use this function to calculate whether a moving point intersects a line. 
	// A time solution set can be obtained by the cross product equation of points and line segments

{
	pre_solutionSet.clear();
	bool foundAnysolutions = false;
	//multiset <float> pre_solutionSet;

	Vector3f P = P_1 - P_0;
	Vector3f Q = P_2 - P_0;
	Vector3f R = V_1 - V_0;
	Vector3f S = V_2 - V_0;

	c0(0) = R(1) * S(2) - R(2) * S(1);
	c0(1) = R(2) * S(0) - R(0) * S(2);
	c0(2) = R(0) * S(1) - R(1) * S(0);

	c1(0) = P(1) * S(2) - P(2) * S(1) + R(1) * Q(2) - R(2) * Q(1);
	c1(1) = P(2) * S(0) - P(0) * S(2) + R(2) * Q(0) - R(0) * Q(2);
	c1(2) = P(0) * S(1) - P(1) * S(0) + R(0) * Q(1) - R(1) * Q(0);

	c2(0) = P(1) * Q(2) - P(2) * Q(1);
	c2(1) = P(2) * Q(0) - P(0) * Q(2);
	c2(2) = P(0) * Q(1) - P(1) * Q(0);

	for (int i = 0; i < 3; ++i)
	{
		if (fabs(c0(i)) < epsilon_coQuadratic && fabs(c1(i)) < epsilon_coQuadratic && fabs(c2(i)) < epsilon_coQuadratic)
			continue;

		float t1, t2;

		if (fabs(c0(i)) < epsilon_coQuadratic)
		{
			if (-c2(i) / c1(i) >= 0 && -c2(i) / c1(i) <= 1)
				pre_solutionSet.push_back(-c2(i) / c1(i));
		}
		else
		{
			float d = c1(i) * c1(i) - 4 * c0(i) * c2(i);
			if (d < 0)
			{
				foundAnysolutions = false;
				pre_solutionSet.clear();
				break;
			}		
			if (d == 0)
			{
				t1 = -c1(i) / (2 * c0(i));
				if (t1 >= 0 && t1 <= 1)
					pre_solutionSet.push_back(t1);
			}
			if (d > 0)
			{
				t1 = (-c1(i) + sqrt(d)) / (2 * c0(i));
				t2 = (-c1(i) - sqrt(d)) / (2 * c0(i));
				if (t1 >= 0 && t1 <= 1)
					pre_solutionSet.push_back(t1);
				if (t2 >= 0 && t2 <= 1)
					pre_solutionSet.push_back(t2);
			}

		}
	}
	if (!pre_solutionSet.empty())
		foundAnysolutions = true;

	multiset <float>::iterator tkLocator;
	//multiset <float> solutionSet;
	if (!pre_solutionSet.empty())
	{
		for (auto tkLocator = pre_solutionSet.cbegin();
			tkLocator != pre_solutionSet.cend();
			++tkLocator)
		{
			if (StaticPointSegmentIntersection(P_2, P_3, V_2, V_3, P_0, P_1, V_0, V_1, *tkLocator))
			{
				solutionSet.insert(*tkLocator);
			}

		}

	}

}

void LineSegmentsIntersection3D::Handle2DCase(const Vector3f &P0, const Vector3f &P1, const Vector3f &V0, const Vector3f &V1,
	const Vector3f &P2, const Vector3f &P3, const Vector3f &V2, const Vector3f &V3)
{
	//First determine whether the line segments overlap at the start time
	//Then we find the time set through the intersection of points and lines(All 4 points must be calculated, take the minimum value)

	//bool Intersection2D = false;
	multiset <float> solutionSetend;
	if (StaticSegmentSegmentIntersection(0))
	{
		if (staticColinearoverlap)
		{
			Vector3f SecondP;
			Vector3f ThirdP;
			Tk = 0;
			Intersection2D = true;

			for (int n = 0; n < 3; ++n)
			{
				vector <float> Points{ line_Point0(n), line_Point1(n), line_Point2(n), line_Point3(n) };
				for (int i = 0; i < Points.size() - 1; ++i)
				{
					for (int j = 0; j < Points.size() - 1 - i; ++j)
					{
						if (Points[j] > Points[j + 1])
							swap(Points[j], Points[j + 1]);
					}
				}
				SecondP(n) = Points[1];
				ThirdP(n) = Points[2];
				Points.clear();
			}
			Inter2DP = (SecondP + ThirdP) / 2;
			return;
		}
		Intersection2D = true;
		Tk = 0;
		Inter2DP = line_Point0 + (line_Point1 - line_Point0)*u;
		return;
	}
	
	MovingPointSegmentIntersection(P0, P1, V0, V1, P2, P3, V2, V3);  //Moving point P2
	if (!solutionSet.empty())
		solutionSetend.insert(solutionSet.begin(), solutionSet.end());  
	MovingPointSegmentIntersection(P0, P1, V0, V1, P3, P2, V3, V2);   //Moving point P3
	if (!solutionSet.empty())
		solutionSetend.insert(solutionSet.begin(), solutionSet.end());
	MovingPointSegmentIntersection(P2, P3, V2, V3, P0, P1, V0, V1);   //Moving point P0
	if (!solutionSet.empty())
		solutionSetend.insert(solutionSet.begin(), solutionSet.end());
	MovingPointSegmentIntersection(P2, P3, V2, V3, P1, P0, V1, V0);	  //Moving point P1
	if (!solutionSet.empty())
		solutionSetend.insert(solutionSet.begin(), solutionSet.end());
	
	if (!solutionSetend.empty())
	{
		Tk = *solutionSetend.begin();
		Intersection2D = true;
		Vector3f p0 = P0 + V0 * Tk;
		Vector3f p1 = P1 + V1 * Tk;
		Vector3f p2 = P2 + V2 * Tk;
		Vector3f p3 = P3 + V3 * Tk;
		
		if (((p0 - p2).cross(p0 - p3)).norm() <= 10.0e-6 && ((p0 - p1).cross(p2 - p3)).norm() >= 10.0e-6) //p0 is on the line segment p2-p3
		{
			if (p0(0) >= min(p2(0), p3(0)) && p0(0) <= max(p2(0), p3(0)) &&
				p0(1) >= min(p2(1), p3(1)) && p0(1) <= max(p2(1), p3(1)) &&
				p0(2) >= min(p2(2), p3(2)) && p0(2) <= max(p2(2), p3(2)))
			{
				Inter2DP = p0;
				u = (p0(0) - p2(0)) / (p3(0) - p2(0));
			}		
		}
		else if (((p1 - p2).cross(p1 - p3)).norm() <= 10.0e-6 && ((p0 - p1).cross(p2 - p3)).norm() >= 10.0e-6)
		{
			if (p1(0) >= min(p2(0), p3(0)) && p1(0) <= max(p2(0), p3(0)) &&
				p1(1) >= min(p2(1), p3(1)) && p1(1) <= max(p2(1), p3(1)) &&
				p1(2) >= min(p2(2), p3(2)) && p1(2) <= max(p2(2), p3(2)))
			{
				Inter2DP = p1;
				u = (p1(0) - p2(0)) / (p3(0) - p2(0));
			}			
		}
		else if (((p2 - p0).cross(p2 - p1)).norm() <= 10.0e-6 && ((p0 - p1).cross(p2 - p3)).norm() >= 10.0e-6)
		{
			if (p2(0) >= min(p0(0), p1(0)) && p2(0) <= max(p0(0), p1(0)) &&
				p2(1) >= min(p0(1), p1(1)) && p2(1) <= max(p0(1), p1(1)) &&
				p2(2) >= min(p0(2), p1(2)) && p2(2) <= max(p0(2), p1(2)))
			{
				Inter2DP = p2;
				u = (p2(0) - p0(0)) / (p1(0) - p0(0));
			}				
		}
		else if (((p3 - p0).cross(p3 - p1)).norm() <= 10.0e-6 && ((p0 - p1).cross(p2 - p3)).norm() >= 10.0e-6)
		{
			if (p3(0) >= min(p0(0), p1(0)) && p3(0) <= max(p0(0), p1(0)) &&
				p3(1) >= min(p0(1), p1(1)) && p3(1) <= max(p0(1), p1(1)) &&
				p3(2) >= min(p0(2), p1(2)) && p3(2) <= max(p0(2), p1(2)))
			{
				Inter2DP = p3;
				u = (p3(0) - p0(0)) / (p1(0) - p0(0));
			}
				
		}
		else if (((p0 - p1).cross(p2 - p3)).norm() <= 10.0e-6)
		{
			Vector3f SecondP;
			Vector3f ThirdP;
			u = 0;
			v = 0;

			for (int n = 0; n < 3; ++n)
			{
				vector <float> Points{ p0(n), p1(n), p2(n), p3(n) };
				for (int i = 0; i < Points.size() - 1; ++i)
				{
					for (int j = 0; j < Points.size() - 1 - i; ++j)
					{
						if (Points[j] > Points[j + 1])
							swap(Points[j], Points[j + 1]);
					}
				}
				SecondP(n) = Points[1];
				ThirdP(n) = Points[2];
				Points.clear();
			}
			Inter2DP = (SecondP + ThirdP) / 2;
		}
	}	
	else
		Intersection2D = false;
}

void LineSegmentsIntersection3D::SolveCubicEquation(float &a, float &b, float &c, float &d)
{
	/************************************************************************/
	/* Shengjin formula to solve the solution of cubic equation
	   ¦Äf=B^2-4AC
		   Here we only solve real roots
	*/
	/************************************************************************/
	float A = b * b - 3 * a*c;
	float B = b * c - 9 * a*d;
	float C = c * c - 3 * b*d;
	float f = B * B - 4 * A*C;
	float i_value;
	float Y1, Y2;

	if (fabs(a) < 1e-6)  //if a == 0, solve the quadratic equation
	{
		float t1, t2;
		if (fabs(b) < epsilon_coQuadratic)
		{
			if (-d / c >= 0 && -d / c <= 1)
				X123.push_back(-d / c);
		}
		else
		{
			float D = c * c - 4 * b * d;
			if (D < 0)
				//foundAnysolutions = false;
				X123.clear();

			if (D == 0)
			{
				t1 = -c / (2 * b);
				if (t1 >= 0 && t1 <= 1)
					X123.push_back(t1);
			}
			if (D > 0)
			{
				t1 = (-c + sqrt(D)) / (2 * b);
				t2 = (-c - sqrt(D)) / (2 * b);
				if (t1 >= 0 && t1 <= 1)
					X123.push_back(t1);
				if (t2 >= 0 && t2 <= 1)
					X123.push_back(t2);
			}

		}
		return;
	}

	if (fabs(A) < 1e-6 && fabs(B) < 1e-6)//formula 1
	{
		X123.push_back(-b / (3 * a));
		X123.push_back(-b / (3 * a));
		X123.push_back(-b / (3 * a));
	}
	else if (fabs(f) < 1e-6)   //formula 3
	{
		float K = B / A;
		X123.push_back(-b / a + K);
		X123.push_back(-K / 2);
		X123.push_back(-K / 2);
	}
	else if (f > 1e-6)      //formula 2
	{
		Y1 = A * b + 3 * a*(-B + sqrt(f)) / 2;
		Y2 = A * b + 3 * a*(-B - sqrt(f)) / 2;
		float Y1_value = (Y1 / fabs(Y1))*pow((float)fabs(Y1), 1.0 / 3);
		float Y2_value = (Y2 / fabs(Y2))*pow((float)fabs(Y2), 1.0 / 3);
		X123.push_back((-b - Y1_value - Y2_value) / (3 * a));  //We don't need imaginary root
		i_value = sqrt(3.0) / 2 * (Y1_value - Y2_value) / (3 * a);
		if (fabs(i_value) < 1e-1)
		{
			X123.push_back((-b + 0.5*(Y1_value + Y2_value)) / (3 * a));
		}
	}
	else if (f < -1e-6)   //formula 4
	{
		float T = (2 * A*b - 3 * a*B) / (2 * A*sqrt(A));
		float S = acos(T);
		X123.push_back((-b - 2 * sqrt(A)*cos(S / 3)) / (3 * a));
		X123.push_back((-b + sqrt(A)*(cos(S / 3) + sqrt(3.0)*sin(S / 3))) / (3 * a));
		X123.push_back((-b + sqrt(A)*(cos(S / 3) - sqrt(3.0)*sin(S / 3))) / (3 * a));
	}
}

void LineSegmentsIntersection3D::HandleNonDegenerateCase()
{
	/*
	This function to deal with the general situation
	1. compute the coefficient of time
	2. calculate the cubic equation
	3. Bring the solution set into the function to verification of the line segements intersection.
	4. take the smallest time as the finial result(first collision time).
	*/

	//multiset <float> pre_solutionSet;
	//multiset <float> solutionSet;    // Final solution set of time
	isNonDegenerate = false;
	pre_solutionSet.clear();
	ComputeCubicCoefficient();

	SolveCubicEquation(c_0, c_1, c_2, c_3);

	vector<float>::iterator element;
	for (auto element = X123.cbegin(); element != X123.cend(); ++element)
	{
		if (1.0e-6 < *element && *element <= 1)
			pre_solutionSet.push_back(*element);
	}
	
	vector<float>::iterator t;
	for (auto t = pre_solutionSet.cbegin(); t != pre_solutionSet.cend(); ++t)
	{
		if (StaticSegmentSegmentIntersection(*t))
		{
			solutionSet.insert(*t);
			isNonDegenerate = true;
		}
	}
	if (isNonDegenerate)
	{
		Tk = *solutionSet.begin();
		StaticSegmentSegmentIntersection(Tk);  // get the right values of u and v
		
		//whether colinear overlap;
		Vector3f p0 = line_Point0 + vel_P0 * Tk;
		Vector3f p1 = line_Point1 + vel_P1 * Tk;
		Vector3f p2 = line_Point2 + vel_P2 * Tk;
		Vector3f p3 = line_Point3 + vel_P3 * Tk;
		Vector3f P = (p1 - p0).cross(p3 - p2);
		if (fabs(P(0)) < epsilon_coQuadratic && fabs(P(1)) < epsilon_coQuadratic && fabs(P(2)) < epsilon_coQuadratic)
		{
			Vector3f SecondP;
			Vector3f ThirdP;

			for (int n = 0; n < 3; ++n)
			{
				vector <float> Points{ p0(n), p1(n), p2(n), p3(n) };
				for (int i = 0; i < Points.size() - 1; ++i)
				{
					for (int j = 0; j < Points.size() - 1 - i; ++j)
					{
						if (Points[j] > Points[j + 1])
							swap(Points[j], Points[j + 1]);
					}
				}
				SecondP(n) = Points[1];
				ThirdP(n) = Points[2];
				Points.clear();
			}
			NonDegenerateP = (SecondP + ThirdP) / 2;
		}
		else
		{	
			NonDegenerateP = line_Point0 + vel_P0 * Tk + (line_Point1 + vel_P1 * Tk - line_Point0 - vel_P0 * Tk)*u;
		}
	}
}




void LineSegmentsIntersection3D::HandleGeneralCase()
{
	
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
	glLineWidth(8.0);
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);

	glVertex3f(line_Point0(0) / 20, line_Point0(1) / 20, line_Point0(2) / 20);
	glVertex3f(line_Point1(0) / 20, line_Point1(1) / 20, line_Point1(2) / 20);
	
	glVertex3f((line_Point0 + vel_P0)(0) / 20, (line_Point0 + vel_P0)(1) / 20, (line_Point0 + vel_P0)(2) / 20);
	glVertex3f((line_Point1 + vel_P1)(0) / 20, (line_Point1 + vel_P1)(1) / 20, (line_Point1 + vel_P1)(2) / 20);

	glEnd();

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
	glLineWidth(8.0);
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);


	glVertex3f(line_Point2(0) / 20, line_Point2(1) / 20, line_Point2(2) / 20);
	glVertex3f(line_Point3(0) / 20, line_Point3(1) / 20, line_Point3(2) / 20);
	
	glVertex3f((line_Point2 + vel_P2)(0) / 20, (line_Point2 + vel_P2)(1) / 20, (line_Point2 + vel_P2)(2) / 20);
	glVertex3f((line_Point3 + vel_P3)(0) / 20, (line_Point3 + vel_P3)(1) / 20, (line_Point3 + vel_P3)(2) / 20);

	glEnd();

	//Connection start pose and end pose
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
	glLineWidth(1.0);
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(line_Point0(0) / 20, line_Point0(1) / 20, line_Point0(2) / 20);
	glVertex3f((line_Point0 + vel_P0)(0) / 20, (line_Point0 + vel_P0)(1) / 20, (line_Point0 + vel_P0)(2) / 20);
	glVertex3f(line_Point1(0) / 20, line_Point1(1) / 20, line_Point1(2) / 20);
	glVertex3f((line_Point1 + vel_P1)(0) / 20, (line_Point1 + vel_P1)(1) / 20, (line_Point1 + vel_P1)(2) / 20);

	glColor3f(1, 0, 0);
	glLineWidth(1.0);
	glBegin(GL_LINES);
	glVertex3f(line_Point2(0) / 20, line_Point2(1) / 20, line_Point2(2) / 20);
	glVertex3f((line_Point2 + vel_P2)(0) / 20, (line_Point2 + vel_P2)(1) / 20, (line_Point2 + vel_P2)(2) / 20);
	glVertex3f(line_Point3(0) / 20, line_Point3(1) / 20, line_Point3(2) / 20);
	glVertex3f((line_Point3 + vel_P3)(0) / 20, (line_Point3 + vel_P3)(1) / 20, (line_Point3 + vel_P3)(2) / 20);

	glEnd();


	


	
	if (LineSegmentsdegenerate())
	{
		cout << "error case" << endl;
		//cout << "One or both of the line segments degenerate into one point.";
		return;
	}
	if (StartPointIdentical())
	{
		//cout << "The movement of two points from the start  to the end always identical.";
		cout << StartP(0)<<' '<< StartP(1) <<' '<< StartP(2) <<' '<< 0 << endl;
		return;
	}

	else if (AreVectorsPermanentlyParallel())
	{
		//cout << "Two line segments are permanenetly parallel." << endl;
		if (IsPointPermanentlyOnLine())
		{
			//cout << "Two line segments are permanently co-linear." << endl;
			HandleParallelCoLinear();
			if (movingIntersection)
			{
				//cout << "Two line segments collide at time t = " << Tk << endl;
				if (Tk >= 0 && Tk <= 1)
				{
					//cout << "The collide point is:" << endl;
					cout << ParallelP(0) <<' '<< ParallelP(1) <<' '<< ParallelP(2) <<' '<< Tk << endl;

					glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
					glColor3f(0, 0, 0);
					glBegin(GL_LINES);

					glVertex3f((line_Point0 + vel_P0 * Tk)(0) / 20, (line_Point0 + vel_P0 * Tk)(1) / 20, (line_Point0 + vel_P0 * Tk)(2) / 20);
					glVertex3f((line_Point1 + vel_P1 * Tk)(0) / 20, (line_Point1 + vel_P1 * Tk)(1) / 20, (line_Point1 + vel_P1 * Tk)(2) / 20);
					glVertex3f((line_Point2 + vel_P2 * Tk)(0) / 20, (line_Point2 + vel_P2 * Tk)(1) / 20, (line_Point2 + vel_P2 * Tk)(2) / 20);
					glVertex3f((line_Point3 + vel_P3 * Tk)(0) / 20, (line_Point3 + vel_P3 * Tk)(1) / 20, (line_Point3 + vel_P3 * Tk)(2) / 20);

					glEnd();

					glColor3f(0, 1, 0);
					glPointSize(6.0f);
					glBegin(GL_POINTS);

					glVertex3f(ParallelP(0) / 20, ParallelP(1) / 20, ParallelP(2) / 20);

					glEnd();

				}		
			}		
			else
				cout << "Two line segments do not collide in one timestep."<< endl;

		}
		else
		{
			//cout << "Two line segments are not permanently co-linear." << endl;
			HandleParallelNonDegenerateCase();
			if (existSolution)
			{
				//cout << "Two line segments are overlap at time t = " << Tk << endl;
				//cout << "The midpoint of overlap is: " << endl;
				cout << ParallelP(0) <<' '<< ParallelP(1) <<' '<< ParallelP(2) <<' '<< Tk << endl;

				glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
				glColor3f(0, 0, 0);
				glBegin(GL_LINES);

				glVertex3f((line_Point0 + vel_P0 * Tk)(0) / 20, (line_Point0 + vel_P0 * Tk)(1) / 20, (line_Point0 + vel_P0 * Tk)(2) / 20);
				glVertex3f((line_Point1 + vel_P1 * Tk)(0) / 20, (line_Point1 + vel_P1 * Tk)(1) / 20, (line_Point1 + vel_P1 * Tk)(2) / 20);
				glVertex3f((line_Point2 + vel_P2 * Tk)(0) / 20, (line_Point2 + vel_P2 * Tk)(1) / 20, (line_Point2 + vel_P2 * Tk)(2) / 20);
				glVertex3f((line_Point3 + vel_P3 * Tk)(0) / 20, (line_Point3 + vel_P3 * Tk)(1) / 20, (line_Point3 + vel_P3 * Tk)(2) / 20);

				glEnd();

				glColor3f(0, 1, 0);
				glPointSize(6.0f);
				glBegin(GL_POINTS);

				glVertex3f(ParallelP(0) / 20, ParallelP(1) / 20, ParallelP(2) / 20);

				glEnd();

			}
			else
				cout << "Two line segments are not overlap in one timetep." << endl;
		}

		

		return;

	}
	else if (MovingCommenPlane())
	{
		//cout << "Two line segments are moving on a plane." << endl;

		Handle2DCase(line_Point0, line_Point1, vel_P0, vel_P1,
			line_Point2, line_Point3, vel_P2, vel_P3);
		if (Intersection2D)
		{
			//cout << "Two line segments  intersect at time t = " << Tk << endl;
			//cout << "u = " << u << "    " << "v = " << v << endl;
			//cout << "The collide point is:" << endl;
			cout << Inter2DP(0) << ' ' << Inter2DP(1) << ' ' << Inter2DP(2) << ' ' << Tk << endl;

			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
			glColor3f(0, 0, 0);
			glBegin(GL_LINES);

			glVertex3f((line_Point0 + vel_P0 * Tk)(0) / 20, (line_Point0 + vel_P0 * Tk)(1) / 20, (line_Point0 + vel_P0 * Tk)(2) / 20);
			glVertex3f((line_Point1 + vel_P1 * Tk)(0) / 20, (line_Point1 + vel_P1 * Tk)(1) / 20, (line_Point1 + vel_P1 * Tk)(2) / 20);
			glVertex3f((line_Point2 + vel_P2 * Tk)(0) / 20, (line_Point2 + vel_P2 * Tk)(1) / 20, (line_Point2 + vel_P2 * Tk)(2) / 20);
			glVertex3f((line_Point3 + vel_P3 * Tk)(0) / 20, (line_Point3 + vel_P3 * Tk)(1) / 20, (line_Point3 + vel_P3 * Tk)(2) / 20);

			glEnd();

			glColor3f(0, 1, 0);
			glPointSize(6.0f);
			glBegin(GL_POINTS);

			glVertex3f(Inter2DP(0) / 20, Inter2DP(1) / 20, Inter2DP(2) / 20);

			glEnd();

		}		
		else
		{
			cout << "Two line segments do not intersect in one timestep. " << endl;
			//cout << "u = " << u << "    " << "v = " << v << endl;
		}

		

		return;
			
	}
	
	else
	{
		//cout << "Two line segments are moving not on plane" << endl;

		HandleNonDegenerateCase();

		if (isNonDegenerate)
		{
			//cout << "Two line segments are intersect at time t =  " << Tk << endl;
			//cout << "u = " << u << "    " << "v = " << v << endl;
			//cout << "The collide point is:" << endl;
			cout << NonDegenerateP(0) << ' ' << NonDegenerateP(1) << ' '<< NonDegenerateP(2) << "     "<< Tk << endl;
			
			

			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
			glColor3f(0, 0, 0);
			glBegin(GL_LINES);

			glVertex3f((line_Point0 + vel_P0 * Tk)(0) / 20, (line_Point0 + vel_P0 * Tk)(1) / 20, (line_Point0 + vel_P0 * Tk)(2) / 20);
			glVertex3f((line_Point1 + vel_P1 * Tk)(0) / 20, (line_Point1 + vel_P1 * Tk)(1) / 20, (line_Point1 + vel_P1 * Tk)(2) / 20);

			glColor3f(0, 0, 0);
			glBegin(GL_LINES);
			glVertex3f((line_Point2 + vel_P2 * Tk)(0) / 20, (line_Point2 + vel_P2 * Tk)(1) / 20, (line_Point2 + vel_P2 * Tk)(2) / 20);
			glVertex3f((line_Point3 + vel_P3 * Tk)(0) / 20, (line_Point3 + vel_P3 * Tk)(1) / 20, (line_Point3 + vel_P3 * Tk)(2) / 20);

			glEnd();

			glColor3f(0, 1, 0);
			glPointSize(6.0f);
			glBegin(GL_POINTS);

			glVertex3f(NonDegenerateP(0) / 20, NonDegenerateP(1) / 20, NonDegenerateP(2) / 20);

			glEnd();

		}
		else
			cout << "Two line segments do not intersect in one timestep " << endl;

		
			
		return;
	}
}

 