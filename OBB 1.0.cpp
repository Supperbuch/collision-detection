#include"OBB 1.0.h"
#include <cmath>
#include <cfloat>
#include <list>

#define CMP(x, y) \
	(fabsf(x - y) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))


OBB::OBB(Vector3f CP1, Vector3f CP2, Quaternionf q1, Quaternionf q2, float l, float w, float h)
{
	position1 = CP1;
	position2 = CP2;
	quaternion1 = q1;
	quaternion2 = q2;
	Length = l;
	Width = w;
	Height = h;
	orientation1 = quaternion1.matrix();
	orientation2 = quaternion2.matrix();
	//cout << orientation1 << endl;

	x << 2, 0, 0;
	y << 0, 2, 0;
	z << 0, 0, 2;
}

bool OBB::PointInOBB(const Vector3f& point, Vector3f &p, Matrix3f &o, float &l, float &w, float &h)
{
	//determine whether a point is in bounding box.
	Vector3f dir = point - p;
	for (int i = 0; i < 3; ++i)
	{
		Vector3f axis(o(0, i), o(1, i), o(2, i));

		float distance = dir.dot(axis);
		Vector3f size(l/2, w/2, h/2);
		if (fabs(distance) - size(i) > 50.0e-6)
			return false;
		
		/*if (distance > size(i))
			return false;
		if (distance < - size(i))
			return false;*/
	}
	
	return true;
}

VectorXf OBB::GetInterval(Vector3f &p1, Matrix3f &o1, float &l1, float &w1, float &h1, Vector3f &p2, Matrix3f &o2, float &l2, float &w2, float &h2, const Vector3f& axis)
{
	//project vertices to 15 axis, find the two points with the furthest distance
	Vector3f vertex[8];

	Vector3f C = p1;
	Vector3f E(l1 / 2, w1 / 2, h1 / 2);
	Matrix3f A = o1;
	Vector3f A1(A(0, 0), A(1, 0), A(2, 0));
	Vector3f A2(A(0, 1), A(1, 1), A(2, 1));
	Vector3f A3(A(0, 2), A(1, 2), A(2, 2));

	vertex[0] = C + A1 * E(0) + A2 * E(1) + A3 * E(2);
	vertex[1] = C - A1 * E(0) + A2 * E(1) + A3 * E(2);
	vertex[2] = C + A1 * E(0) - A2 * E(1) + A3 * E(2);
	vertex[3] = C + A1 * E(0) + A2 * E(1) - A3 * E(2);
	vertex[4] = C - A1 * E(0) - A2 * E(1) - A3 * E(2);
	vertex[5] = C + A1 * E(0) - A2 * E(1) - A3 * E(2);
	vertex[6] = C - A1 * E(0) + A2 * E(1) - A3 * E(2);
	vertex[7] = C - A1 * E(0) - A2 * E(1) + A3 * E(2);


	//vector<Vector3f> allPoints;
	//allPoints.reserve(contact.size() + 8);
	//allPoints.insert(allPoints.end(), contacts.begin(), contacts.end());
	//
	////judge whether the vertices are in another cuboid， 
	//vector<Vector3f> x = GetVertices(p1, o1, l1, w1, h1);
	//
	//for (vector<Vector3f> ::iterator iter = x.begin(); iter != x.end(); ++iter)
	//{
	//	if (PointInOBB(*iter, p2, o2, l2, w2, h2))
	//	{
	//		allPoints.push_back(*iter);
	//	}
	//}

	//VectorXf result(2);

	////result interval of vertices (0):min   result(1): max
	//VectorXf result1(2);   
	//result1 << axis.dot(vertex[0]), axis.dot(vertex[0]);

	//Vector3f vMax;
	//Vector3f vMin;
	//for (int i = 1; i < 8; ++i)
	//{
	//	float projection = axis.dot(vertex[i]);
	//	if (projection < result1(0))
	//	{
	//		result1(0) = projection;
	//		vMin = vertex[i];
	//	}	
	//	if (projection > result1(1))
	//	{
	//		result1(1) = projection;
	//		vMax = vertex[i];
	//	}
	//		
	//}

	////result interval of contact points(contain vertices in another cubiod
	//VectorXf result2(2);
	//result2 << axis.dot(allPoints[0]), axis.dot(allPoints[0]);

	//Vector3f cMax;
	//Vector3f cMin;
	//for (vector<Vector3f> ::iterator iter = allPoints.begin()+1; iter != allPoints.end(); ++iter)
	//{
	//	float projection = axis.dot(*iter);
	//	if (projection < result2(0))
	//	{
	//		result2(0) = projection;
	//		cMin = *iter;
	//	}
	//	if (projection > result2(1))
	//	{
	//		result2(1) = projection;
	//		cMax = *iter;
	//	}
	//}
	////project center point o2 to axis
	//float o1_project = axis.dot(p1);
	//float o2_project = axis.dot(p2);

	//result << result1(0), result2(1);


	VectorXf result(2);   //result(0):min   result(1): max
	result << axis.dot(vertex[0]), axis.dot(vertex[0]);

	for (int i = 1; i < 8; ++i)
	{
		float projection = axis.dot(vertex[i]);
		result(0) = (projection < result(0)) ? projection : result[0];
		result(1) = (projection > result(1)) ? projection : result[1];
	}

	return result;	
}



vector<Vector3f> OBB::GetVertices(Vector3f &p, Matrix3f &o, float &l, float &w, float &h)
{
	//get the coordinates of each vertex
	vector<Vector3f> v;
	v.resize(8);
	Vector3f C = p;
	Vector3f E(l / 2, w / 2, h / 2);
	Matrix3f A = o;
	Vector3f A1(A(0, 0), A(1, 0), A(2, 0));
	Vector3f A2(A(0, 1), A(1, 1), A(2, 1));
	Vector3f A3(A(0, 2), A(1, 2), A(2, 2));

	v[0] = C + A1 * E(0) + A2 * E(1) + A3 * E(2);
	v[1] = C - A1 * E(0) + A2 * E(1) + A3 * E(2);
	v[2] = C + A1 * E(0) - A2 * E(1) + A3 * E(2);
	v[3] = C + A1 * E(0) + A2 * E(1) - A3 * E(2);
	v[4] = C - A1 * E(0) - A2 * E(1) - A3 * E(2);
	v[5] = C + A1 * E(0) - A2 * E(1) - A3 * E(2);
	v[6] = C - A1 * E(0) + A2 * E(1) - A3 * E(2);
	v[7] = C - A1 * E(0) - A2 * E(1) + A3 * E(2);

	return v;
}

vector<MatrixXf> OBB::GetEdges(Vector3f &p, Matrix3f &o, float &l, float &w, float &h)
{
	//get edges of bounding box
	/********************
	  Matrix(3,2) = (startpoint, endpoint)
	*********************/
	vector<MatrixXf> result;
	result.reserve(12);
	vector<Vector3f> x = GetVertices(p, o, l, w, h);
	int index[][2] = { // Indices of edges
		{ 6, 1 },{ 6, 3 },{ 6, 4 },{ 2, 7 },{ 2, 5 },{ 2, 0 },
		{ 0, 1 },{ 0, 3 },{ 7, 1 },{ 7, 4 },{ 4, 5 },{ 5, 3 }
	};

	for (int j = 0; j < 12; ++j)
	{
		MatrixXf A(3,2);
		A << x[index[j][0]], x[index[j][1]];
		result.push_back(A);
	}
	return result;
}

vector<VectorXf> OBB::GetPlanes(Vector3f &p, Matrix3f &o, float &l, float &w, float &h)
{
	//get planes of bounding box
	/*
	Vector plane = ( normal, 
	*/
	Vector3f c = p;
	Vector3f e(l / 2, w / 2, h / 2);
	Matrix3f a = o;
	Vector3f a1(a(0, 0), a(1, 0), a(2, 0));
	Vector3f a2(a(0, 1), a(1, 1), a(2, 1));
	Vector3f a3(a(0, 2), a(1, 2), a(2, 2));

	vector<VectorXf> result;
	result.resize(6);

	//project center point of each face on its normal vector
	VectorXf P1(4), P2(4), P3(4), P4(4), P5(4), P6(4);
	P1 << a1, a1.dot(c + a1 * e(0));
	P2 << a1 * -1.0f, - a1.dot(c - a1 * e(0));
	P3 << a2, a2.dot(c + a2 * e(1));
	P4 << a2 * -1.0f, - a2.dot(c - a2 * e(1));
	P5 << a3, a3.dot(c + a3 * e(2));
	P6 << a3 * -1.0f, - a3.dot(c - a3 * e(2));

	result[0] = P1;
	result[1] = P2;
	result[2] = P3;
	result[3] = P4;
	result[4] = P5;
	result[5] = P6;

	return result;
}

bool OBB::ClipToPlane(const VectorXf& plane, const MatrixXf& line, Vector3f* outPoint)
{
	/*This function checks if a line intersects a plane and if it does, 
	  the line is clipped to the plane*/
	Vector3f s(line(0, 0), line(1, 0), line(2, 0));
	Vector3f e(line(0, 1), line(1, 1), line(2, 1));
	Vector3f ab = e - s;
	//Vector3f ab = s - e;
	

	Vector3f normal(plane(0), plane(1), plane(2));
	float nA = normal.dot(s);
	float nAB = normal.dot(ab);

	if (CMP(nAB, 0)) //ensure that the line and plane intersect
	{
		return false;
	}

	float distance = plane(3);
	float t = (distance - nA) / nAB;
	//cout << t << endl;

	if (t >= 0.0f && t <= 1.0f)
	{
		if (outPoint != 0) {
			//the point at which the line and plane intersect
			*outPoint = s + ab *  t;
			//cout << *outPoint << endl;
		}
		return true;
	}

	return false;

}


vector<Vector3f> OBB::ClipEdgesToOBB(const vector<MatrixXf>& edges, Vector3f &p, Matrix3f &o, float &l, float &w, float &h)
{
	//record the resulting point, if the edge and plane intersect.
	vector<Vector3f> result;
	result.reserve(edges.size() * 3);

	Vector3f intersection; //If the edge and plane intersect, record the resulting point
	vector<VectorXf> planes = GetPlanes(p, o, l, w, h);

	for (int i = 0; i < planes.size(); ++i)
	{
		for (int j = 0; j < edges.size(); ++j)
		{
			if (ClipToPlane(planes[i], edges[j], &intersection))
			{
				/*bool B = ClipToPlane(planes[i], edges[j], &intersection);
				cout << B << endl;*/

				if (PointInOBB(intersection, p, o, l, w, h))
				{
					result.push_back(intersection);
				}
				/*bool A = PointInOBB(intersection, p, o, l, w, h);
				cout << A << endl;*/
			}
		}
	}


	return result;

}

float OBB::PenetrationDepth(vector<Vector3f> &contact1, vector<Vector3f> &contact2, Vector3f &p1, Matrix3f &o1, float &l1, float &w1, float &h1, Vector3f &p2, Matrix3f &o2, float &l2, float &w2, float &h2, const Vector3f &axis, bool* outShouldFlip)
{
	VectorXf i1 = GetInterval(p1, o1, l1, w1, h1, p2, o2, l2, w2, h2, axis / (axis.norm()));
	VectorXf i2 = GetInterval(p2, o2, l2, w2, h2, p1, o1, l1, w1, h1, axis / (axis.norm()));

	if (!((i2(0) <= i1(1)) && (i1(0) <= i2(1))))
	{
		return 0.0f;
	}

	float result;

	//judge whether interval of one cubiod contain another one.
	if ((i1(1) > i2(1) && i1(0) < i2(0)) || (i2(1) > i1(1) && i2(0) < i1(0)))
	{

		if (outShouldFlip != 0)
		{
			*outShouldFlip = (i2(0) < i1(0));
		}

		result = ((i2(1) - i1(0)) < (i1(1) - i2(0))) ? (i2(1) - i1(0)) : (i1(1) - i2(0));
		return result;
	}
	
	float len1 = i1(1) - i1(0);
	float len2 = i2(1) - i2(0);
	float min = fminf(i1(0), i2(0));
	float max = fmaxf(i1(1), i2(1));
	float length = max - min;

	if (outShouldFlip != 0)
	{
		*outShouldFlip = (i2(0) < i1(0));
	}

	result = (len1 + len2) - length;

	return result;

}

void OBB::FindCollisionFeatures()
{
	
	colliding = false;
	normal << 0, 0, 1;
	depth = FLT_MAX;
	if (contacts.size() > 0)
		contacts.clear();


	vector<Vector3f> c1 = ClipEdgesToOBB(GetEdges(position2, orientation2, Length, Width, Height), position1, orientation1, Length, Width, Height);
	vector<Vector3f> c2 = ClipEdgesToOBB(GetEdges(position1, orientation1, Length, Width, Height), position2, orientation2, Length, Width, Height);
	contacts.reserve(c1.size() + c2.size());
	contacts.insert(contacts.end(), c1.begin(), c1.end());
	contacts.insert(contacts.end(), c2.begin(), c2.end());

	if (contacts.empty())
	{
		cout << "not colliding" << endl;
		return;
	}

	/*vector<Vector3f>::iterator M;
	for (auto M = c1.begin(); M != c1.end(); ++M)
	{
		cout << (*M).transpose() << endl;	
	}*/

	Matrix3f A = orientation1;
	Vector3f A1(A(0, 0), A(1, 0), A(2, 0));
	Vector3f A2(A(0, 1), A(1, 1), A(2, 1));
	Vector3f A3(A(0, 2), A(1, 2), A(2, 2));

	Matrix3f B = orientation2;
	Vector3f B1(B(0, 0), B(1, 0), B(2, 0));
	Vector3f B2(B(0, 1), B(1, 1), B(2, 1));
	Vector3f B3(B(0, 2), B(1, 2), B(2, 2));

	Vector3f test[15] = {
		A1,A2,A3,B1,B2,B3
	};

	for (int i = 0; i < 3; ++i)
	{
		/*There is an error in Cookbook, page 376. cross product of axes
		  test[3], test[4], test[5], instead of test[0], test[1], test[2]
		*/
		test[6 + i * 3 + 0] = test[i].cross(test[3]);
		test[6 + i * 3 + 1] = test[i].cross(test[4]);
		test[6 + i * 3 + 2] = test[i].cross(test[5]);
	}

	//draw all axis(15)
	/*for (int j = 0; j < 15; ++j)
	{
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glColor3f(0, 1, 1);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(test[j](0), test[j](1), test[j](2));
		glEnd();
	}*/

	//create a temporary variable for the direction of the collision normal
	Vector3f *hitNormal = 0;
	bool shouldFlip;

	for (int i = 0; i < 15; ++i)
	{
		if (fabs(test[i](0)) < 0.000001f) test[i](0) = 0.0f;
		if (fabs(test[i](1)) < 0.000001f) test[i](1) = 0.0f;
		if (fabs(test[i](2)) < 0.000001f) test[i](2) = 0.0f;
		if (test[i].dot(test[i]) < 0.001f)
		{
			continue;
		}

		float depth1 = PenetrationDepth(c1, c2, position1, orientation1, Length, Width, Height, position2, orientation2, Length, Width, Height, test[i], &shouldFlip);
		if (depth1 < 0.0f)
		{
			cout << "not colliding" << endl;
			return;
		}
		else if (depth1 < depth)
		{
			if (shouldFlip)
			{
				test[i] = test[i] * -1.0f;
			}
			depth = depth1;
			hitNormal = &test[i];
		}

	}

	if (hitNormal == 0)
	{
		//If no collision normal was found, the OBBs do not intersect
		cout << " not colliding" << endl;
		return;
	}

	Vector3f axis = (*hitNormal / (*hitNormal).norm());

	//interval of one cubiod contain another one.
	VectorXf i1 = GetInterval(position1, orientation1, Length, Width, Height, position2, orientation2, Length, Width, Height, axis);
	VectorXf i2 = GetInterval(position2, orientation2, Length, Width, Height, position1, orientation1, Length, Width, Height, axis);

	Vector3f pointOnPlane;
	if (i1(1) > i2(1) && i1(0) < i2(0)) // i1 contain i2
	{
		float distance = (i2(1) - i2(0)) * 0.5f;
		//Vector3f pointOnPlane = position2 + axis * distance;
		pointOnPlane = position2;
	}

	else if (i2(1) > i1(1) && i2(0) < i1(0)) //i2 contain i1
	{
		float distance = (i1(1) - i1(0)) * 0.5f;
		//Vector3f pointOnPlane = position1 + axis * distance;
		pointOnPlane = position1;
	}

	else
	{
		VectorXf i = GetInterval(position1, orientation1, Length, Width, Height, position2, orientation2, Length, Width, Height, axis);
		float distance = (i(1) - i(0)) * 0.5f - depth * 0.5f;
		pointOnPlane = position1 + axis * distance;
	}

	//project contact points to shared plane
	for (int i = contacts.size() - 1; i >= 0; --i)
	{
		Vector3f contact = contacts[i];
		contacts[i] = contact + (axis * axis.dot(pointOnPlane - contact));

		for (int j = contacts.size() - 1; j > i; --j)
		{
			if ((contacts[j] - contacts[i]).dot(contacts[j] - contacts[i]) < 0.0001f)
			{
				contacts.erase(contacts.begin() + j);
				break;
			}
		}
	}

	colliding = true;
	normal = axis;

	cout << colliding << endl;
	cout << "normal:" << normal.transpose() << endl;
	cout << "depth: "<< depth << endl;

	//draw normal
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glColor3f(1, 0, 1);
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(normal(0) * 2, normal(1) * 2, normal(2) * 2);
	glVertex3f(0, 0, 0);
	glVertex3f(-normal(0) * 2, -normal(1) * 2, -normal(2) * 2);
	glEnd();


	//draw contact points
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
	glColor3f(0, 0, 0);
	glPointSize(6.0f);
	glBegin(GL_POINTS);

	vector<Vector3f>::iterator C;
	for (auto C = contacts.begin(); C != contacts.end(); ++C)
	{
		cout << (*C).transpose() << endl;
		glVertex3f((*C)(0) / 20, (*C)(1) / 20, (*C)(2) / 20);
	}

	glEnd();
}

VectorXf OBB::result()
{
	FindCollisionFeatures();
	VectorXf A(4);
	if (depth > 0)
	{
		A(0) = depth;
		A(1) = normal(0);
		A(2) = normal(1);
		A(3) = normal(2);
	}
	else
	{
		A(0) = 0;
		A(1) = normal(0);
		A(2) = normal(1);
		A(3) = normal(2);
	}
	return A;
}

bool OBB::CorrectNormal(Vector3f &axis)
{
	VectorXf i1 = GetInterval(position1, orientation1, Length, Width, Height, position2, orientation2, Length, Width, Height, axis / (axis.norm()));
	VectorXf i2 = GetInterval(position2, orientation2, Length, Width, Height, position1, orientation1, Length, Width, Height, axis / (axis.norm()));

	//judge whether the projection intervals of two objects overlap
	if ((i1(1) - i2(0)) < 10.0e-6 || (i2(1) - i1(0)) < 10.0e-6)  //not overlap
		return true;
	else
		return false;
}


void OBB::projectToNormal(Vector3f &p, Matrix3f &o, Vector3f &normal, Vector3f &pointColor)
{
	Vector3f vertex1[8];

	Vector3f C = p;
	Vector3f E(0.5*Length, 0.5*Width, 0.5*Height);
	Matrix3f A = o;
	Vector3f A1(A(0, 0), A(1, 0), A(2, 0));
	Vector3f A2(A(0, 1), A(1, 1), A(2, 1));
	Vector3f A3(A(0, 2), A(1, 2), A(2, 2));

	vertex1[0] = C + A1 * E(0) + A2 * E(1) + A3 * E(2);
	vertex1[1] = C - A1 * E(0) + A2 * E(1) + A3 * E(2);
	vertex1[2] = C + A1 * E(0) - A2 * E(1) + A3 * E(2);
	vertex1[3] = C + A1 * E(0) + A2 * E(1) - A3 * E(2);
	vertex1[4] = C - A1 * E(0) - A2 * E(1) - A3 * E(2);
	vertex1[5] = C + A1 * E(0) - A2 * E(1) - A3 * E(2);
	vertex1[6] = C - A1 * E(0) + A2 * E(1) - A3 * E(2);
	vertex1[7] = C - A1 * E(0) - A2 * E(1) + A3 * E(2);

	//get the interval of two project point
	Vector3f Ob1Min;
	Vector3f Ob1Max;

	VectorXf result(2);   //result(0):min   result(1): max
	result << normal.dot(vertex1[0]), normal.dot(vertex1[0]);
	Ob1Min = normal.dot(vertex1[0]) * normal;
	Ob1Max = normal.dot(vertex1[0]) * normal;

	for (int i = 1; i < 8; ++i)
	{
		float projection = normal.dot(vertex1[i]);
		if (projection < result(0))
		{
			Ob1Min = normal.dot(vertex1[i]) * normal;
			result(0) = projection;
		}
		if (projection > result(1))
		{
			Ob1Max = normal.dot(vertex1[i]) * normal;
			result(1) = projection;
		}

		/*Ob1Min = (projection < result(0)) ? normal.dot(vertex1[i]) * normal : Ob1Min;
		Ob1Max = (projection > result(1)) ? normal.dot(vertex1[i]) * normal : Ob1Max;*/
	}


	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
	glColor3f(pointColor(0), pointColor(1), pointColor(2));

	//draw line
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glVertex3f(Ob1Min(0) / 20, Ob1Min(1) / 20, Ob1Min(2) / 20);
	glVertex3f(Ob1Max(0) / 20, Ob1Max(1) / 20, Ob1Max(2) / 20);

	glEnd();

	//draw point
	glColor3f(1, 0, 0);
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glVertex3f(Ob1Min(0) / 20, Ob1Min(1) / 20, Ob1Min(2) / 20);
	glVertex3f(Ob1Max(0) / 20, Ob1Max(1) / 20, Ob1Max(2) / 20);

	glEnd();
}

void OBB::drawProjectPoint()
{
	FindCollisionFeatures();
	if (colliding)
	{
		Vector3f color1(0, 0, 1);
		projectToNormal(position1, orientation1, normal, color1);


		Vector3f color2(0, 1, 0);
		//original projection
		/*projectToNormal(position2, orientation2, normal, color2);*/

		//after separate
		Vector3f V(depth*normal(0), depth*normal(1), depth*normal(2));
		Vector3f V1;
		V1 = position2 + V;
		VectorXf i1 = GetInterval(position1, orientation1, Length, Width, Height, position2, orientation2, Length, Width, Height, normal / (normal.norm()));
		VectorXf i2 = GetInterval(V1, orientation2, Length, Width, Height, position1, orientation1, Length, Width, Height, normal / (normal.norm()));
		//judge whether the projection intervals of two objects overlap
		if ((i1(1) - i2(0)) < 10.0e-6 || (i2(1) - i1(0)) < 10.0e-6)  //not overlap
		{
			projectToNormal(V1, orientation2, normal, color2);
		}
		else
		{
			Vector3f V2;
			V2 = position2 - V;
			projectToNormal(V2, orientation2, normal, color2);
		}
	}
}

void OBB::drawBox(Vector3f &color1, Vector3f &color2, float &lineWidth)
{


	vector<Vector3f> V1 = GetVertices(position1, orientation1, Length, Width, Height);
	vector<Vector3f> V2 = GetVertices(position2, orientation2, Length, Width, Height);

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color


	int index[][2] = { // Indices of edges
		{ 6, 1 },{ 6, 3 },{ 6, 4 },{ 2, 7 },{ 2, 5 },{ 2, 0 },
		{ 0, 1 },{ 0, 3 },{ 7, 1 },{ 7, 4 },{ 4, 5 },{ 5, 3 }
	};


	glLineWidth(lineWidth);
	glColor3f(color1(0), color1(1), color1(2));
	glBegin(GL_LINES);

	for (int j = 0; j < 12; ++j)
	{
		glVertex3f(V1[index[j][0]](0) / 20, V1[index[j][0]](1) / 20, V1[index[j][0]](2) / 20);
		glVertex3f(V1[index[j][1]](0) / 20, V1[index[j][1]](1) / 20, V1[index[j][1]](2) / 20);

	}

	glColor3f(color2(0), color2(1), color2(2));
	glBegin(GL_LINES);

	for (int j = 0; j < 12; ++j)
	{
		glVertex3f(V2[index[j][0]](0) / 20, V2[index[j][0]](1) / 20, V2[index[j][0]](2) / 20);
		glVertex3f(V2[index[j][1]](0) / 20, V2[index[j][1]](1) / 20, V2[index[j][1]](2) / 20);
	}

	glEnd();


	//draw the planes with color
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);  // 此处需要禁止深度测试
	glBegin(GL_QUADS);
	glColor4f(0.8, 0.8, 0.8, 0.25);

	glVertex3f(V1[0](0) / 20, V1[0](1) / 20, V1[0](2) / 20);
	glVertex3f(V1[1](0) / 20, V1[1](1) / 20, V1[1](2) / 20);
	glVertex3f(V1[7](0) / 20, V1[7](1) / 20, V1[7](2) / 20);
	glVertex3f(V1[2](0) / 20, V1[2](1) / 20, V1[2](2) / 20);

	glVertex3f(V1[3](0) / 20, V1[3](1) / 20, V1[3](2) / 20);
	glVertex3f(V1[5](0) / 20, V1[5](1) / 20, V1[5](2) / 20);
	glVertex3f(V1[4](0) / 20, V1[4](1) / 20, V1[4](2) / 20);
	glVertex3f(V1[6](0) / 20, V1[6](1) / 20, V1[6](2) / 20);

	glVertex3f(V1[2](0) / 20, V1[2](1) / 20, V1[2](2) / 20);
	glVertex3f(V1[5](0) / 20, V1[5](1) / 20, V1[5](2) / 20);
	glVertex3f(V1[4](0) / 20, V1[4](1) / 20, V1[4](2) / 20);
	glVertex3f(V1[7](0) / 20, V1[7](1) / 20, V1[7](2) / 20);

	glVertex3f(V1[0](0) / 20, V1[0](1) / 20, V1[0](2) / 20);
	glVertex3f(V1[1](0) / 20, V1[1](1) / 20, V1[1](2) / 20);
	glVertex3f(V1[6](0) / 20, V1[6](1) / 20, V1[6](2) / 20);
	glVertex3f(V1[3](0) / 20, V1[3](1) / 20, V1[3](2) / 20);

	glVertex3f(V1[0](0) / 20, V1[0](1) / 20, V1[0](2) / 20);
	glVertex3f(V1[2](0) / 20, V1[2](1) / 20, V1[2](2) / 20);
	glVertex3f(V1[5](0) / 20, V1[5](1) / 20, V1[5](2) / 20);
	glVertex3f(V1[3](0) / 20, V1[3](1) / 20, V1[3](2) / 20);

	glVertex3f(V1[1](0) / 20, V1[1](1) / 20, V1[1](2) / 20);
	glVertex3f(V1[7](0) / 20, V1[7](1) / 20, V1[7](2) / 20);
	glVertex3f(V1[4](0) / 20, V1[4](1) / 20, V1[4](2) / 20);
	glVertex3f(V1[6](0) / 20, V1[6](1) / 20, V1[6](2) / 20);

	glEnd();


	glBegin(GL_QUADS);
	glColor4f(0.8, 0.8, 0.8, 0.25);

	glVertex3f(V2[0](0) / 20, V2[0](1) / 20, V2[0](2) / 20);
	glVertex3f(V2[1](0) / 20, V2[1](1) / 20, V2[1](2) / 20);
	glVertex3f(V2[7](0) / 20, V2[7](1) / 20, V2[7](2) / 20);
	glVertex3f(V2[2](0) / 20, V2[2](1) / 20, V2[2](2) / 20);

	glVertex3f(V2[3](0) / 20, V2[3](1) / 20, V2[3](2) / 20);
	glVertex3f(V2[5](0) / 20, V2[5](1) / 20, V2[5](2) / 20);
	glVertex3f(V2[4](0) / 20, V2[4](1) / 20, V2[4](2) / 20);
	glVertex3f(V2[6](0) / 20, V2[6](1) / 20, V2[6](2) / 20);

	glVertex3f(V2[2](0) / 20, V2[2](1) / 20, V2[2](2) / 20);
	glVertex3f(V2[5](0) / 20, V2[5](1) / 20, V2[5](2) / 20);
	glVertex3f(V2[4](0) / 20, V2[4](1) / 20, V2[4](2) / 20);
	glVertex3f(V2[7](0) / 20, V2[7](1) / 20, V2[7](2) / 20);

	glVertex3f(V2[0](0) / 20, V2[0](1) / 20, V2[0](2) / 20);
	glVertex3f(V2[1](0) / 20, V2[1](1) / 20, V2[1](2) / 20);
	glVertex3f(V2[6](0) / 20, V2[6](1) / 20, V2[6](2) / 20);
	glVertex3f(V2[3](0) / 20, V2[3](1) / 20, V2[3](2) / 20);

	glVertex3f(V2[0](0) / 20, V2[0](1) / 20, V2[0](2) / 20);
	glVertex3f(V2[2](0) / 20, V2[2](1) / 20, V2[2](2) / 20);
	glVertex3f(V2[5](0) / 20, V2[5](1) / 20, V2[5](2) / 20);
	glVertex3f(V2[3](0) / 20, V2[3](1) / 20, V2[3](2) / 20);

	glVertex3f(V2[1](0) / 20, V2[1](1) / 20, V2[1](2) / 20);
	glVertex3f(V2[7](0) / 20, V2[7](1) / 20, V2[7](2) / 20);
	glVertex3f(V2[4](0) / 20, V2[4](1) / 20, V2[4](2) / 20);
	glVertex3f(V2[6](0) / 20, V2[6](1) / 20, V2[6](2) / 20);

	glEnd();
	glDisable(GL_BLEND);

	//draw local coordinate of cubiods
	Vector3f x1;
	Vector3f y1;
	Vector3f z1;
	Vector3f position1s;
	position1s = position1 / 20;
	x1 = (quaternion1 * x + position1) / 20;
	y1 = (quaternion1 * y + position1) / 20;
	z1 = (quaternion1 * z + position1) / 20;

	Vector3f x2;
	Vector3f y2;
	Vector3f z2;
	Vector3f position2s;
	position2s = position2 / 20;
	x2 = (quaternion2 * x + position2) / 20;
	y2 = (quaternion2 * y + position2) / 20;
	z2 = (quaternion2 * z + position2) / 20;

	//coordinate of cubiod1
	glLineWidth(1.0f);
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(x1(0), x1(1), x1(2));
	glVertex3f(position1s(0), position1s(1), position1s(2));

	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(y1(0), y1(1), y1(2));
	glVertex3f(position1s(0), position1s(1), position1s(2));

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(z1(0), z1(1), z1(2));
	glVertex3f(position1s(0), position1s(1), position1s(2));

	//coordinate of cubiod2
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(x2(0), x2(1), x2(2));
	glVertex3f(position2s(0), position2s(1), position2s(2));

	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(y2(0), y2(1), y2(2));
	glVertex3f(position2s(0), position2s(1), position2s(2));

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(z2(0), z2(1), z2(2));
	glVertex3f(position2s(0), position2s(1), position2s(2));

	glEnd();

}


Matrix3f OBB::PointToSegment(Vector3f &A, Vector3f &B, Vector3f &C)
{
	float t, m, n, d, u, v, w;
	m = (B(0) - A(0))*(B(0) - C(0)) + (B(1) - A(1))*(B(1) - C(1)) + (C(2) - B(2))*(A(2) - B(2));
	n = (B(0) - C(0))*(B(0) - C(0)) + (B(1) - C(1))*(B(1) - C(1)) + (C(2) - B(2))*(C(2) - B(2));
	t = m / n;

	Matrix3f TwoSegment;

	if (t < 0)
	{
		d = sqrt((A(0) - B(0)) * (A(0) - B(0)) + (A(1) - B(1)) * (A(1) - B(1)) + (A(2) - B(2)) * (A(2) - B(2)));
		Vector3f N(d, 0, 0);
		TwoSegment << N, A, B;
		return TwoSegment;
	}
	else if (t > 1)
	{
		d = sqrt((A(0) - C(0)) * (A(0) - C(0)) + (A(1) - C(1)) * (A(1) - C(1)) + (A(2) - C(2)) * (A(2) - C(2)));
		Vector3f N(d, 0, 0);
		TwoSegment << N, A, C;
		return TwoSegment;
	}
	else
	{
		u = B(0) + (C(0) - B(0)) * t;
		v = B(1) + (C(0) - B(0)) * t;
		w = B(2) + (C(0) - B(0)) * t;
		d = sqrt((A(0) - u) * (A(0) - u) + (A(1) - v) * (A(1) - v) + (A(2) - w) * (A(2) - w));
		Vector3f P;
		P << u, v, w;
		Vector3f N(d, 0, 0);
		TwoSegment << N, A, P;
		return TwoSegment;
	}
}

void OBB::DistanceOfBoxesVerification()
{
#define EPSILON 0.00001f

	vector<Vector3f> V1 = GetVertices(position1, orientation1, Length, Width, Height);
	vector<Vector3f> V2 = GetVertices(position2, orientation2, Length, Width, Height);

	/***********************************************************/
	//get the normal of each plane
	Matrix3f Ao = orientation1;
	Vector3f A1(Ao(0, 0), Ao(1, 0), Ao(2, 0));
	Vector3f A2(Ao(0, 1), Ao(1, 1), Ao(2, 1));
	Vector3f A3(Ao(0, 2), Ao(1, 2), Ao(2, 2));

	Matrix3f Bo = orientation2;
	Vector3f B1(Bo(0, 0), Bo(1, 0), Bo(2, 0));
	Vector3f B2(Bo(0, 1), Bo(1, 1), Bo(2, 1));
	Vector3f B3(Bo(0, 2), Bo(1, 2), Bo(2, 2));
	/***********************************************************/

	//front plane
	MatrixXf t1(3, 4);
	t1 << V1[0], V1[2], V1[3], A1;
	MatrixXf t2(3, 4);
	t2 << V1[5], V1[2], V1[3], A1;
	//back plane
	MatrixXf t3(3, 4);
	t3 << V1[1], V1[6], V1[7], A1;
	MatrixXf t4(3, 4);
	t4 << V1[4], V1[6], V1[7], A1;
	//top plane
	MatrixXf t5(3, 4);
	t5 << V1[0], V1[1], V1[2], A3;
	MatrixXf t6(3, 4);
	t6 << V1[7], V1[1], V1[2], A3;
	//under plane
	MatrixXf t7(3, 4);
	t7 << V1[3], V1[5], V1[6], A3;
	MatrixXf t8(3, 4);
	t8 << V1[4], V1[5], V1[6], A3;
	//left plane
	MatrixXf t9(3, 4);
	t9 << V1[2], V1[5], V1[7], A2;
	MatrixXf t10(3, 4);
	t10 << V1[4], V1[5], V1[7], A2;
	//right plane
	MatrixXf t11(3, 4);
	t11 << V1[0], V1[1], V1[3], A2;
	MatrixXf t12(3, 4);
	t12 << V1[6], V1[1], V1[3], A2;

	vector<MatrixXf> Triangle1{ t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12 };

	//front plane
	MatrixXf i1(3, 4);
	i1 << V2[0], V2[2], V2[3],	B1;
	MatrixXf i2(3, 4);
	i2 << V2[5], V2[2], V2[3], B1;
	//back plane
	MatrixXf i3(3, 4);
	i3 << V2[1], V2[6], V2[7], B1;
	MatrixXf i4(3, 4);
	i4 << V2[4], V2[6], V2[7], B1;
	//top plane
	MatrixXf i5(3, 4);
	i5 << V2[0], V2[1], V2[2], B3;
	MatrixXf i6(3, 4);
	i6 << V2[7], V2[1], V2[2], B3;
	//under plane
	MatrixXf i7(3, 4);
	i7 << V2[3], V2[5], V2[6], B3;
	MatrixXf i8(3, 4);
	i8 << V2[4], V2[5], V2[6], B3;
	//left plane
	MatrixXf i9(3, 4);
	i9 << V2[2], V2[5], V2[7], B2;
	MatrixXf i10(3, 4);
	i10 << V2[4], V2[5], V2[7], B2;
	//right plane
	MatrixXf i11(3, 4);
	i11 << V2[0], V2[1], V2[3], B2;
	MatrixXf i12(3, 4);
	i12 << V2[6], V2[1], V2[3], B2;

	vector<MatrixXf> Triangle2{ i1,i2,i3,i4,i5,i6,i7,i8,i9,i10,i11,i12 };

	//get the edges of objects
	vector<MatrixXf> E1 = GetEdges(position1, orientation1, Length, Width, Height);
	vector<MatrixXf> E2 = GetEdges(position2, orientation2, Length, Width, Height);

	//edges of object2 clip object1
	float distanceofP1 = FLT_MAX;
	Matrix3f result1;
	//Initialization Matrix
	result1
		<< FLT_MIN, 1, FLT_MAX,
		FLT_MIN, 1, FLT_MAX,
		FLT_MIN, 1, FLT_MAX;
	for (int i = 0; i < Triangle1.size(); ++i)
	{
		for (int j = 0; j < E2.size(); ++j)
		{
			Vector3f tri0(Triangle1[i](0, 0), Triangle1[i](1, 0), Triangle1[i](2, 0));
			Vector3f tri1(Triangle1[i](0, 1), Triangle1[i](1, 1), Triangle1[i](2, 1));
			Vector3f tri2(Triangle1[i](0, 2), Triangle1[i](1, 2), Triangle1[i](2, 2));
			Vector3f e0 = tri1 - tri0;
			Vector3f e1 = tri2 - tri0;

			Vector3f line0(E2[j](0, 0), E2[j](1, 0), E2[j](2, 0));
			Vector3f line1(E2[j](0, 1), E2[j](1, 1), E2[j](2, 1));
			Vector3f dir = line1 - line0;
			Vector3f dir_normal = dir / dir.norm();

			Vector3f h = dir_normal.cross(e1);
			const float a = e0.dot(h);

			if (a > -EPSILON && a < EPSILON)
				continue;

			Vector3f s = line0 - tri0;
			const float f = 1.0f / a;
			const float u = f * s.dot(h);

			if (u < 0.0f || u > 1.0f)
				continue;

			Vector3f q = s.cross(e0);
			const float v = f * dir_normal.dot(q);

			if (v < 0.0f || u + v > 1.0f)
				continue;

			const float t = f * e1.dot(q);
			Vector3f point;
			point = line0 + dir_normal * t;

			//Get the end point of edges, which is closer to intersect point
			Vector3f nearPoint;
			Vector3f Dif0;
			Vector3f Dif1;
			Dif0 = point - line0;
			Dif1 = point - line1;

			if (Dif0.norm() > Dif1.norm())
				nearPoint = line1;
			else
				nearPoint = line0;

			//Add normal of triangle, nearPoint of edges to point,  and point in one matrix(3,3)
			Vector3f normal(Triangle1[i](0, 3), Triangle1[i](1, 3), Triangle1[i](2, 3));
			Matrix3f pointNormal;
			pointNormal << point, normal, nearPoint;

			float dis;
			dis = fabs((point - nearPoint).dot(normal));

			if (dis < distanceofP1)
			{
				distanceofP1 = dis;
				result1 = pointNormal;
			}
		}
	}

	//edges of object1 clip object2
	float distanceofP2 = FLT_MAX;
	Matrix3f result2;
	//Initialization Matrix
	result2
		<< FLT_MIN, 1, FLT_MAX,
		FLT_MIN, 1, FLT_MAX,
		FLT_MIN, 1, FLT_MAX;
	for (int i = 0; i < Triangle2.size(); ++i)
	{
		for (int j = 0; j < E1.size(); ++j)
		{
			Vector3f tri0(Triangle2[i](0, 0), Triangle2[i](1, 0), Triangle2[i](2, 0));
			Vector3f tri1(Triangle2[i](0, 1), Triangle2[i](1, 1), Triangle2[i](2, 1));
			Vector3f tri2(Triangle2[i](0, 2), Triangle2[i](1, 2), Triangle2[i](2, 2));
			Vector3f e0 = tri1 - tri0;
			Vector3f e1 = tri2 - tri0;

			Vector3f line0(E1[j](0, 0), E1[j](1, 0), E1[j](2, 0));
			Vector3f line1(E1[j](0, 1), E1[j](1, 1), E1[j](2, 1));
			Vector3f dir = line1 - line0;
			Vector3f dir_normal = dir / dir.norm();

			Vector3f h = dir_normal.cross(e1);
			const float a = e0.dot(h);

			if (a > -EPSILON && a < EPSILON)
				continue;

			Vector3f s = line0 - tri0;
			const float f = 1.0f / a;
			const float u = f * s.dot(h);

			if (u < 0.0f || u > 1.0f)
				continue;

			Vector3f q = s.cross(e0);
			const float v = f * dir_normal.dot(q);

			if (v < 0.0f || u + v > 1.0f)
				continue;

			const float t = f * e1.dot(q);
			Vector3f point;
			point = line0 + dir_normal * t;

			//Get the end point of edges, which is closer to intersect point
			Vector3f nearPoint;
			Vector3f Dif0;
			Vector3f Dif1;
			Dif0 = point - line0;
			Dif1 = point - line1;

			if (Dif0.norm() > Dif1.norm())
				nearPoint = line1;
			else
				nearPoint = line0;

			//Add normal of triangle, nearPoint of edges to point,  and point in one matrix(3,3)
			Vector3f normal(Triangle2[i](0, 3), Triangle2[i](1, 3), Triangle2[i](2, 3));
			Matrix3f pointNormal;
			pointNormal << point, normal, nearPoint;

			float dis;
			dis = fabs((point - nearPoint).dot(normal));

			if (dis < distanceofP2)
			{
				distanceofP2 = dis;
				result2 = pointNormal;
			}
		}
	}

	float dis1;
	Vector3f normal1(result1(0, 1), result1(1, 1), result1(2, 1));
	Vector3f point1(result1(0, 0), result1(1, 0), result1(2, 0));
	Vector3f nearPoint1(result1(0, 2), result1(1, 2), result1(2, 2));
	dis1 = fabs((point1 - nearPoint1).dot(normal1));

	float dis2;
	Vector3f normal2(result2(0, 1), result2(1, 1), result2(2, 1));
	Vector3f point2(result2(0, 0), result2(1, 0), result2(2, 0));
	Vector3f nearPoint2(result2(0, 2), result2(1, 2), result2(2, 2));
	dis2 = fabs((point2 - nearPoint2).dot(normal2));

	cout << result1 << endl;
	cout << result2 << endl;

	//distance of edge to edge, here we use the algorithm to calculate Segment Segment Distance
	float disEE = FLT_MAX;
	for (int i = 0; i < E1.size(); ++i)
	{
		for (int j = 0; j < E2.size(); ++j)
		{
			//AB segment
			Vector3f A(E2[j](0, 0), E2[j](1, 0), E2[j](2, 0));
			Vector3f B(E2[j](0, 1), E2[j](1, 1), E2[j](2, 1));
			//CD segment
			Vector3f C(E1[i](0, 0), E1[i](1, 0), E1[i](2, 0));
			Vector3f D(E1[i](0, 1), E1[i](1, 1), E1[i](2, 1));

			float a, b, c, m, n, Q, t, s, distan;
			float X, Y, Z, U, V, W;

			a = (B(0) - A(0)) * (B(0) - A(0)) + (B(1) - A(1)) * (B(1) - A(1)) + (B(2) - A(2)) * (B(2) - A(2));
			b = (B(0) - A(0)) * (D(0) - C(0)) + (B(1) - A(1)) * (D(1) - C(1)) + (B(2) - A(2)) * (D(2) - C(2));
			c = (A(0) - B(0)) * (A(0) - C(0)) + (A(1) - B(1)) * (A(1) - C(1)) + (A(2) - B(2)) * (A(2) - C(2));
			m = (B(0) - A(0)) * (D(0) - C(0)) + (B(1) - A(1)) * (D(1) - C(1)) + (B(2) - A(2)) * (D(2) - C(2));
			n = (D(0) - C(0)) * (D(0) - C(0)) + (D(1) - C(1)) * (D(1) - C(1)) + (D(2) - C(2)) * (D(2) - C(2));
			Q = (A(0) - C(0)) * (D(0) - C(0)) + (A(1) - C(1)) * (D(1) - C(1)) + (A(2) - C(2)) * (D(2) - C(2));

			t = (c*m + a * Q) / (a*n - b * m);             //两条平行线时t,s会出错，但答案正确
			s = (c*n + b * Q) / (a*n - b * m);

			if ((t >= 0 && t <= 1) && (s >= 0 && s <= 1)) // 异面直线的公垂线都在线段上，(X,Y,Z)在线段AB上，(U,V,W)在线段CD上
			{
				X = A(0) + s * (B(0) - A(0));
				Y = A(1) + s * (B(1) - A(1));
				Z = A(2) + s * (B(2) - A(2));
				U = C(0) + t * (D(0) - C(0));
				V = C(1) + t * (D(1) - C(1));
				W = C(2) + t * (D(2) - C(2));

				distan = sqrt((X - U)*(X - U) + (Y - V)*(Y - V) + (Z - W)*(Z - W));
				if (distan < disEE)
				{
					disEE = distan;
					Point1 << X, Y, Z;
					Point2 << U, V, W;
				}

			}
			else
			{
				Vector3f Pp1;
				Vector3f Pp2;
				Matrix3f K = PointToSegment(A, C, D);
				distan = K(0, 0);
				Pp1 << K(0, 1), K(1, 1), K(2, 1);
				Pp2 << K(0, 2), K(1, 2), K(2, 2);

				Matrix3f L = PointToSegment(B, C, D);
				if (distan > L(0, 0))
				{
					distan = L(0, 0);
					Pp1 << L(0, 1), L(1, 1), L(2, 1);
					Pp2 << L(0, 2), L(1, 2), L(2, 2);
				}

				Matrix3f Z = PointToSegment(C, A, B);
				if (distan > Z(0, 0))
				{
					distan = Z(0, 0);
					Pp1 << Z(0, 1), Z(1, 1), Z(2, 1);
					Pp2 << Z(0, 2), Z(1, 2), Z(2, 2);
				}

				Matrix3f X = PointToSegment(D, A, B);
				if (distan > X(0, 0))
				{
					distan = X(0, 0);
					Pp1 << X(0, 1), X(1, 1), X(2, 1);
					Pp2 << X(0, 2), X(1, 2), X(2, 2);
				}
				if (distan < disEE)
				{
					disEE = distan;
					Point1 = Pp1;
					Point2 = Pp2;
				}
			}
		}
	}

	cout << dis1 << endl;
	cout << dis2 << endl;
	cout << disEE << endl;

	//(point2 - nearPoint2).norm < EPSILON
	if (dis1 > dis2 && disEE > dis2)
	{



		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
		//draw point on the plane
		glColor3f(0.5, 0.25, 0);
		glPointSize(5.0f);
		glBegin(GL_POINTS);
		glVertex3f(point2(0) / 20, point2(1) / 20, point2(2) / 20);
		glEnd();


		//connect point on plane to nearest end point of edge
		glColor3f(0.5, 0.25, 0);
		glLineWidth(1.0f);
		glBegin(GL_LINES);
		glVertex3f(point2(0) / 20, point2(1) / 20, point2(2) / 20);
		glVertex3f(nearPoint2(0) / 20, nearPoint2(1) / 20, nearPoint2(2) / 20);
		glEnd();


		//normal2
		glColor3f(1, 0, 0);
		glBegin(GL_LINES);
		glVertex3f((normal2(0) * 30 + point2(0)) / 20, (normal2(1) * 30 + point2(1)) / 20, (normal2(2) * 30 + point2(2)) / 20);
		glVertex3f(point2(0) / 20, point2(1) / 20, point2(2) / 20);
		glEnd();

		//project distance to normal
		Vector3f P((point2(0) + normal2(0) * dis2), (point2(1) + normal2(1) * dis2), (point2(2) + normal2(2) * dis2));
		glLineWidth(3.0f);
		glColor3f(1, 0, 0);
		glBegin(GL_LINES);
		glVertex3f((point2(0) + normal2(0) * dis2) / 20, (point2(1) + normal2(1) * dis2) / 20, (point2(2) + normal2(2) * dis2) / 20);
		glVertex3f(point2(0) / 20, point2(1) / 20, point2(2) / 20);

		glEnd();


		if (dis2 < EPSILON)
			cout << "real distance: " << 0 << endl;
		else
			cout << "real distance: " << dis2 << endl;
	}
	if (dis2 > dis1 && disEE > dis1)
	{
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
		//draw point on the plane
		glColor3f(0.5, 0.25, 0);
		glPointSize(5.0f);
		glBegin(GL_POINTS);
		glVertex3f(point1(0) / 20, point1(1) / 20, point1(2) / 20);
		glEnd();

		//connect point on plane to nearest end point of edge
		glColor3f(0.5, 0.25, 0);
		glLineWidth(1.0f);
		glBegin(GL_LINES);
		glVertex3f(point1(0) / 20, point1(1) / 20, point1(2) / 20);
		glVertex3f(nearPoint1(0) / 20, nearPoint1(1) / 20, nearPoint1(2) / 20);
		glEnd();

		//normal1
		glColor3f(1, 0, 0);
		glBegin(GL_LINES);
		glVertex3f((normal1(0) * 30 + point1(0)) / 20, (normal1(1) * 30 + point1(1)) / 20, (normal1(2) * 30 + point1(2)) / 20);
		glVertex3f(point1(0) / 20, point1(1) / 20, point1(2) / 20);
		glEnd();

		//project distance to normal
		Vector3f P((point1(0) + normal1(0) * dis1), (point1(1) + normal1(1) * dis1), (point1(2) + normal1(2) * dis1));
		glLineWidth(3.0f);
		glColor3f(1, 0, 0);
		glBegin(GL_LINES);
		glVertex3f((point1(0) + normal1(0) * dis1) / 20, (point1(1) + normal1(1) * dis1) / 20, (point1(2) + normal1(2) * dis1) / 20);
		glVertex3f(point1(0) / 20, point1(1) / 20, point1(2) / 20);

		glEnd();


		if (dis1 < EPSILON)
			cout << "real distance: " << 0 << endl;
		else
			cout << "real distance: " << dis1 << endl;
	}


	/*
	The following conditional judgment means,
	1.there are no edges or extend edges intersect any plane.
	2.distance of edge to edge is smaller than edge to plane.
	*/
	if (dis1 > disEE && dis2 > disEE)
	{
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//background color
		glColor3f(1, 0, 0);
		glLineWidth(3.0f);
		glBegin(GL_LINES);
		glVertex3f(Point1(0) / 20, Point1(1) / 20, Point1(2) / 20);
		glVertex3f(Point2(0) / 20, Point2(1) / 20, Point2(2) / 20);
		glEnd();

		glColor3f(0.5, 0.25, 0);
		glPointSize(5.0f);
		glBegin(GL_POINTS);
		glVertex3f(Point1(0) / 20, Point1(1) / 20, Point1(2) / 20);
		glVertex3f(Point2(0) / 20, Point2(1) / 20, Point2(2) / 20);
		glEnd();

		if (disEE < EPSILON)
			cout << "real distance: " << 0 << endl;
		else
			cout << "real distance: " << disEE << endl;
	}



}





