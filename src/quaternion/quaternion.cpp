#include "quaternion.hpp"
#include "linalg/linalg.hpp"
#include <vector>

using namespace std;
using namespace linalg;

void quaternion::q_to_euler(Vector& q, Vector& euler)
{
	double qw, qx, qy, qz;
	qw = q[0];
	qx = q[1];
	qy = q[2];
	qz = q[3];

	euler[0] = atan2(2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);
	euler[1] = asin(-2.0 * (qx * qz - qw * qy));
	euler[2] = atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
}

Vector quaternion::q_to_euler(Vector& q)
{
	Vector euler(3, 0.0);
	q_to_euler(q, euler);
	return euler;
}

void quaternion::q_to_mat4(Vector& q, Matrix& mat)
{
	double qw, qx, qy, qz;
	qw = q[0];
	qx = q[1];
	qy = q[2];
	qz = q[3];

	mat[0][0] = 2 * (qw * qw + qx * qx) - 1;
	mat[0][1] = 2 * (qx * qy - qw * qz);
	mat[0][2] = 2 * (qx * qz + qw * qy);
	mat[0][3] = 0.0;

	mat[1][0] = 2 * (qx * qy + qw * qz);
	mat[1][1] = 2 * (qw * qw + qy * qy) - 1;
	mat[1][2] = 2 * (qy * qz - qw * qx);
	mat[1][3] = 0.0;

	mat[2][0] = 2 * (qx * qz - qw * qy);
	mat[2][1] = 2 * (qy * qz + qw * qx);
	mat[2][2] = 2 * (qw * qw + qz * qz) - 1;
	mat[2][3] = 0.0;

	mat[3][0] = 0.0;
	mat[3][1] = 0.0;
	mat[3][2] = 0.0;
	mat[3][3] = 1.0;
}

Matrix quaternion::q_to_mat4(Vector& q)
{
	Matrix rot(4, 4, 0.0);
	q_to_mat4(q, rot);
	return rot;
}
