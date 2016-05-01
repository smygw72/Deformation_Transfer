#include "mesh.h"

using namespace Eigen;

#pragma region Mesh
void Mesh::Reset() {
	Cleanup();
	Init();
}

void Mesh::Cleanup() {
}


void Mesh::computeNormal() {
	std::cout << "computing normals..." << std::endl;

	// reset all the normal
	m_N.setZero();
	
	// calculate normal for each individual triangle
	unsigned int triangle_num = getTriNum();
	unsigned int id0, id1, id2;
	EigenVector3 p0, p1, p2;
	EigenVector3 normal;
	for (unsigned int i = 0; i < triangle_num; ++i)
	{
		id0 = m_F(i, 0); 
		id1 = m_F(i, 1); 
		id2 = m_F(i, 2);

		p0 = m_V.row(id0).transpose(); 
		p1 = m_V.row(id1).transpose();
		p2 = m_V.row(id2).transpose();

		normal = (p1 - p0).cross(p2 - p1);
		normal.normalize();

		m_N.row(id0) += normal.transpose();
		m_N.row(id1) += normal.transpose();
		m_N.row(id2) += normal.transpose();
		
	}
	// re-normalize all the normals.
	for (int i = 0; i < m_N.rows(); i++)
	{
		if (m_N.row(i).norm() > EPSILON)
			m_N.row(i).normalize();

		//std::cout << m_N.row(i) << std::endl;
	}
	
}

int Mesh::getm_F(int i,int j) {
	return m_F(i, j);
}

double Mesh::getm_V(int i, int j) {
	return m_V(i, j);
}

//void Mesh::setm_F(double F,int i,int j) {
//	m_F(i, j) = F;
//}

void Mesh::setm_V(EigenMatrixXs V) {
	m_V = V;
}

#pragma endregion


#pragma region ObjMesh
bool ObjMesh::Init() {
	std::cout << "Init ObjMesh..." << std::endl;

	computeNormal();

	return true;
}

#include "GL/glut.h"
void ObjMesh::renderMesh() {
	glLineWidth(0.3);
	for (int i = 0; i < getTriNum(); ++i)
	{
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < 3; ++j)
		{
			int VertIndex = m_F(i, j);

			GLdouble normal[3] = { m_N(VertIndex, 0), m_N(VertIndex, 1), m_N(VertIndex, 2) };
			glNormal3dv(normal);

			glVertex3f(m_V(VertIndex, 0), m_V(VertIndex, 1), m_V(VertIndex, 2));
		}
		glEnd();
	}
}

void ObjMesh::renderPoly() {
	glLineWidth(0.3);
	for (int i = 0; i < getTriNum(); ++i){
		glBegin(GL_TRIANGLES);
		for (int j = 0; j < 3; ++j)
		{
			int VertIndex = m_F(i, j);

			GLdouble normal[3] = { m_N(VertIndex, 0), m_N(VertIndex, 1), m_N(VertIndex, 2) };
			glNormal3dv(normal);

			glVertex3f(m_V(VertIndex, 0), m_V(VertIndex, 1), m_V(VertIndex, 2));
		}
		glEnd();
	}
}

#include <igl/read_triangle_mesh.h>
void ObjMesh::read_from_file(std::string filename) {
	
	std::cout << "reading Obj file through libigl..." << std::endl;

	igl::readOBJ(filename, m_V, m_F);

	// adjust scaling
	m_scaling = 180;
	m_V *= m_scaling;

	// resize normal Mat
	m_N.resize(getVertNum(), 3);

}
#pragma endregion