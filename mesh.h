#pragma once

#include "iostream"
#include "math_headers.h"

typedef enum{
	MESH_TYPE_OBJ,
	MESH_TYPE_TOTAL_NUM
} MeshType;

class Mesh{
	public:
		Mesh() : m_mesh_type() {}
		Mesh(MeshType mesh_type) : m_mesh_type(mesh_type) {}
		virtual ~Mesh() { Cleanup(); }

		void Reset();
		virtual bool Init() { std::cout << "Warning: reach base class virtual init function." << std::endl; return false; }
		virtual void Cleanup();

	protected:
		MeshType m_mesh_type;

		EigenMatrixXs m_V0;	// rest pose
		EigenMatrixXs m_V;	// current pose
		EigenMatrixXs m_N;	// current normal
		EigenMatrixXi m_F;  // tri index
		EigenMatrixXi m_T;  // tet index

	public:	// get method
		int getVertNum() const { return (int)m_V.rows(); }	
		int getTriNum()  const { return (int)m_F.rows(); }	
		int getTetNum()  const { return (int)m_T.rows(); }	
		void computeNormal();
		int getm_F(int i,int j);
		double getm_V(int i,int j);
//		void setm_F(double F,int i,int j);
		void setm_V(EigenMatrixXs V);
		virtual void renderMesh() { std::cout << "Warning: reach base class virtual draw function." << std::endl; };
		virtual void renderPoly() { std::cout << "Warning: reach base class virtual draw function." << std::endl; };

		virtual void read_from_file(std::string filename) { std::cout << "Warning: reach base class virtual init function." << std::endl; };

};

class ObjMesh : public Mesh {
	public:
		ObjMesh() : Mesh(MESH_TYPE_OBJ) {}
		virtual ~ObjMesh() {}

		virtual bool Init();
	
	protected:
		float m_scaling;

	public:
		virtual void renderMesh();
		virtual void renderPoly();
		virtual void read_from_file(std::string filename);
};