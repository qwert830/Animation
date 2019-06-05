#pragma once

#ifdef max

#undef max
#undef min
#endif

#include "assimp\Importer.hpp"
#include "assimp\cimport.h"
#include "assimp\postprocess.h"
#include "assimp\scene.h"

#include "FrameResource.h"
#include "d3dUtil.h"
#include <fstream>
#include <iostream>

#pragma comment(lib, "assimp-vc140-mt.lib")

using namespace std;

/*
	aiScene = Mesh, Rootnode, Animation
		Mesh->aiMesh = Name, [normal], face, [bone], [vertice]
			[[bone->aiBone = Offsetmatrix, weight, Name]]
		Rootnode->aiNode = Child(Child->aiNode), [[Meshindex(->aiMesh)]], Name, Matrix
		
		Animation->aiAnimation = Name, channel->aiNodeAnim, Duration, Ticks per second
			[[aiNodeAnim = Name, RotKey(회전), ScaleKey, PosKey]] / 전부 Time, value값을 가짐

*/
struct mesh
{
	vector<Vertex> m_vertices;
	vector<int>	m_indices;
};

struct Bone
{
	XMFLOAT4X4 BoneOffset = MathHelper::Identity4x4();
	XMFLOAT4X4 TransFormation = MathHelper::Identity4x4();
};

class ModelLoader
{
private:
	const aiScene*							m_pScene;
	vector<mesh>							m_Meshes;
	vector<pair<string, Bone>>	m_Bones;

	unsigned int m_NumVertices = -1;
	unsigned int m_NumBones = -1;

public:
	ModelLoader();
	~ModelLoader();
	void InitScene();
	void InitMesh(unsigned int index, const aiMesh* pMesh);
	void InitBone(unsigned int index, const aiMesh* pMesh);
	void ModelLoad(const string& file, bool isStatic);
	void TestPrint();
};

