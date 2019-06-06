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

#pragma comment(lib, "assimp-vc140-mt.lib")

using namespace std;

/*
	aiScene = Mesh, Rootnode, Animation
		Mesh->aiMesh = Name, [normal], face, [bone], [vertice]
			[[bone->aiBone = Offsetmatrix, weight, Name]]
		Rootnode->aiNode = Child(Child->aiNode), [[Meshindex(->aiMesh)]], Name, Matrix
		
		Animation->aiAnimation = Name, channel->aiNodeAnim, Duration, Ticks per second
			[[aiNodeAnim = Name, RotKey(ȸ��), ScaleKey, PosKey]] / ���� Time, value���� ����
*/

struct mesh
{
	vector<Vertex> m_vertices;
	vector<int>	m_indices;
};

struct Bone
{
	XMMATRIX BoneOffset = XMMatrixIdentity();
	XMMATRIX TransFormation = XMMatrixIdentity();
};

class ModelLoader
{
private:
	const aiScene*				m_pScene;
	vector<mesh>				m_Meshes;
	vector<pair<string, Bone>>	m_Bones;

	unsigned int m_NumVertices = -1;
	unsigned int m_NumBones = -1;

	XMMATRIX m_GlobalInverseTransform;

public:
	//�Ž�����, �� 
	ModelLoader();
	~ModelLoader();
	void InitScene();
	void InitMesh(unsigned int index, const aiMesh* pMesh);
	void InitBone(unsigned int index, const aiMesh* pMesh);
	void ModelLoad(const string& file, bool isStatic);

	vector<mesh> GetMesh();

	//�ִϸ��̼�

	void ReadNodeHeirarchy(float AnimationTime, const aiNode* pNose, const XMMATRIX& ParentTransform);
	const aiNodeAnim* FindNodeAnim(const aiAnimation* pAnimaition, const string& NodeName);

	void CalcInterpolatedScaling(aiVector3D& Scaling, float AnimationTime, const aiNodeAnim* pNodeAnim);
	void CalcInterpolatedRotation(aiQuaternion& RotationQ, float AnimationTime, const aiNodeAnim* pNodeAnim);
	void CalcInterpolatedPosition(aiVector3D& Translation, float AnimationTime, const aiNodeAnim* pNodeAnim);

};

