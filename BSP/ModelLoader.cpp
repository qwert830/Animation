#include "ModelLoader.h"



ModelLoader::ModelLoader()
{
}

ModelLoader::~ModelLoader()
{
}

void ModelLoader::InitScene()
{
	for (unsigned int i = 0; i < m_Meshes.size(); ++i)
	{
		const aiMesh* pMesh = m_pScene->mMeshes[i];
		InitMesh(i, pMesh);
		m_NumVertices += (unsigned int)m_Meshes[i].m_vertices.size();
	}

}

void ModelLoader::InitMesh(unsigned int index, const aiMesh * pMesh)
{
	m_Meshes[index].m_vertices.reserve(pMesh->mNumVertices);
	m_Meshes[index].m_indices.reserve(pMesh->mNumFaces*3);

	for (UINT i = 0; i < pMesh->mNumVertices; ++i) {

		XMFLOAT3 pos(&pMesh->mVertices[i].x);

		XMFLOAT3 normal(&pMesh->mNormals[i].x);

		XMFLOAT2 tex;

		if (pMesh->HasTextureCoords(0))

			tex = XMFLOAT2(&pMesh->mTextureCoords[0][i].x);

		else

			tex = XMFLOAT2(0.0f, 0.0f);


		const Vertex data(XMFLOAT4(pos.x, pos.y, pos.z, 1.0f), normal, tex);

		m_Meshes[index].m_vertices.push_back(data);
	}
	for (UINT i = 0; i < pMesh->mNumFaces; ++i) {

		const aiFace& face = pMesh->mFaces[i];

		m_Meshes[index].m_indices.push_back(face.mIndices[0]);

		m_Meshes[index].m_indices.push_back(face.mIndices[1]);

		m_Meshes[index].m_indices.push_back(face.mIndices[2]);
	}
}

void ModelLoader::InitBone(unsigned int index, const aiMesh* pMesh)
{
	for (int i = 0; i < pMesh->mNumBones; ++i)
	{
		aiBone* bone = pMesh->mBones[i];
		auto numOfVertex = bone->mNumWeights;
		for (unsigned int b = 0; b < numOfVertex; ++b)
		{
			unsigned int id = bone->mWeights[b].mVertexId;
			if (id == 0)
			{
				cout << "?" << endl;
			}
			if (m_Meshes[index].m_vertices[id].BoneWeights.x == 0.0f)
			{
				m_Meshes[index].m_vertices[id].BoneWeights.x = bone->mWeights[b].mWeight;
				m_Meshes[index].m_vertices[id].BoneIndices[0] = i;
			}
			else if (m_Meshes[index].m_vertices[id].BoneWeights.y == 0.0f)
			{
				m_Meshes[index].m_vertices[id].BoneWeights.y = bone->mWeights[b].mWeight;
				m_Meshes[index].m_vertices[id].BoneIndices[1] = i;
			}
			else if (m_Meshes[index].m_vertices[id].BoneWeights.z == 0.0f)
			{
				m_Meshes[index].m_vertices[id].BoneWeights.z = bone->mWeights[b].mWeight;
				m_Meshes[index].m_vertices[id].BoneIndices[2] = i;
			}
			else
			{
				m_Meshes[index].m_vertices[id].BoneIndices[3] = i;
			}
		}
		m_NumBones++;
	}
}

void ModelLoader::ModelLoad(const std::string & file, bool isStatic)
{
	UINT flag = 
		aiProcess_JoinIdenticalVertices |			// join identical vertices/ optimize indexing
		aiProcess_ValidateDataStructure |			// perform a full validation of the loader's output
		aiProcess_ImproveCacheLocality |			// improve the cache locality of the output vertices
		aiProcess_RemoveRedundantMaterials |		// remove redundant materials
		aiProcess_GenUVCoords |						// convert spherical, cylindrical, box and planar mapping to proper UVs
		aiProcess_TransformUVCoords |				// pre-process UV transformations (scaling, translation ...)
		aiProcess_FindInstances |					// search for instanced meshes and remove them by references to one master
		aiProcess_LimitBoneWeights |				// limit bone weights to 4 per vertex
		aiProcess_OptimizeMeshes |					// join small meshes, if possible;
		aiProcess_GenSmoothNormals |				// generate smooth normal vectors if not existing
		aiProcess_SplitLargeMeshes |				// split large, unrenderable meshes into sub-meshes
		aiProcess_Triangulate |						// triangulate polygons with more than 3 edges
		aiProcess_ConvertToLeftHanded |				// convert everything to D3D left handed space
		aiProcess_SortByPType;						// make 'clean' meshes which consist of a single type of primitives

	if (isStatic)
		flag |= aiProcess_PreTransformVertices;			// preTransform Vertices (no bone & animation flag)

	m_pScene = aiImportFile(file.c_str(), flag);

	if (m_pScene) {
		m_Meshes.resize(m_pScene->mNumMeshes);
		m_NumBones = 0;
		InitScene();

		for (int i = 0; i < m_Meshes.size(); ++i)
		{
			const aiMesh* pMesh = m_pScene->mMeshes[i];
			if (pMesh->HasBones())
			{
				InitBone(i, pMesh);
			}
		}

		TestPrint();
	}
}

void ModelLoader::TestPrint()
{
	ofstream out;
	out.open("test.txt");
	auto data = m_Meshes[0].m_vertices;
	for (int i = 0; i < m_Meshes[0].m_vertices.size(); ++i)
	{
		out.setf(ios::fixed);
		out.precision(3);
		out << "i:" << i 
			<< "\t\tbi1:" << (unsigned int)data[i].BoneIndices[0] << "\t\tbi2:" << (unsigned int)data[i].BoneIndices[1] << "\t\tbi3:" << (unsigned int)data[i].BoneIndices[2]
			<< "\t\tbw1:" << data[i].BoneWeights.x << "\t\tbw2:" << data[i].BoneWeights.y << "\t\tbw3:" << data[i].BoneWeights.z << endl;
	}
	out.close();
}
