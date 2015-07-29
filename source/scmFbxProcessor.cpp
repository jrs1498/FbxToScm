#include <string>
#include <deque>
#include <fbxsdk.h>

#include "../include/scmLog.h"
#include "../include/scmScmSerializer.h"
#include "../include/scmFbxProcessMesh.h"
#include "../include/scmFbxProcessSequence.h"
#include "../include/scmFbxProcessor.h"

namespace scm
{
	static FbxManager * fbxManager;
	static FbxIOSettings * fbxIOSettings;
	static FbxImporter * fbxImporter;
	static FbxScene * fbxScene;
	static std::deque<PathObj> fbxFiles;

	//=================================================================

	enum FbxFileType
	{
		invalid = 0x0,
		mesh,
		sequence
	};

	//=================================================================

	static void LoadFbxFile(PathObj * pathObj)
	{
		scmprint("Loading Fbx file:\r\n\t%s\r\n", pathObj->GetFileName());

		fbxManager = FbxManager::Create();
		fbxIOSettings = FbxIOSettings::Create(fbxManager, IOSROOT);
		fbxManager->SetIOSettings(fbxIOSettings);
		fbxImporter = FbxImporter::Create(fbxManager, "");

		if (!fbxImporter->Initialize(pathObj->GetFilePath(), -1, fbxManager->GetIOSettings()))
		{
			scmprint("Load file failed\r\n");
			return;
		}

		fbxScene = FbxScene::Create(fbxManager, "myScene");

		fbxImporter->Import(fbxScene);

		fbxImporter->Destroy();
	}

	//=================================================================

	static void CleanUpFbxFile(void)
	{
		scmprint("Cleaning up loaded fbx file\r\n");

		if (fbxManager)
			fbxManager->Destroy();

		fbxManager = 0;
		fbxIOSettings = 0;
		fbxImporter = 0;
		fbxScene = 0;
	}

	//=================================================================

	static void ExtractFileNameLessPrefix(PathObj * pathObj, char outName[kSCM_LongNameLen])
	{
		const char * fileName = pathObj->GetFileName();

		int32_t fptr1, fptr2;
		fptr1 = 0;
		fptr2 = 0;

		while (fileName[fptr1] != '_')
		{
			fptr1++;
		}
		fptr1++;
		while (fileName[fptr1] != '\0')
		{
			outName[fptr2] = fileName[fptr1];
			fptr1++;
			fptr2++;
		}
		outName[fptr2] = '\0';
	}

	static void AppendScmExtension(const char * fileName, char outName[kSCM_LongNameLen])
	{
		int32_t fptr = 0;
		while (fileName[fptr] != '\0')
		{
			outName[fptr] = fileName[fptr];
			fptr++;
		}
		outName[fptr++] = '.';
		outName[fptr++] = 's';
		outName[fptr++] = 'c';
		outName[fptr++] = 'm';
		outName[fptr] = '\0';
	}

	//=================================================================

	FbxFileType GetFbxFileType(PathObj * pathObj)
	{
		std::string fileName(pathObj->GetFileName());
		std::string substrMesh("mesh_");
		std::string substrSequence("sequence_");

		if (fileName.find(substrMesh) != std::string::npos)
			return FbxFileType::mesh;
		if (fileName.find(substrSequence) != std::string::npos)
			return FbxFileType::sequence;

		return FbxFileType::invalid;
	}

	//=================================================================

	void FbxProcessor::EnqueueFbxFile(PathObj * pathObj)
	{
		scmprint("Enqueue Fbx file\r\n");

		FbxFileType fileType = GetFbxFileType(pathObj);

		switch (fileType)
		{
		case FbxFileType::invalid:
			scmprint("Fbx files must preceed with mesh_ or sequence_\r\n");
			break;

		case FbxFileType::mesh:
			fbxFiles.push_front(*pathObj);
			break;

		case FbxFileType::sequence:
			fbxFiles.push_back(*pathObj);
			break;
		}
	}

	//=================================================================

	void FbxProcessor::ProcessFbxFiles(void)
	{
		scmprint("Processing all fbx files\r\n");

		for (int32_t i = 0; i < fbxFiles.size(); i++)
		{
			PathObj & fbxFile = fbxFiles[i];

			// Get the file name without its mesh_ or sequence_ prefix
			char fbxFilenameLessPrefix[kSCM_LongNameLen];
			ExtractFileNameLessPrefix(&fbxFile, fbxFilenameLessPrefix);

			LoadFbxFile(&fbxFile);

			FbxFileType fileType = GetFbxFileType(&fbxFiles[i]);

			// Convert scene from right to left hand coordinate system
			//FbxAxisSystem fbxAxisSystemLeftHand(
			//	FbxAxisSystem::EUpVector::eYAxis,
			//	FbxAxisSystem::EFrontVector::eParityOdd,
			//	FbxAxisSystem::ECoordSystem::eRightHanded);
			//fbxAxisSystemLeftHand.ConvertScene(fbxScene);

			// Process the file
			switch (fileType)
			{
			case FbxFileType::mesh:
				char fbxFileExportPath[kSCM_LongNameLen];
				AppendScmExtension(fbxFilenameLessPrefix, fbxFileExportPath);
				FbxProcessMesh::ProcessFbxMesh(fbxScene, fbxFileExportPath);
				break;

			case FbxFileType::sequence:
				FbxProcessSequence::ProcessFbxSequence(fbxScene, fbxFilenameLessPrefix);
				break;
			}

			CleanUpFbxFile();
		}
	}
}