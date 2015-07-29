#include <string>
#include <iostream>

#include "../include/scmPreReq.h"
#include "../include/scmLog.h"
#include "../include/scmPathObj.h"
#include "../include/scmScmSerializer.h"
#include "../include/scmFbxProcessor.h"
#include "../include/scmUtility.h"

using namespace scm;

//=================================================================

enum InputFileType
{
	invalid = 0x0,
	fbx
};

//=================================================================

InputFileType GetInputFileType(PathObj * pathObj)
{
	const char * pathExt = pathObj->GetExtension();

	if (!strcmp(pathExt, "fbx"))
		return InputFileType::fbx;

	return InputFileType::invalid;
}

//=================================================================

void ProcessCmdArg(const char * arg)
{
	scmprint("Processing command line arg:\r\n\t%s\r\n", arg);

	PathObj pathObj(arg);
	scmprint("\tFile: %s\r\n", pathObj.GetFileName());
	scmprint("\tExtension: %s\r\n", pathObj.GetExtension());
	scmprint("\tDirectory: %s\r\n", pathObj.GetDirectory());

	InputFileType ift = GetInputFileType(&pathObj);

	switch (ift)
	{
	case InputFileType::fbx:
		FbxProcessor::EnqueueFbxFile(&pathObj);
		break;

	case InputFileType::invalid:
		scmprint("Unrecognized file type\r\n");
		break;
	}
}

//=================================================================

int main(int argc, char ** argv)
{
	if (argc < 2)
	return -1;

	for (int i = 1; i < argc; i++)
	{
		ProcessCmdArg(argv[i]);
	}
	
	ScmSerializer::Initialize();
	FbxProcessor::ProcessFbxFiles();
	ScmSerializer::SerializeAndExport();

	system("PAUSE");
	return 0;
}