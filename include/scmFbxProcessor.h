#ifndef SCMFBXPROCESSOR_H
#define SCMFBXPROCESSOR_H

#include "scmPathObj.h"

namespace scm
{
	class FbxProcessor
	{
	public:
		static void EnqueueFbxFile(PathObj * pathObj);
		static void ProcessFbxFiles(void);
	};
};

#endif