#include <string>
#include "../include/scmPathObj.h"

//=================================================================

namespace scm
{
	PathObj::PathObj(const char * path)
	{
		// Copy full file path
		memcpy(&mPath[0], &path[0], sizeof(char)* kSCMX_MAX_PATH);

		// Get file extension
		const char * ext = strpbrk(path, ".");
		if (!ext)
			mExtension[0] = '\0';
		else
		{
			memcpy(&mExtension[0], ext + 1, sizeof(char)* kSCMX_MAX_FILE_EXTENSION);
		}

		// Get file name
		int i = 0;
		while (path[i] != '\0')
			i++;
		while (i >= 0 && path[i] != '\\')
			i--;
		i++;

		int j = 0;
		while (j < kSCMX_MAX_PATH && path[i] != '.')
		{
			mFile[j] = path[i];
			j++;
			i++;
		}
		mFile[j] = '\0';

		// Get the directory
		i = j = 0;
		while (path[i] != '\0')
			i++;
		while (i >= 0 && path[i] != '\\')
			i--;
		while (j < i)
		{
			mDirectory[j] = path[j];
			j++;
		}
		mDirectory[j] = '\0';
	}

	//=================================================================

	PathObj::~PathObj(void)
	{

	}

	//=================================================================

	const char * PathObj::GetExtension(void)
	{
		return mExtension;
	}

	//=================================================================

	const char * PathObj::GetFileName(void)
	{
		return mFile;
	}

	//=================================================================

	const char * PathObj::GetDirectory(void)
	{
		return mDirectory;
	}

	//=================================================================

	const char * PathObj::GetFilePath(void)
	{
		return mPath;
	}
}