#ifndef SCMPATHOBJ_H
#define SCMPATHOBJ_H

#define kSCMX_MAX_FILE_EXTENSION 8
#define kSCMX_MAX_PATH 512

namespace scm
{
	class PathObj
	{
	private:
		char mExtension[kSCMX_MAX_FILE_EXTENSION];
		char mFile[kSCMX_MAX_PATH];
		char mDirectory[kSCMX_MAX_PATH];
		char mPath[kSCMX_MAX_PATH];

	public:
		PathObj(const char * path);
		~PathObj(void);

		const char * GetExtension(void);
		const char * GetFileName(void);
		const char * GetDirectory(void);
		const char * GetFilePath(void);
	};
}

#endif