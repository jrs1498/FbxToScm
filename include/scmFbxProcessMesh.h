#ifndef SCMFBXPROCESSMESH_H
#define SCMFBXPROCESSMESH_H

namespace scm
{
	class FbxProcessMesh
	{
	public:
		static void ProcessFbxMesh(void * scene, const char * meshFileName);
	};
}

#endif