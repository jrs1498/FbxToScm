#ifndef SCMFBXPROCESSSEQUENCE_H
#define SCMFBXPROCESSSEQUENCE_H

namespace scm
{
	class FbxProcessSequence
	{
	public:
		static void ProcessFbxSequence(void * scene, const char * sequenceFileName);
	};
}

#endif