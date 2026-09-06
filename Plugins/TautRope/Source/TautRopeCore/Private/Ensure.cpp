#include "TautRopeCore/Core.h"

#include <cstdio>

namespace TautRope
{
	namespace
	{
		EnsureHandler GEnsureHandler = nullptr;
	}

	void SetEnsureHandler(EnsureHandler Handler)
	{
		GEnsureHandler = Handler;
	}

	bool HandleEnsureFailure(const char* Expression, const char* File, int Line)
	{
		if (GEnsureHandler != nullptr)
		{
			GEnsureHandler(Expression, File, Line);
		}
		else
		{
			std::fprintf(stderr, "TautRope ensure failed: %s at %s:%d\n", Expression, File, Line);
		}
		return false;
	}
}
