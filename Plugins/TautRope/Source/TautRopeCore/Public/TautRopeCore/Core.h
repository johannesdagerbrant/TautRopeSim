#pragma once

#include <cstdint>

// Set from TautRopeCore.Build.cs when built inside Unreal, where every module is
// its own DLL. The standalone replay program defines neither and gets plain
// symbols. UE's own TAUTROPECORE_API is unusable here: it expands to DLLEXPORT,
// which only exists in engine platform headers this module must not include.
#if defined(TAUTROPE_CORE_SHARED) && defined(_MSC_VER)
	#if defined(TAUTROPE_CORE_EXPORTS)
		#define TAUTROPE_CORE_API __declspec(dllexport)
	#else
		#define TAUTROPE_CORE_API __declspec(dllimport)
	#endif
#else
	#define TAUTROPE_CORE_API
#endif

namespace TautRope
{
	using int32 = std::int32_t;

	inline constexpr int32 IndexNone = -1;

	using EnsureHandler = void (*)(const char* Expression, const char* File, int Line);

	// Replaces UE's ensure(). Reports and keeps going rather than aborting, so a
	// violated invariant produces the same control flow in the editor and in the
	// replay program.
	TAUTROPE_CORE_API void SetEnsureHandler(EnsureHandler Handler);
	TAUTROPE_CORE_API bool HandleEnsureFailure(const char* Expression, const char* File, int Line);
}

#define TAUTROPE_ENSURE(Expression) \
	((Expression) ? true : TautRope::HandleEnsureFailure(#Expression, __FILE__, __LINE__))
