#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Determinism guards
//
// This module is compiled twice -- once by UBT into the editor, once by CMake
// into the replay program -- and a recording only replays bit-identically if
// both agree on floating-point codegen. Identical source is not sufficient.
//
// These checks live here rather than in either build file precisely because
// they then apply to both, and fail loudly at compile time instead of showing
// up later as a divergence that looks like a simulation bug.
//
// Verified against what UBT passes the editor build (see the response files
// under Intermediate/Build/.../TautRopeCore): /fp:precise, no /arch: flag.
// ---------------------------------------------------------------------------
#if defined(_MSC_VER)
	#if !defined(_M_FP_PRECISE)
		#error "TautRopeCore requires /fp:precise. The editor build uses it, and anything else diverges from recordings captured there."
	#endif
	#if defined(_M_FP_FAST)
		#error "TautRopeCore cannot be built with /fp:fast: it lets the compiler reassociate floating-point operations, so results stop being reproducible."
	#endif
	#if defined(_M_FP_STRICT)
		#error "TautRopeCore cannot be built with /fp:strict: it is deterministic but not identical to the /fp:precise results the editor produces."
	#endif
	// No /arch: flag in either build, so both keep the x64 SSE2 baseline. A wider
	// baseline changes auto-vectorisation and with it the results.
	#if defined(__AVX__) || defined(__AVX2__) || defined(__AVX512F__)
		#error "TautRopeCore must be built at the default x64 baseline. The editor build passes no /arch: flag, so raising it here would diverge."
	#endif
#endif

#if defined(__FAST_MATH__)
	#error "TautRopeCore cannot be built with fast-math: reproducibility is the point."
#endif

// Tracks the /std:c++20 UBT gives the editor build. Compiling these sources
// under a different standard than the editor does is a correctness hazard well
// beyond floating point. MSVC needs /Zc:__cplusplus for this to read correctly.
#if defined(__cplusplus) && __cplusplus < 202002L
	#error "TautRopeCore requires C++20, to match the standard the editor build compiles it with."
#endif

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
