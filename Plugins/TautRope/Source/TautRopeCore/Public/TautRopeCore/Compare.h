#pragma once

#include "TautRopeCore/Core.h"
#include "TautRopeCore/Recording.h"

#include <string>

namespace TautRope
{
	// Where two recordings first stop agreeing. Doubles are compared bitwise, not
	// against an epsilon: an epsilon hides exactly the drift this exists to catch,
	// and a simulation that is only nearly reproducible is not reproducible.
	struct RecordingDivergence
	{
		bool bDiverged = false;

		// "structure" when the recordings do not even have the same shape, so the
		// remaining fields may not apply.
		const char* Phase = "";
		const char* Field = "";
		int32 FrameIndex = IndexNone;
		int32 PointIndex = IndexNone;

		std::string RecordedValue;
		std::string ReplayedValue;

		// How widespread the disagreement is, not just where it starts.
		int32 DivergentFrameCount = 0;
		int32 ComparedFrameCount = 0;
	};

	// Returns true when the two recordings agree exactly. Comparison covers the
	// per-frame inputs as well as the captured output, so a format-level fault
	// cannot masquerade as a simulation difference.
	TAUTROPE_CORE_API bool CompareRecordings(
		const Recording& Recorded
		, const Recording& Replayed
		, RecordingDivergence& OutDivergence
	);
}
