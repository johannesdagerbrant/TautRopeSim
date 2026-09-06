#pragma once

#include "CoreMinimal.h"
#include "TautRopeCore/Recording.h"

namespace TautRope
{
	class Rope;
}

// Drives recording from the tautrope.record CVar. Owns the CVar, the file path
// and the lifetime; the format itself lives in TautRopeCore so the replay
// program can read and write the same thing without Unreal.
class TAUTROPE_API FTautRopeRecorder
{
public:
	// Names the recording file. Set once from the owning actor.
	void SetLabel(const FString& InLabel) { Label = InLabel; }

	// Handles the CVar transitions and opens the frame. Returns the buffer
	// UpdateRope should fill this frame, or nullptr when not recording. Call
	// immediately before UpdateRope.
	TautRope::FrameCapture* BeginFrame(
		const TautRope::Rope& Rope
		, const FVector& StartLocation
		, const FVector& EndLocation
		, float MaxLength
		, float DeltaTime
	);

	// Writes out anything captured so far. Called on the record 1 -> 0
	// transition, and on EndPlay so a recording is not lost by stopping PIE.
	void Flush();

	bool IsRecording() const { return bIsRecording; }

private:
	void Start(const TautRope::Rope& Rope);

	bool bIsRecording = false;
	TautRope::Recording Recording;
	FString Label;
};
