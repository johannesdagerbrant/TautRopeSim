#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

#include <string>
#include <vector>

namespace TautRope
{
	struct Point;

	// A rope point as captured. Id is stable for as long as the point lives, so a
	// diff across frames can tell "the point moved" from "the point was replaced".
	struct RecordedPoint
	{
		int32 Id = IndexNone;
		Vec3 Location;
		int32 ShapeIndex = IndexNone;
		int32 EdgeIndex = IndexNone;
		int32 VertIndex = IndexNone;
	};

	// Filled by Rope::UpdateRope when recording. Snapshotting after each phase
	// rather than only at end-of-frame catches points that are inserted by the
	// collision phase and pruned again before the frame ends -- which is exactly
	// what the convergence bugs do.
	struct FrameCapture
	{
		std::vector<RecordedPoint> AfterMovement;
		std::vector<RecordedPoint> AfterCollision;
		std::vector<RecordedPoint> AfterPruning;

		void Clear()
		{
			AfterMovement.clear();
			AfterCollision.clear();
			AfterPruning.clear();
		}
	};

	struct RecordedFrame
	{
		// Simulation inputs. These plus the initial state are all a replay needs.
		Vec3 StartLocation;
		Vec3 EndLocation;
		float MaxLength = 0.f;

		// Metadata only. UpdateRope does not consume DeltaTime; it is recorded so
		// a recording can be related back to what was seen in the editor.
		float DeltaTime = 0.f;

		FrameCapture Capture;
	};

	struct Recording
	{
		std::string EngineBuild;

		// Shapes are static, so they are captured once when recording starts.
		std::vector<CollisionShape> Shapes;

		// Rope state entering the first recorded frame. The simulation is
		// stateful, so a replay that starts from anything else diverges.
		std::vector<RecordedPoint> InitialPoints;

		// The point id allocator entering the first recorded frame. It cannot be
		// inferred from InitialPoints: ids allocated to points that were pruned
		// before recording started have already advanced the counter.
		int32 NextPointId = 0;

		std::vector<RecordedFrame> Frames;
	};

	TAUTROPE_CORE_API void CapturePoints(const std::vector<Point>& Points, std::vector<RecordedPoint>& OutPoints);

	// Text, so a human and an agent can read a recording without a decoder.
	// Doubles round-trip exactly (%.17g); anything lossy here would destroy the
	// bit-identity the replay is meant to prove.
	TAUTROPE_CORE_API bool WriteRecording(const Recording& InRecording, const char* Path, std::string& OutError);
	TAUTROPE_CORE_API bool ReadRecording(Recording& OutRecording, const char* Path, std::string& OutError);

	inline constexpr int32 RecordingFormatVersion = 2;
}
