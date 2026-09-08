#include "TautRopeCore/Compare.h"

#include <cstdio>
#include <cstdint>
#include <cstring>

namespace TautRope
{
	namespace
	{
		bool SameBits(const double A, const double B)
		{
			std::uint64_t BitsA = 0;
			std::uint64_t BitsB = 0;
			std::memcpy(&BitsA, &A, sizeof(BitsA));
			std::memcpy(&BitsB, &B, sizeof(BitsB));
			return BitsA == BitsB;
		}

		bool SameBits(const float A, const float B)
		{
			std::uint32_t BitsA = 0;
			std::uint32_t BitsB = 0;
			std::memcpy(&BitsA, &A, sizeof(BitsA));
			std::memcpy(&BitsB, &B, sizeof(BitsB));
			return BitsA == BitsB;
		}

		std::string Format(const double Value)
		{
			char Buffer[64];
			std::snprintf(Buffer, sizeof(Buffer), "%.17g", Value);
			return std::string(Buffer);
		}

		std::string Format(const float Value)
		{
			char Buffer[64];
			std::snprintf(Buffer, sizeof(Buffer), "%.9g", Value);
			return std::string(Buffer);
		}

		std::string Format(const int32 Value)
		{
			char Buffer[32];
			std::snprintf(Buffer, sizeof(Buffer), "%d", Value);
			return std::string(Buffer);
		}

		void Record(
			RecordingDivergence& Out
			, const char* Phase
			, const char* Field
			, const int32 FrameIndex
			, const int32 PointIndex
			, std::string Recorded
			, std::string Replayed
		)
		{
			// Only the first divergence is kept; later ones are usually
			// consequences of it, not independent facts.
			if (Out.bDiverged)
			{
				return;
			}
			Out.bDiverged = true;
			Out.Phase = Phase;
			Out.Field = Field;
			Out.FrameIndex = FrameIndex;
			Out.PointIndex = PointIndex;
			Out.RecordedValue = std::move(Recorded);
			Out.ReplayedValue = std::move(Replayed);
		}

		// Returns true when the two point sets agree.
		bool ComparePoints(
			const std::vector<RecordedPoint>& A
			, const std::vector<RecordedPoint>& B
			, const char* Phase
			, const int32 FrameIndex
			, RecordingDivergence& Out
		)
		{
			if (A.size() != B.size())
			{
				Record(Out, Phase, "point count", FrameIndex, IndexNone,
					Format(static_cast<int32>(A.size())), Format(static_cast<int32>(B.size())));
				return false;
			}

			bool bAgrees = true;
			for (int32 Index = 0; Index < static_cast<int32>(A.size()); ++Index)
			{
				const RecordedPoint& PointA = A[Index];
				const RecordedPoint& PointB = B[Index];

				if (PointA.Id != PointB.Id)
				{
					Record(Out, Phase, "id", FrameIndex, Index, Format(PointA.Id), Format(PointB.Id));
					bAgrees = false;
				}
				if (!SameBits(PointA.Location.X, PointB.Location.X))
				{
					Record(Out, Phase, "location.x", FrameIndex, Index, Format(PointA.Location.X), Format(PointB.Location.X));
					bAgrees = false;
				}
				if (!SameBits(PointA.Location.Y, PointB.Location.Y))
				{
					Record(Out, Phase, "location.y", FrameIndex, Index, Format(PointA.Location.Y), Format(PointB.Location.Y));
					bAgrees = false;
				}
				if (!SameBits(PointA.Location.Z, PointB.Location.Z))
				{
					Record(Out, Phase, "location.z", FrameIndex, Index, Format(PointA.Location.Z), Format(PointB.Location.Z));
					bAgrees = false;
				}
				if (PointA.ShapeIndex != PointB.ShapeIndex)
				{
					Record(Out, Phase, "shape index", FrameIndex, Index, Format(PointA.ShapeIndex), Format(PointB.ShapeIndex));
					bAgrees = false;
				}
				if (PointA.EdgeIndex != PointB.EdgeIndex)
				{
					Record(Out, Phase, "edge index", FrameIndex, Index, Format(PointA.EdgeIndex), Format(PointB.EdgeIndex));
					bAgrees = false;
				}
				if (PointA.VertIndex != PointB.VertIndex)
				{
					Record(Out, Phase, "vert index", FrameIndex, Index, Format(PointA.VertIndex), Format(PointB.VertIndex));
					bAgrees = false;
				}
			}
			return bAgrees;
		}
	}

	bool CompareRecordings(
		const Recording& Recorded
		, const Recording& Replayed
		, RecordingDivergence& OutDivergence
	)
	{
		OutDivergence = RecordingDivergence();

		if (Recorded.Shapes.size() != Replayed.Shapes.size())
		{
			Record(OutDivergence, "structure", "shape count", IndexNone, IndexNone,
				Format(static_cast<int32>(Recorded.Shapes.size())),
				Format(static_cast<int32>(Replayed.Shapes.size())));
			return false;
		}
		if (Recorded.NextPointId != Replayed.NextPointId)
		{
			Record(OutDivergence, "structure", "next point id", IndexNone, IndexNone,
				Format(Recorded.NextPointId), Format(Replayed.NextPointId));
			return false;
		}
		if (!ComparePoints(Recorded.InitialPoints, Replayed.InitialPoints, "initial", IndexNone, OutDivergence))
		{
			return false;
		}
		if (Recorded.Frames.size() != Replayed.Frames.size())
		{
			Record(OutDivergence, "structure", "frame count", IndexNone, IndexNone,
				Format(static_cast<int32>(Recorded.Frames.size())),
				Format(static_cast<int32>(Replayed.Frames.size())));
			return false;
		}

		OutDivergence.ComparedFrameCount = static_cast<int32>(Recorded.Frames.size());

		for (int32 FrameIndex = 0; FrameIndex < static_cast<int32>(Recorded.Frames.size()); ++FrameIndex)
		{
			const RecordedFrame& A = Recorded.Frames[FrameIndex];
			const RecordedFrame& B = Replayed.Frames[FrameIndex];

			bool bFrameAgrees = true;

			// The inputs are carried through rather than recomputed, so a mismatch
			// here means the format lost something rather than the simulation
			// drifting. Worth catching separately.
			if (!SameBits(A.StartLocation.X, B.StartLocation.X)
				|| !SameBits(A.StartLocation.Y, B.StartLocation.Y)
				|| !SameBits(A.StartLocation.Z, B.StartLocation.Z))
			{
				Record(OutDivergence, "input", "start location", FrameIndex, IndexNone,
					Format(A.StartLocation.X), Format(B.StartLocation.X));
				bFrameAgrees = false;
			}
			if (!SameBits(A.EndLocation.X, B.EndLocation.X)
				|| !SameBits(A.EndLocation.Y, B.EndLocation.Y)
				|| !SameBits(A.EndLocation.Z, B.EndLocation.Z))
			{
				Record(OutDivergence, "input", "end location", FrameIndex, IndexNone,
					Format(A.EndLocation.X), Format(B.EndLocation.X));
				bFrameAgrees = false;
			}
			if (!SameBits(A.MaxLength, B.MaxLength))
			{
				Record(OutDivergence, "input", "max length", FrameIndex, IndexNone,
					Format(A.MaxLength), Format(B.MaxLength));
				bFrameAgrees = false;
			}

			if (!ComparePoints(A.Capture.AfterMovement, B.Capture.AfterMovement, "movement", FrameIndex, OutDivergence))
			{
				bFrameAgrees = false;
			}
			if (!ComparePoints(A.Capture.AfterCollision, B.Capture.AfterCollision, "collision", FrameIndex, OutDivergence))
			{
				bFrameAgrees = false;
			}
			if (!ComparePoints(A.Capture.AfterPruning, B.Capture.AfterPruning, "pruning", FrameIndex, OutDivergence))
			{
				bFrameAgrees = false;
			}

			if (!bFrameAgrees)
			{
				++OutDivergence.DivergentFrameCount;
			}
		}

		return !OutDivergence.bDiverged;
	}
}
