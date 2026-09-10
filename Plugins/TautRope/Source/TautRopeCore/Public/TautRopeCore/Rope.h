#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"
#include "TautRopeCore/Point.h"
#include "TautRopeCore/Recording.h"

#include <vector>

namespace TautRope
{
	class IDebugDraw;

	class TAUTROPE_CORE_API Rope
	{
	public:
		void AppendToNearbyShapes(const std::vector<CollisionShape>& Shapes);

		std::vector<Vec3> GetRopePoints() const;

		void UpdateRope(
			const Vec3& StartLocation
			, const Vec3& EndLocation
			, const float MaxLength
			, IDebugDraw* Debug = nullptr
			, FrameCapture* Capture = nullptr
		);

		// Seeds the rope from a recording's initial state, identities included, so
		// a replay continues allocating the same ids the editor run did.
		void RestoreState(const std::vector<RecordedPoint>& Points, int32 InNextPointId);

		int32 GetNextPointId() const { return NextPointId; }

		// Whether these run at all is the caller's decision, so that the CVars
		// stay in the glue module.
		void DrawDebugRope(IDebugDraw& Debug) const;
		void DrawDebugRopeTouchedShapeEdges(IDebugDraw& Debug) const;

		const std::vector<Point>& GetPoints() const { return RopePoints; }
		const std::vector<CollisionShape>& GetNearbyShapes() const { return NearbyShapes; }

		// Number of frames whose collision phase ran out of iterations instead of
		// settling. Non-zero means points were still being inserted when the loop
		// gave up, which is how runaway insertion presents: the frame does not
		// crash, it just gets slower until it looks like a hang. Watched by the
		// tests and printed by the replay tool.
		int32 CollisionIterationCapHits = 0;
		int32 MostCollisionIterations = 0;

		// The same two numbers for the remove sweep inside the pruning phase.
		int32 RemoveSweepIterationCapHits = 0;
		int32 MostRemoveSweepIterations = 0;

		// Frames that hit MaxRopePoints, and the largest the rope ever got.
		int32 RopePointCeilingHits = 0;
		int32 MostRopePoints = 0;

	private:
		std::vector<Vec3> MovementPhase(
			const Vec3& StartLocation
			, const Vec3& EndLocation
			, const float MaxLength
		);

		bool CollisionPhase(std::vector<Vec3>& TargetRopePoints, IDebugDraw* Debug);

		bool PruningPhase(IDebugDraw* Debug);

		std::vector<Point> RopePoints;
		std::vector<CollisionShape> NearbyShapes;

		int32 NextPointId = 0;
	};
}
