#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

#include <vector>

namespace TautRope
{
	struct Point;

	// The pruning phase's removal decision: vertex-cone groups, same-edge
	// duplicates, and points the rope no longer wraps. A wrap verdict whose
	// neighbour is itself marked in the same pass is deferred to the next frame
	// rather than judged against a rope that never exists.
	TAUTROPE_CORE_API std::vector<bool> GetPointsToRemove(
		const std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	);

	TAUTROPE_CORE_API bool IsRopeWrappingEdge(
		const Vec3& PointLocationA
		, const Vec3& PointLocationB
		, const Vec3& PointLocationC
		, const Quat& EdgeRotation
	);
}
