#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

#include <vector>

namespace TautRope
{
	struct Point;

	// A run of consecutive rope points riding distinct edges that all share one
	// vertex — the vertex fan the movement phase straightens in one solve
	// instead of point by point.
	struct TAUTROPE_CORE_API MovementGroup
	{
		MovementGroup(
			const int32 InShapeIndex
			, const int32 InVertIndex
			, const int32 InFirstPointIndex
		);

		int32 ShapeIndex = IndexNone;
		int32 VertIndex = IndexNone;
		int32 FirstPointIndex = IndexNone;
		int32 LastPointIndex = IndexNone;
		std::vector<int32> EdgeIndices;
	};

	// Finds every fan of two or more consecutive points whose edges share a
	// vertex. The shared vertex is the intersection of the candidate sets of
	// every edge in the run, narrowed point by point; a run ends when its edges
	// stop sharing a vertex, the shape changes, an edge repeats, or a point is
	// already sitting on a vertex.
	TAUTROPE_CORE_API std::vector<MovementGroup> GetMovementGroups(
		const std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	);

	// Straightens one fan group: unfolds the fan into a plane around the shared
	// vertex, pivots the two neighbouring anchor targets into that plane about
	// the outermost fan edges, and places each point where the straight
	// anchor-to-anchor segment crosses its edge. An edge the segment no longer
	// crosses means the taut rope pinches onto the shared vertex, so that point
	// enters the vertex-crossing state instead. Returns false on degenerate
	// geometry (an anchor on the vertex, a zero-length edge); the caller falls
	// back to the per-point solve.
	TAUTROPE_CORE_API bool SolveFanMovementGroup(
		std::vector<Point>& RopePoints
		, std::vector<Vec3>& RopeTargetLocations
		, const MovementGroup& Group
		, const std::vector<CollisionShape>& NearbyShapes
	);

	TAUTROPE_CORE_API std::vector<int32> GetCandidateVerts(
		const Point& P
		, const CollisionShape& Shape
	);

	TAUTROPE_CORE_API Vec3 FindMinDistancePointBetweenABOnLineXY(
		const Vec3& A
		, const Vec3& B
		, const Vec3& X
		, const Vec3& Y
		, float& OutDistAlongEdge
		, float& OutEdgeLength
	);
}
