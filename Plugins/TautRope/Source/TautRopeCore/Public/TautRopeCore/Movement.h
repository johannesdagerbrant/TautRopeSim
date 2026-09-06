#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

#include <vector>

namespace TautRope
{
	struct Point;

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

	TAUTROPE_CORE_API std::vector<MovementGroup> GetMovementGroups(
		const std::vector<Point>& RopePoints
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
