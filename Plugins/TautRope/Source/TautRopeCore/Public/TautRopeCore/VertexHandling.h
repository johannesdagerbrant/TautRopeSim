#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"

#include <vector>

namespace TautRope
{
	struct Point;

	TAUTROPE_CORE_API std::vector<bool> GetAdjacentPointsOnSameVertexCone(
		const std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	);

	TAUTROPE_CORE_API void LetPointsOnVertexSlideOntoNewEdge(
		std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	);
}
