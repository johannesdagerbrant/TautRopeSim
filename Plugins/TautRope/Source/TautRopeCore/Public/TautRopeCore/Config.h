#pragma once

#include "TautRopeCore/Core.h"

namespace TautRope
{
	inline constexpr float DistanceTolerance = 0.01f;
	inline constexpr float VertexCrossingOffset = 0.1f;
	inline constexpr float ShapeMergeVertexThreshold = 0.1f;
	inline constexpr float ShapeEdgeRayIncrementDistance = 1.f;

	inline constexpr int32 MaxCollisionIterations = 100;

	inline constexpr float DistanceToleranceSquared = DistanceTolerance * DistanceTolerance;
	inline constexpr float ShapeMergeVertexThresholdSquared = ShapeMergeVertexThreshold * ShapeMergeVertexThreshold;
	inline constexpr float ShapeEdgeRayIncrementDistanceSquared = ShapeEdgeRayIncrementDistance * ShapeEdgeRayIncrementDistance;
}
