#pragma once

#include "TautRopeCore/Core.h"

namespace TautRope
{
	inline constexpr float DistanceTolerance = 0.01f;
	inline constexpr float VertexCrossingOffset = 0.1f;
	inline constexpr float ShapeMergeVertexThreshold = 0.1f;
	inline constexpr float ShapeEdgeRayIncrementDistance = 1.f;

	inline constexpr int32 MaxCollisionIterations = 100;

	// Two edges are reached by the same sweep at the same moment when the sweep
	// crosses the vertex they share. Which one has the marginally smaller ratio
	// is arbitrary, so both are reported rather than one being discarded.
	// Measured on the wrap recordings: every runner-up sits within 0.001 of the
	// winner, most of them at exactly the same ratio, and nothing sits between
	// 0.001 and the next real hit.
	inline constexpr float SweepRatioTieTolerance = 1.e-3f;

	inline constexpr float DistanceToleranceSquared = DistanceTolerance * DistanceTolerance;
	inline constexpr float ShapeMergeVertexThresholdSquared = ShapeMergeVertexThreshold * ShapeMergeVertexThreshold;
	inline constexpr float ShapeEdgeRayIncrementDistanceSquared = ShapeEdgeRayIncrementDistance * ShapeEdgeRayIncrementDistance;
}
