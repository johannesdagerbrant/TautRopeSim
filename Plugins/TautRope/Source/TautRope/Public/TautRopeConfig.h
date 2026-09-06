#pragma once

#include "TautRopeCore/Config.h"

// Debug drawing is glue-side only; the core simulation emits primitives through
// TautRope::IDebugDraw and has no opinion on whether they are rendered.
#define TAUT_ROPE_DEBUG_DRAWING							!(UE_BUILD_SHIPPING || UE_BUILD_TEST)

// Aliases so the UE-side shape construction keeps reading the way it did, while
// the values live in the core module as the single source of truth.
#define TAUT_ROPE_DISTANCE_TOLERANCE					(TautRope::DistanceTolerance)
#define TAUT_ROPE_SHAPE_MERGE_VERTEX_THRESHOLD_SQUARED	(TautRope::ShapeMergeVertexThresholdSquared)
#define TAUT_ROPE_SHAPE_EDGE_RAY_INCREMENT_DISTANCE_SQUARED	(TautRope::ShapeEdgeRayIncrementDistanceSquared)
