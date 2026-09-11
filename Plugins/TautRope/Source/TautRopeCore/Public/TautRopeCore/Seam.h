#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"

#include <vector>

namespace TautRope
{
	// Where two hulls touch, their boundary edges overlap on the same line but
	// with different extents, so one shape's vertex can sit in the middle of the
	// other's edge. A rope point sliding along that edge crosses the foreign
	// vertex without the vertex machinery firing, which is how the double-cone
	// junction exploded. Splitting every edge at the foreign vertices that lie on
	// it turns each overlapping seam into 1:1 coincident segments: seam twins
	// then share extents and reach vertices together.
	//
	// Splits append vertices and edges, never renumber, so indices held by rope
	// points and recordings stay valid. Runs to a fixpoint; deterministic for a
	// given shape set.
	TAUTROPE_CORE_API void WeldSeamsAcrossShapes(std::vector<CollisionShape>& Shapes);
}
