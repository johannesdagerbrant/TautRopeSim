#pragma once

#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

#include <vector>

namespace TautRope
{
	// Pure shape data. Building this from UE collision primitives stays in the
	// glue module; the simulation only ever reads it.
	struct CollisionShape
	{
		std::vector<Vec3> Vertices;
		std::vector<Int2> Edges;
		std::vector<std::vector<int32>> VertToEdges;
		std::vector<Quat> EdgeRotations;
		std::vector<bool> IsCornerVertexList;

		// True if the vertex has FEWER than two adjacent edges, i.e. a dangling or
		// terminal vertex the rope cannot slide over. The builder sets it as
		// VertToEdges[v].size() < 2, so on a closed hull every vertex is false and
		// the movement phase is free to snap a point onto any of them.
		//
		// The comment here used to say "more than one adjacent edge", the exact
		// inverse, which reads as though a hull's vertices are all corners and the
		// vertex-crossing path is dead code. It is not: it is enabled everywhere.
		// VertexIndex is assumed to be in range.
		bool IsCornerVertex(const int32 VertexIndex) const
		{
			return IsCornerVertexList[VertexIndex];
		}
	};
}
