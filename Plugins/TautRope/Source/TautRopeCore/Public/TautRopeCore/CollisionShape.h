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

		// True if the vertex has more than one adjacent edge.
		// VertexIndex is assumed to be in range.
		bool IsCornerVertex(const int32 VertexIndex) const
		{
			return IsCornerVertexList[VertexIndex];
		}
	};
}
