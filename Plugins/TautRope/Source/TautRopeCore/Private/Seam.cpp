#include "TautRopeCore/Seam.h"

#include "TautRopeCore/Config.h"
#include "TautRopeCore/Math.h"

namespace TautRope
{
	namespace
	{
		// One split: edge keeps its index and becomes the A-side half, the B-side
		// half and the new vertex are appended. The new vertex takes the foreign
		// vertex's exact coordinates so the two shapes' junctions coincide.
		void SplitEdgeAtVertex(
			CollisionShape& Shape
			, const int32 EdgeIndex
			, const Vec3& ForeignVert
		)
		{
			const Int2 Edge = Shape.Edges[EdgeIndex];
			const int32 NewVertIndex = Num(Shape.Vertices);
			const int32 NewEdgeIndex = Num(Shape.Edges);

			Shape.Vertices.push_back(ForeignVert);
			Shape.Edges[EdgeIndex] = Int2(Edge.X, NewVertIndex);
			Shape.Edges.push_back(Int2(NewVertIndex, Edge.Y));
			// Both halves keep the original line and X-to-Y orientation, so the
			// original rotation is correct for both.
			Shape.EdgeRotations.push_back(Shape.EdgeRotations[EdgeIndex]);

			for (int32& AdjacentEdge : Shape.VertToEdges[Edge.Y])
			{
				if (AdjacentEdge == EdgeIndex)
				{
					AdjacentEdge = NewEdgeIndex;
				}
			}
			Shape.VertToEdges.push_back({ EdgeIndex, NewEdgeIndex });
			// Two adjacent edges, so the rope may slide across it.
			Shape.IsCornerVertexList.push_back(false);
		}
	}

	void WeldSeamsAcrossShapes(std::vector<CollisionShape>& Shapes)
	{
		const float Tolerance = ShapeMergeVertexThreshold;
		bool bAnySplit = true;
		while (bAnySplit)
		{
			bAnySplit = false;
			for (int32 s = 0; s < Num(Shapes); ++s)
			{
				CollisionShape& Shape = Shapes[s];
				for (int32 t = 0; t < Num(Shapes); ++t)
				{
					if (t == s)
					{
						continue;
					}
					for (const Vec3& ForeignVert : Shapes[t].Vertices)
					{
						// Edge count grows as we split; new halves are re-examined
						// against later vertices by the fixpoint loop.
						const int32 NumEdgesNow = Num(Shape.Edges);
						for (int32 e = 0; e < NumEdgesNow; ++e)
						{
							const Int2& Edge = Shape.Edges[e];
							const Vec3& A = Shape.Vertices[Edge.X];
							const Vec3& B = Shape.Vertices[Edge.Y];
							const Vec3 Along = B - A;
							const float Length = static_cast<float>(Along.Size());
							if (Length <= 2.f * Tolerance)
							{
								continue;
							}
							const Vec3 Dir = Along / Length;
							const float T = static_cast<float>(Vec3::Dot(ForeignVert - A, Dir));
							if (T < Tolerance || T > Length - Tolerance)
							{
								continue;
							}
							const Vec3 OnLine = A + Dir * T;
							if (static_cast<float>((ForeignVert - OnLine).SizeSquared()) > Tolerance * Tolerance)
							{
								continue;
							}
							SplitEdgeAtVertex(Shape, e, ForeignVert);
							bAnySplit = true;
						}
					}
				}
			}
		}
	}
}
