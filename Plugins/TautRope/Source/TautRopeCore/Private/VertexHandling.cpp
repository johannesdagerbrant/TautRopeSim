#include "TautRopeCore/VertexHandling.h"

#include "TautRopeCore/Config.h"
#include "TautRopeCore/Math.h"
#include "TautRopeCore/Point.h"

namespace TautRope
{
	std::vector<bool> GetAdjacentPointsOnSameVertexCone(
		const std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	)
	{
		std::vector<bool> ToRemove;
		ToRemove.assign(RopePoints.size(), false);
		for (int32 i = 0; i < Num(RopePoints); ++i)
		{
			const Point& PointAtVert = RopePoints[i];
			if (PointAtVert.VertIndex == IndexNone)
			{
				continue;
			}
			const CollisionShape& Shape = NearbyShapes[PointAtVert.ShapeIndex];
			const std::vector<int32>& AdjacentEdges = Shape.VertToEdges[PointAtVert.VertIndex];

			int32 GroupStart = i;
			for (int32 j = i - 1; j >= 0; --j)
			{
				const Point& Prev = RopePoints[j];
				if (Prev.VertIndex == PointAtVert.VertIndex ||
					Contains(AdjacentEdges, Prev.EdgeIndex))
				{
					GroupStart = j;
				}
				else
				{
					break;
				}
			}
			int32 GroupEnd = i;
			for (int32 j = i + 1; j < Num(RopePoints); ++j)
			{
				const Point& Next = RopePoints[j];
				if (Next.VertIndex == PointAtVert.VertIndex ||
					Contains(AdjacentEdges, Next.EdgeIndex))
				{
					GroupEnd = j;
				}
				else
				{
					break;
				}
			}
			for (int32 j = GroupStart; j <= GroupEnd; ++j)
			{
				ToRemove[j] = true;
			}
			i = GroupEnd;
		}
		return ToRemove;
	}

	void LetPointsOnVertexSlideOntoNewEdge(
		std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	)
	{
		for (int32 i = 1; i < Num(RopePoints) - 1; ++i)
		{
			Point& PointB = RopePoints[i];
			if (PointB.VertIndex == IndexNone) // The point is not touching a vertex
			{
				continue;
			}
			const Vec3& LocationA = RopePoints[i - 1].Location;
			const Vec3& LocationB = PointB.Location;
			const Vec3& LocationC = RopePoints[i + 1].Location;

			const CollisionShape& Shape = NearbyShapes[PointB.ShapeIndex];
			const Int2& FromEdge = Shape.Edges[PointB.EdgeIndex];
			const Vec3& FromEdgeVertX = Shape.Vertices[FromEdge.X];
			const Vec3& FromEdgeVertY = Shape.Vertices[FromEdge.Y];
			const Vec3 FromEdgeDirection = PointB.VertIndex == FromEdge.X
				? (FromEdgeVertX - FromEdgeVertY).GetSafeNormal()
				: (FromEdgeVertY - FromEdgeVertX).GetSafeNormal();

			const Vec3 RopeUp = Vec3::Cross(LocationA - LocationB, LocationC - LocationB).GetSafeNormal();
			const float FromEdgeRopeUpDot = static_cast<float>(Vec3::Dot(FromEdgeDirection, RopeUp));
			Vec3 RopeSlidingDirection;
			if (Math::IsNearlyZero(FromEdgeRopeUpDot, KindaSmallNumber))
			{
				RopeSlidingDirection = FromEdgeDirection;
			}
			else if (FromEdgeRopeUpDot > 0.f)
			{
				RopeSlidingDirection = RopeUp;
			}
			else
			{
				RopeSlidingDirection = -RopeUp;
			}
			const Vec3& VertexLocation = Shape.Vertices[PointB.VertIndex];
			int32 MostOffendingEdgeIndex = IndexNone;
			float MostOffendingEdgeDot = 0.f;
			const std::vector<int32>& AdjacentEdges = Shape.VertToEdges[PointB.VertIndex];
			for (const int32 AdjacentEdgeIndex : AdjacentEdges)
			{
				const Int2& AdjacentEdge = Shape.Edges[AdjacentEdgeIndex];
				const Vec3& OtherEndLocation = PointB.VertIndex == AdjacentEdge.X ? Shape.Vertices[AdjacentEdge.Y] : Shape.Vertices[AdjacentEdge.X];
				const Vec3 EdgeDir = OtherEndLocation - VertexLocation;
				const float EdgeDot = static_cast<float>(Vec3::Dot(EdgeDir, RopeSlidingDirection));
				if (EdgeDot > MostOffendingEdgeDot)
				{
					MostOffendingEdgeDot = EdgeDot;
					MostOffendingEdgeIndex = AdjacentEdgeIndex;
				}
			}
			if (MostOffendingEdgeIndex != IndexNone)
			{
				PointB.VertIndex = IndexNone;
				PointB.EdgeIndex = MostOffendingEdgeIndex;
			}
		}
	}
}
