
#include "TautRopeHelpersVertexHandling.h"
#include "TautRopePoint.h"

namespace TautRope
{
	TBitArray<> GetAdjacentPointsOnSameVertexCone(
		const TArray<FPoint>& RopePoints
		, const TArray<FTautRopeCollisionShape>& NearbyShapes
	)
	{
		TBitArray<> ToRemove;
		ToRemove.Init(false, RopePoints.Num());
		for (int32 i = 0; i < RopePoints.Num(); ++i)
		{
			const TautRope::FPoint& PointAtVert = RopePoints[i];
			if (PointAtVert.VertIndex == INDEX_NONE)
			{
				continue;
			}
			const FTautRopeCollisionShape& Shape = NearbyShapes[PointAtVert.ShapeIndex];
			const TArray<int32>& AdjacentEdges = Shape.VertToEdges[PointAtVert.VertIndex].Edges;

			int32 GroupStart = i;
			for (int32 j = i - 1; j >= 0; --j)
			{
				const TautRope::FPoint& Prev = RopePoints[j];
				if (Prev.VertIndex == PointAtVert.VertIndex ||
					AdjacentEdges.Contains(Prev.EdgeIndex))
				{
					GroupStart = j;
				}
				else
				{
					break;
				}
			}
			int32 GroupEnd = i;
			for (int32 j = i + 1; j < RopePoints.Num(); ++j)
			{
				const TautRope::FPoint& Next = RopePoints[j];
				if (Next.VertIndex == PointAtVert.VertIndex ||
					AdjacentEdges.Contains(Next.EdgeIndex))
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
		TArray<FPoint>& RopePoints
		, const TArray<FTautRopeCollisionShape>& NearbyShapes
	)
	{
		for (int32 i = 1; i < RopePoints.Num() - 1; ++i)
		{
			TautRope::FPoint& PointB = RopePoints[i];
			if (PointB.VertIndex == INDEX_NONE) // The point is not touching a vertex
			{
				continue;
			}
			const FVector& LocationA = RopePoints[i - 1].Location;
			const FVector& LocationB = PointB.Location;
			const FVector& LocationC = RopePoints[i + 1].Location;

			const FTautRopeCollisionShape& Shape = NearbyShapes[PointB.ShapeIndex];
			const FIntVector2& FromEdge = Shape.Edges[PointB.EdgeIndex];
			const FVector& FromEdgeVertX = Shape.Vertices[FromEdge.X];
			const FVector& FromEdgeVertY = Shape.Vertices[FromEdge.Y];
			const FVector FromEdgeDirection = PointB.VertIndex == FromEdge.X
				? (FromEdgeVertX - FromEdgeVertY).GetSafeNormal()
				: (FromEdgeVertY - FromEdgeVertX).GetSafeNormal();

			const FVector RopeUp = FVector::CrossProduct(LocationA - LocationB, LocationC - LocationB).GetSafeNormal();
			const float FromEdgeRopeUpDot = FVector::DotProduct(FromEdgeDirection, RopeUp);
			FVector RopeSlidingDirection = FVector::ZeroVector;
			if (FMath::IsNearlyZero(FromEdgeRopeUpDot, KINDA_SMALL_NUMBER))
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
			const FVector& VertexLocation = Shape.Vertices[PointB.VertIndex];
			int32 MostOffendingEdgeIndex = INDEX_NONE;
			float MostOffendingEdgeDot = 0.f;
			const TArray<int32>& AdjacentEdges = Shape.VertToEdges[PointB.VertIndex].Edges;
			for (const int32 AdjacentEdgeIndex : AdjacentEdges)
			{
				const FIntVector2& AdjacentEdge = Shape.Edges[AdjacentEdgeIndex];
				const FVector& AdjacentEdgeVertA = Shape.Vertices[AdjacentEdge.X];
				const FVector& OtherEndLocation = PointB.VertIndex == AdjacentEdge.X ? Shape.Vertices[AdjacentEdge.Y] : Shape.Vertices[AdjacentEdge.X];
				const FVector EdgeDir = OtherEndLocation - VertexLocation;
				const float EdgeDot = FVector::DotProduct(EdgeDir, RopeSlidingDirection);
				if (EdgeDot > MostOffendingEdgeDot)
				{
					MostOffendingEdgeDot = EdgeDot;
					MostOffendingEdgeIndex = AdjacentEdgeIndex;
				}
			}
			if (MostOffendingEdgeIndex != INDEX_NONE)
			{
				PointB.VertIndex = INDEX_NONE;
				PointB.EdgeIndex = MostOffendingEdgeIndex;
			}
		}
	}
}