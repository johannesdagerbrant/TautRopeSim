
#include "TautRopeHelpersMovement.h"
#include "TautRopePoint.h"

namespace TautRope
{
    FMovementGroup::FMovementGroup(
        const int32 InShapeIndex
        , const int32 InVertIndex
        , const int32 InFirstPointIndex
    )
        : ShapeIndex(InShapeIndex)
        , VertIndex(InVertIndex)
        , FirstPointIndex(InFirstPointIndex)
    {}

    TArray<FMovementGroup> GetMovementGroups(
        const TArray<FPoint>& RopePoints,
        const TArray<FTautRopeCollisionShape>& NearbyShapes
    )
    {
        TArray<FMovementGroup> MovementGroups;
        const int32 NumPoints = RopePoints.Num();
        if (NumPoints < 3)
        {
            return MovementGroups;
        }

        MovementGroups.Add(
            FMovementGroup(RopePoints[1].ShapeIndex, RopePoints[1].VertIndex, 0)
        );

        for (int32 i = 1; i < NumPoints - 1; i++)
        {
            const FPoint& Point = RopePoints[i];
            FMovementGroup& CurrentGroup = MovementGroups.Last();
            CurrentGroup.LastPointIndex = i;

            const FTautRopeCollisionShape& Shape = NearbyShapes[Point.ShapeIndex];
            const TArray<int32> CandidateVerts = GetCandidateVerts(Point, Shape);
            const int32 GroupVertIndex = CurrentGroup.VertIndex;

            const bool bBelongsInLastVertexGroup =
                CurrentGroup.ShapeIndex == Point.ShapeIndex &&
                CandidateVerts.Contains(GroupVertIndex) &&
                !CurrentGroup.EdgeIndices.Contains(Point.EdgeIndex);

            if (bBelongsInLastVertexGroup)
            {
                CurrentGroup.EdgeIndices.Add(Point.EdgeIndex);
            }
            else
            {
                MovementGroups.Add(
                    FMovementGroup(Point.ShapeIndex, CandidateVerts[0], i)
                );
            }
        }

        if (!MovementGroups.IsEmpty())
        {
            MovementGroups[0].FirstPointIndex = 0;
            MovementGroups.Last().LastPointIndex = NumPoints - 1;
        }

        return MovementGroups;
    }

    TArray<int32> GetCandidateVerts(const FPoint& Point, const FTautRopeCollisionShape& Shape)
    {
        if (Point.VertIndex != INDEX_NONE)
        {
            return { Point.VertIndex };
        }
        else
        {
            const FIntVector2& Edge = Shape.Edges[Point.EdgeIndex];
            return { Edge.X, Edge.Y };
        }
    }

	FVector FindMinDistancePointBetweenABOnLineXY(
		const FVector& A
		, const FVector& B
		, const FVector& LineX
		, const FVector& LineY
		, float& OutDistAlongEdge
		, float& OutEdgeLength
	)
	{
		const FVector Edge = LineY - LineX;
		OutEdgeLength = Edge.Size();
		ensure(OutEdgeLength > KINDA_SMALL_NUMBER);

		const FVector EdgeDir = Edge / OutEdgeLength;

		// Projections of A and B onto the edge axis
		const float Alpha = FVector::DotProduct(A - LineX, EdgeDir);
		const float Beta = FVector::DotProduct(B - LineX, EdgeDir);

		// Perpendicular offsets
		const float rhoA = FVector::CrossProduct(A - LineX, EdgeDir).Size();
		const float rhoB = FVector::CrossProduct(B - LineX, EdgeDir).Size();

		if (rhoA < KINDA_SMALL_NUMBER && rhoB < KINDA_SMALL_NUMBER)
		{
			// Both lie essentially on the line -> midpoint
			OutDistAlongEdge = 0.5f * (Alpha + Beta);
		}
		else if (rhoA < KINDA_SMALL_NUMBER)
		{
			OutDistAlongEdge = Alpha;
		}
		else if (rhoB < KINDA_SMALL_NUMBER)
		{
			OutDistAlongEdge = Beta;
		}
		else
		{
			// Weighted average
			const float r = rhoA / (rhoA + rhoB);
			OutDistAlongEdge = (1.0f - r) * Alpha + r * Beta;
		}
		return LineX + EdgeDir * FMath::Clamp(OutDistAlongEdge, TAUT_ROPE_DISTANCE_TOLERANCE, OutEdgeLength - TAUT_ROPE_DISTANCE_TOLERANCE);
	}
}