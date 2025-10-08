
#include "TautRopeHelpersCollision.h"
#include "TautRopeConfig.h"
#include "TautRopePoint.h"

namespace TautRope
{
	void SweepSegmentThroughShapes(
		FHitData& OutHitData,
		FPoint& InOutSegmentPointA,
		FPoint& InOutSegmentPointB,
		const FVector& OriginLocationA,
		const FVector& OriginLocationB,
		const FVector& TargetLocationA,
		const FVector& TargetLocationB,
		const TArray<FRopeCollisionShape>& Shapes,
		const int32 RopePointIndex
	)
	{
		OutHitData.BaryCoords = FVector(MAX_FLT);
		// First perform triangle sweep for A-movement
		for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
		{
			const FRopeCollisionShape& Shape = Shapes[ShapeIndex];
			SweepTriangleAgainstShape(
				OriginLocationA,	// TriA
				TargetLocationA,	// TriB
				OriginLocationB,	// TriC
				Shapes[ShapeIndex],
				ShapeIndex,
				InOutSegmentPointA.ShapeIndex,	// ShapeIndexPointA
				InOutSegmentPointB.ShapeIndex,	// ShapeIndexPointB
				InOutSegmentPointA.EdgeIndex,	// EdgeIndexPointA
				InOutSegmentPointB.EdgeIndex,	// EdgeIndexPointB
				InOutSegmentPointA.VertIndex,	// VertIndexPointA
				InOutSegmentPointB.VertIndex,	// VertIndexPointB
				RopePointIndex,
				true,	// bIsFirstTriangleSweep
				OutHitData
			);
		}

		if (OutHitData.bIsHit)
		{
			InOutSegmentPointA.Location = FMath::Lerp(OriginLocationA, TargetLocationA, OutHitData.BaryCoords.Y);
			return;
		}

		// No new collisions from A-movement triangle sweep
		InOutSegmentPointA.Location = TargetLocationA;

		OutHitData.BaryCoords = FVector(MAX_FLT);
		// Perform triangle sweep for B-movement
		for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
		{
			const FRopeCollisionShape& Shape = Shapes[ShapeIndex];
			SweepTriangleAgainstShape(
				OriginLocationB,	// TriA
				TargetLocationB,	// TriB
				TargetLocationA,	// TriC
				Shape,
				ShapeIndex,
				InOutSegmentPointA.ShapeIndex,	// ShapeIndexPointA
				InOutSegmentPointB.ShapeIndex,	// ShapeIndexPointB
				InOutSegmentPointA.EdgeIndex,	// EdgeIndexPointA
				InOutSegmentPointB.EdgeIndex,	// EdgeIndexPointB
				InOutSegmentPointA.VertIndex,	// VertIndexPointA
				InOutSegmentPointB.VertIndex,	// VertIndexPointB
				RopePointIndex,
				false,	// bIsFirstTriangleSweep
				OutHitData
			);
		}

		if (OutHitData.bIsHit)
		{
			InOutSegmentPointB.Location = FMath::Lerp(OriginLocationB, TargetLocationB, OutHitData.BaryCoords.Y);
			return;
		}

		// No new collisions from B-movement triangle sweep
		InOutSegmentPointB.Location = TargetLocationB;
	}


	void SweepTriangleAgainstShape(
		const FVector& TriA,
		const FVector& TriB,
		const FVector& TriC,
		const FRopeCollisionShape& Shape,
		const int32 ShapeIndex,
		const int32 ShapeIndexPointA,
		const int32 ShapeIndexPointB,
		const int32 EdgeIndexPointA,
		const int32 EdgeIndexPointB,
		const int32 VertIndexPointA,
		const int32 VertIndexPointB,
		const int32 RopePointIndex,
		const bool bIsFirstTriangleSweep,
		FHitData& OutHitData
	)
	{
		for (int32 EdgeIndex = 0; EdgeIndex < Shape.Edges.Num(); ++EdgeIndex)
		{
			if (ShapeIndex == ShapeIndexPointA)
			{
				if (EdgeIndex == EdgeIndexPointA)
					continue;

				if (VertIndexPointA != INDEX_NONE && Shape.VertToEdges[VertIndexPointA].Contains(EdgeIndex))
					continue;
			}

			if (ShapeIndex == ShapeIndexPointB)
			{
				if (EdgeIndex == EdgeIndexPointB)
					continue;

				if (VertIndexPointB != INDEX_NONE && Shape.VertToEdges[VertIndexPointB].Contains(EdgeIndex))
					continue;
			}

			const FIntVector2& EdgeVerts = Shape.Edges[EdgeIndex];
			const FVector& EdgeA = Shape.Vertices[EdgeVerts.X];
			const FVector& EdgeB = Shape.Vertices[EdgeVerts.Y];

			FVector ClosestPointOnLine;
			FVector BaryCoords;
			const bool bIsIntersection = GetTriangleLineIntersection(
				TriA
				, TriB
				, TriC
				, EdgeA
				, EdgeB
				, ClosestPointOnLine
				, BaryCoords
			);

			if (bIsIntersection && BaryCoords.Y < OutHitData.BaryCoords.Y)
			{
				OutHitData.bIsHit = true;
				OutHitData.Location = ClosestPointOnLine;
				OutHitData.BaryCoords = BaryCoords;
				OutHitData.bIsHitOnFirstTriangleSweep = bIsFirstTriangleSweep;
				OutHitData.RopePointIndex = RopePointIndex;
				OutHitData.EdgeIndex = EdgeIndex;
				OutHitData.ShapeIndex = ShapeIndex;
			}
		}
	}

	bool GetTriangleLineIntersection(
		const FVector& TriA,
		const FVector& TriB,
		const FVector& TriC,
		const FVector& LineA,
		const FVector& LineB,
		FVector& OutLocation,
		FVector& OutBaryCoords
	)
	{
		const FVector Dir = LineB - LineA;
		const FVector Edge1 = TriB - TriA;
		const FVector Edge2 = TriC - TriA;

		const FVector PVec = FVector::CrossProduct(Dir, Edge2);
		const float Det = FVector::DotProduct(Edge1, PVec);

		if (FMath::Abs(Det) < KINDA_SMALL_NUMBER)
		{
			return false; // Line parallel to triangle
		}

		const float InvDet = 1.0f / Det;
		const FVector TVec = LineA - TriA;

		const float U = FVector::DotProduct(TVec, PVec) * InvDet;
		if (U < KINDA_SMALL_NUMBER || U > 1.0f)
		{
			return false;
		}

		const FVector QVec = FVector::CrossProduct(TVec, Edge1);
		const float V = FVector::DotProduct(Dir, QVec) * InvDet;
		if (V < -KINDA_SMALL_NUMBER || U + V > 1.0f)
		{
			return false;
		}

		const float T = FVector::DotProduct(Edge2, QVec) * InvDet;
		if (T < -KINDA_SMALL_NUMBER || T > 1.f + KINDA_SMALL_NUMBER)
		{
			return false;
		}

		OutLocation = LineA + Dir * T;
		const float W = 1.0f - U - V;
		OutBaryCoords = FVector(W, U, V);

		return true;
	}

#if TAUT_ROPE_DEBUG_DRAWING
	void DebugDrawSweep(
		const UWorld* World
		, const FVector& OriginLocationA
		, const FVector& OriginLocationB
		, const FVector& TargetLocationA
		, const FVector& TargetLocationB
		, const FHitData& OutHitData
	)
	{
		if (OriginLocationA == TargetLocationA && OriginLocationB == TargetLocationB)
		{
			return;
		}
		TArray<FVector> Vertices;
		Vertices.Add(OriginLocationA);
		Vertices.Add(TargetLocationA);
		Vertices.Add(OriginLocationB);
		Vertices.Add(TargetLocationB);

		TArray<int32> Indices;
		// First triangle (OriginA, TargetA, OriginB)
		Indices.Add(0); Indices.Add(1); Indices.Add(2);
		// Second triangle (OriginB, TargetB, TargetA)
		Indices.Add(2); Indices.Add(3); Indices.Add(1);

		FColor MeshColor = OutHitData.bIsHit ? FColor::Red : FColor::Yellow;
		DrawDebugMesh(
			World,
			Vertices,
			Indices,
			MeshColor,
			false,	// persistent lines
			0.5f   // lifetime
		);
	}
#endif // TAUT_ROPE_DEBUG_DRAWING

}