
#include "TautRopeHelpersCollision.h"
#include "TautRopeConfig.h"
#include "TautRopePoint.h"

namespace TautRope
{
	void SweepRemovePoint
	(
		TArray<FPoint>& RopePoints
		, const int32 RemovePointIndex
		, const TArray<FRopeCollisionShape>& Shapes
#if TAUT_ROPE_DEBUG_DRAWING
		, const UWorld* World
		, const bool bIsDebugDrawingActive
#endif
	)
	{
		ensure(RemovePointIndex > 0 && RemovePointIndex < RopePoints.Num());

		bool bFoundIntersections = false;
		FVector FromLocation = RopePoints[RemovePointIndex].Location;
		FVector ToLocation = RopePoints[RemovePointIndex - 1].Location;
		FVector SupportLocation = RopePoints[RemovePointIndex + 1].Location;

		TArray<FIntVector2> IgnoredEdges = {
			FIntVector2(RopePoints[RemovePointIndex - 1].ShapeIndex, RopePoints[RemovePointIndex - 1].EdgeIndex)
			, FIntVector2(RopePoints[RemovePointIndex].ShapeIndex, RopePoints[RemovePointIndex].EdgeIndex)
			, FIntVector2(RopePoints[RemovePointIndex + 1].ShapeIndex, RopePoints[RemovePointIndex + 1].EdgeIndex)
		};

		FHitData HitData;
		HitData.bIsHit = true;
		while(HitData.bIsHit)
		{
			HitData = FHitData();
			for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
			{
				SweepRemoveTriangleAgainstShape(
					FromLocation
					, ToLocation
					, SupportLocation
					, Shapes[ShapeIndex]
					, ShapeIndex
					, IgnoredEdges
					, HitData
				);
			}
#if TAUT_ROPE_DEBUG_DRAWING
			if (IsValid(World) && bIsDebugDrawingActive)
			{
				if (HitData.bIsHit)
				{
					DebugDrawSweep(World, FromLocation, HitData.OnSweepEdgeLocation, SupportLocation, true);
				}
				else
				{
					DebugDrawSweep(World, FromLocation, ToLocation, SupportLocation, false);
				}
			}
#endif
			if (HitData.bIsHit)
			{
				if (!bFoundIntersections)
				{
					bFoundIntersections = true;
					RopePoints[RemovePointIndex] = FPoint(HitData);
				}
				else
				{
					RopePoints.Insert(TautRope::FPoint(HitData), RemovePointIndex);
				}
				FromLocation = HitData.OnSweepEdgeLocation;
				ToLocation = RopePoints[RemovePointIndex - 1].Location;
				SupportLocation = RopePoints[RemovePointIndex].Location;
				IgnoredEdges = {
					FIntVector2(RopePoints[RemovePointIndex - 1].ShapeIndex, RopePoints[RemovePointIndex - 1].EdgeIndex)
					, FIntVector2(RopePoints[RemovePointIndex].ShapeIndex, RopePoints[RemovePointIndex].EdgeIndex)
				};
			}
		}
		if (!bFoundIntersections)
		{
			RopePoints.RemoveAt(RemovePointIndex);
		}
	}

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
#if TAUT_ROPE_DEBUG_DRAWING
		, const UWorld* World
		, const bool bIsDebugDrawingActive
#endif
	)
	{
		OutHitData.SweepRatio = MAX_FLT;
		// First perform triangle sweep for A-movement
		for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
		{
			const FRopeCollisionShape& Shape = Shapes[ShapeIndex];
			SweepSegmentTriangleAgainstShape(
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
#if TAUT_ROPE_DEBUG_DRAWING
		if (IsValid(World) && bIsDebugDrawingActive)
		{
			if (OutHitData.bIsHit)
			{
				DebugDrawSweep(World, OriginLocationA, OutHitData.OnSweepEdgeLocation, OriginLocationB, true);
			}
			else
			{
				DebugDrawSweep(World, OriginLocationA, TargetLocationA, OriginLocationB, false);
			}
		}
#endif
		if (OutHitData.bIsHit)
		{
			InOutSegmentPointA.Location = OutHitData.OnSweepEdgeLocation;
			return;
		}

		// No new collisions from A-movement triangle sweep
		InOutSegmentPointA.Location = TargetLocationA;

		OutHitData.SweepRatio = MAX_FLT;
		// Perform triangle sweep for B-movement
		for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
		{
			const FRopeCollisionShape& Shape = Shapes[ShapeIndex];
			SweepSegmentTriangleAgainstShape(
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
#if TAUT_ROPE_DEBUG_DRAWING
		if (IsValid(World) && bIsDebugDrawingActive)
		{
			if (OutHitData.bIsHit)
			{
				DebugDrawSweep(World, OriginLocationB, OutHitData.OnSweepEdgeLocation, TargetLocationA, true);
			}
			else
			{
				DebugDrawSweep(World, OriginLocationB, TargetLocationB, TargetLocationA, false);
			}
		}
#endif
		if (OutHitData.bIsHit)
		{
			InOutSegmentPointB.Location = OutHitData.OnSweepEdgeLocation;
			return;
		}

		// No new collisions from B-movement triangle sweep
		InOutSegmentPointB.Location = TargetLocationB;
	}


	void SweepSegmentTriangleAgainstShape(
		const FVector& FromCorner,
		const FVector& ToCorner,
		const FVector& SupportCorner,
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
			FVector OnSweepEdgeLocation;
			float SweepRatio = MAX_FLT;
			const bool bIsIntersection = GetTriangleLineIntersection(
				FromCorner
				, ToCorner
				, SupportCorner
				, EdgeA
				, EdgeB
				, ClosestPointOnLine
				, OnSweepEdgeLocation
				, SweepRatio
			);

			if (bIsIntersection && SweepRatio < OutHitData.SweepRatio)
			{
				OutHitData.bIsHit = true;
				OutHitData.Location = ClosestPointOnLine;
				OutHitData.OnSweepEdgeLocation = OnSweepEdgeLocation;
				OutHitData.SweepRatio = SweepRatio;
				OutHitData.bIsHitOnFirstTriangleSweep = bIsFirstTriangleSweep;
				OutHitData.RopePointIndex = RopePointIndex;
				OutHitData.EdgeIndex = EdgeIndex;
				OutHitData.ShapeIndex = ShapeIndex;
			}
		}
	}
	void SweepRemoveTriangleAgainstShape(
		const FVector& FromCorner
		, const FVector& ToCorner
		, const FVector& SupportCorner
		, const FRopeCollisionShape& Shape
		, const int32 ShapeIndex
		, const TArray<FIntVector2>& IgnoredEdges
		, FHitData& OutHitData
	)
	{
		for (int32 EdgeIndex = 0; EdgeIndex < Shape.Edges.Num(); ++EdgeIndex)
		{
			if (IgnoredEdges.Contains(FIntVector2(ShapeIndex, EdgeIndex)))
			{
				continue;
			}
			const FIntVector2& EdgeVerts = Shape.Edges[EdgeIndex];
			const FVector& EdgeA = Shape.Vertices[EdgeVerts.X];
			const FVector& EdgeB = Shape.Vertices[EdgeVerts.Y];

			FVector ClosestPointOnLine;
			FVector OnSweepEdgeLocation;
			float SweepRatio = MAX_FLT;
			const bool bIsIntersection = GetTriangleLineIntersection(
				FromCorner
				, ToCorner
				, SupportCorner
				, EdgeA
				, EdgeB
				, ClosestPointOnLine
				, OnSweepEdgeLocation
				, SweepRatio
			);
			if (bIsIntersection && SweepRatio < OutHitData.SweepRatio)
			{
				OutHitData.bIsHit = true;
				OutHitData.Location = ClosestPointOnLine;
				OutHitData.OnSweepEdgeLocation = OnSweepEdgeLocation;
				OutHitData.SweepRatio = SweepRatio;
				OutHitData.EdgeIndex = EdgeIndex;
				OutHitData.ShapeIndex = ShapeIndex;
			}
		}
	}

	bool GetTriangleLineIntersection(
		const FVector& FromCorner,
		const FVector& ToCorner,
		const FVector& SupportCorner,
		const FVector& LineA,
		const FVector& LineB,
		FVector& OutLocation,
		FVector& OutOnSweepEdgeLocation,
		float& OutSweepRatio
	)
	{
		// --- Step 1: Möller–Trumbore triangle-line intersection ---
		const FVector Dir = LineB - LineA;
		const FVector Edge1 = ToCorner - FromCorner;
		const FVector Edge2 = SupportCorner - FromCorner;

		const FVector PVec = FVector::CrossProduct(Dir, Edge2);
		const float Det = FVector::DotProduct(Edge1, PVec);
		if (FMath::Abs(Det) < KINDA_SMALL_NUMBER)
		{
			return false; // Line parallel to triangle
		}

		const float InvDet = 1.0f / Det;
		const FVector TVec = LineA - FromCorner;

		const float U = FVector::DotProduct(TVec, PVec) * InvDet;
		if (U < 0.f || U > 1.0f)
		{
			return false;
		}

		const FVector QVec = FVector::CrossProduct(TVec, Edge1);
		const float V = FVector::DotProduct(Dir, QVec) * InvDet;
		if (V < 0.f || U + V > 1.0f)
		{
			return false;
		}

		const float T = FVector::DotProduct(Edge2, QVec) * InvDet;
		if (T < 0.f || T > 1.f)
		{
			return false;
		}

		OutLocation = LineA + Dir * T;

		// --- Step 2: Compute continuation along SupportCorner → OutLocation ---
		const FVector RayOrigin = SupportCorner;
		const FVector RayDir = (OutLocation - SupportCorner).GetSafeNormal();

		const FVector LinePoint = FromCorner;
		const FVector LineDir = ToCorner - FromCorner;

		const FVector CrossDir = FVector::CrossProduct(LineDir, RayDir);
		const float Denom = CrossDir.SizeSquared();

		if (Denom > KINDA_SMALL_NUMBER)
		{
			const float t = FVector::DotProduct(FVector::CrossProduct(RayOrigin - LinePoint, RayDir), CrossDir) / Denom;

			OutSweepRatio = t / LineDir.Size(); // normalized ratio along rope edge
			OutSweepRatio = FMath::Clamp(OutSweepRatio, 0.f, 1.f);

			OutOnSweepEdgeLocation = LinePoint + LineDir * t;
		}
		else
		{
			// Ray and edge are parallel → fallback to nearest edge point
			OutSweepRatio = 0.f;
			OutOnSweepEdgeLocation = FromCorner;
		}

		return true;
	}

#if TAUT_ROPE_DEBUG_DRAWING
	void DebugDrawSweep(
		const UWorld* World
		, const FVector& A
		, const FVector& B
		, const FVector& C
		, bool bIsHit
	)
	{
		TArray<FVector> Vertices = { A, B, C };
		TArray<int32> Indices;
		// First triangle (OriginA, TargetA, OriginB)
		Indices.Add(0); Indices.Add(1); Indices.Add(2);

		DrawDebugMesh(
			World,
			Vertices,
			Indices,
			FColor::Magenta,
			false,	// persistent lines
			5.f   // lifetime
		);
	}
#endif // TAUT_ROPE_DEBUG_DRAWING

}