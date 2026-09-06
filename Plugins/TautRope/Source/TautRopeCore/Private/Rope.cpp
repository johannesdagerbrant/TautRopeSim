#include "TautRopeCore/Rope.h"

#include "TautRopeCore/Collision.h"
#include "TautRopeCore/Config.h"
#include "TautRopeCore/DebugDraw.h"
#include "TautRopeCore/Movement.h"
#include "TautRopeCore/Pruning.h"
#include "TautRopeCore/VertexHandling.h"
#include "TautRopeCore/Recording.h"

#include <algorithm>

namespace TautRope
{
	void Rope::AppendToNearbyShapes(const std::vector<CollisionShape>& Shapes)
	{
		NearbyShapes.insert(NearbyShapes.end(), Shapes.begin(), Shapes.end());
	}

	std::vector<Vec3> Rope::GetRopePoints() const
	{
		std::vector<Vec3> Result;
		Result.resize(RopePoints.size());
		for (int32 i = 0; i < Num(RopePoints); ++i)
		{
			Result[i] = RopePoints[i].Location;
		}
		return Result;
	}

	void Rope::UpdateRope(
		const Vec3& StartLocation
		, const Vec3& EndLocation
		, const float MaxLength
		, IDebugDraw* Debug
		, FrameCapture* Capture
	)
	{
		if (Capture != nullptr)
		{
			Capture->Clear();
		}

		if (Num(RopePoints) < 2)
		{
			RopePoints.clear();
			Point Start(StartLocation);
			Start.Id = NextPointId++;
			Point End(EndLocation);
			End.Id = NextPointId++;
			RopePoints.push_back(Start);
			RopePoints.push_back(End);
			if (Capture != nullptr)
			{
				CapturePoints(RopePoints, Capture->AfterMovement);
				CapturePoints(RopePoints, Capture->AfterCollision);
				CapturePoints(RopePoints, Capture->AfterPruning);
			}
			return;
		}
		// Move phase
		std::vector<Vec3> TargetRopePoints = MovementPhase(StartLocation, EndLocation, MaxLength);
		if (Capture != nullptr)
		{
			CapturePoints(RopePoints, Capture->AfterMovement);
		}
		// Collision phase
		const bool bHadCollision = CollisionPhase(TargetRopePoints, Debug);
		if (Capture != nullptr)
		{
			CapturePoints(RopePoints, Capture->AfterCollision);
		}
		// Pruning phase
		const bool bWasPruned = PruningPhase(Debug);
		if (Capture != nullptr)
		{
			CapturePoints(RopePoints, Capture->AfterPruning);
		}

		(void)bHadCollision;
		(void)bWasPruned;
	}

	void Rope::RestoreState(const std::vector<RecordedPoint>& Points)
	{
		RopePoints.clear();
		RopePoints.reserve(Points.size());
		int32 MaxId = IndexNone;
		for (const RecordedPoint& Recorded : Points)
		{
			Point Restored;
			Restored.Location = Recorded.Location;
			Restored.ShapeIndex = Recorded.ShapeIndex;
			Restored.EdgeIndex = Recorded.EdgeIndex;
			Restored.VertIndex = Recorded.VertIndex;
			Restored.Id = Recorded.Id;
			RopePoints.push_back(Restored);
			if (Recorded.Id > MaxId)
			{
				MaxId = Recorded.Id;
			}
		}
		NextPointId = MaxId + 1;
	}

	std::vector<Vec3> Rope::MovementPhase(
		const Vec3& StartLocation
		, const Vec3& EndLocation
		, const float MaxLength
	)
	{
		TAUTROPE_ENSURE(Num(RopePoints) >= 2);

		std::vector<Vec3> RopeTargetLocations;

		RopeTargetLocations.resize(RopePoints.size());
		RopeTargetLocations[0] = StartLocation;

		float RopeDistanceToSecondLastPoint = 0.f;
		for (int32 Index = 0; Index < Num(RopePoints) - 2; ++Index)
		{
			RopeDistanceToSecondLastPoint += static_cast<float>(Vec3::Dist(
				RopePoints[Index].Location
				, RopePoints[Index + 1].Location
			));
		}
		const float AvaliableDistanceTowardsEndPoint = MaxLength - RopeDistanceToSecondLastPoint;
		const Vec3 SecondLastPointLocation = RopePoints[Num(RopePoints) - 2].Location;
		const float DistanceToEndPoint = static_cast<float>(Vec3::Dist(SecondLastPointLocation, EndLocation));
		const float ToEndPointAlpha = Math::Clamp(AvaliableDistanceTowardsEndPoint / DistanceToEndPoint, 0.f, 1.f);
		const Vec3 LastPointLocation = Math::Lerp(SecondLastPointLocation, EndLocation, ToEndPointAlpha);
		RopeTargetLocations.back() = LastPointLocation;
		if (Num(RopeTargetLocations) == 2)
		{
			return RopeTargetLocations;
		}
		for (int32 i = 1; i < Num(RopePoints) - 1; ++i)
		{
			RopeTargetLocations[i] = RopePoints[i].Location;
		}
		// TODO: Grouping of rope points that belong to the same vertex fan of edges so we can draw a straight line across multiple edges in 2d space,
		for (int32 i = 1; i < Num(RopePoints) - 1; ++i)
		{
			Point& PointB = RopePoints[i];
			if (PointB.VertIndex != IndexNone)
			{
				continue;
			}
			const Vec3& LocationA = RopeTargetLocations[i - 1];
			const Vec3& LocationC = RopeTargetLocations[i + 1];
			const CollisionShape& Shape = NearbyShapes[PointB.ShapeIndex];
			const Int2& Edge = Shape.Edges[PointB.EdgeIndex];
			const bool bIsEdgeCornerAtVertexA = NearbyShapes[PointB.ShapeIndex].IsCornerVertex(Edge.X);
			const bool bIsEdgeCornerAtVertexB = NearbyShapes[PointB.ShapeIndex].IsCornerVertex(Edge.Y);
			const Vec3& EdgeVertA = Shape.Vertices[Edge.X];
			const Vec3& EdgeVertB = Shape.Vertices[Edge.Y];
			float OutDistAlongEdge = 0.f;
			float OutEdgeLength = 0.f;
			const Vec3 RopeTargetLocation = FindMinDistancePointBetweenABOnLineXY(LocationA, LocationC, EdgeVertA, EdgeVertB, OutDistAlongEdge, OutEdgeLength);

			// Used to get a normalized vector inbetween two unit length orthogonal vectors.
			constexpr float INV_SQRT2 = 0.70710678f;

			if (!bIsEdgeCornerAtVertexA && OutDistAlongEdge < DistanceTolerance)
			{
				PointB.VertIndex = Edge.X;
				const Quat& EdgeRotation = Shape.EdgeRotations[PointB.EdgeIndex];
				RopeTargetLocations[i] = EdgeVertA + (EdgeRotation.GetUpVector() - EdgeRotation.GetForwardVector()) * INV_SQRT2 * VertexCrossingOffset;
			}
			else if (!bIsEdgeCornerAtVertexB && OutDistAlongEdge > OutEdgeLength - DistanceTolerance)
			{
				PointB.VertIndex = Edge.Y;
				const Quat& EdgeRotation = Shape.EdgeRotations[PointB.EdgeIndex];
				RopeTargetLocations[i] = EdgeVertB + (EdgeRotation.GetUpVector() + EdgeRotation.GetForwardVector()) * INV_SQRT2 * VertexCrossingOffset;
			}
			else
			{
				PointB.VertIndex = IndexNone;
				RopeTargetLocations[i] = RopeTargetLocation;
			}
		}
		return RopeTargetLocations;
	}

	bool Rope::CollisionPhase(std::vector<Vec3>& TargetRopePoints, IDebugDraw* Debug)
	{
		std::vector<Vec3> OriginRopePoints;
		OriginRopePoints.resize(RopePoints.size());
		for (int32 i = 0; i < Num(RopePoints); ++i)
		{
			OriginRopePoints[i] = RopePoints[i].Location;
		}

		int32 CollisionItr = 0;
		bool bIsAnyNewCollision = true;
		while (bIsAnyNewCollision && CollisionItr < MaxCollisionIterations)
		{
			std::vector<HitData> SegmentSweepHits;
			for (int32 i = 0; i < Num(RopePoints) - 1; ++i)
			{
				Point& SegmentPointA = RopePoints[i];
				Point& SegmentPointB = RopePoints[i + 1];
				const Vec3& OriginLocationA = OriginRopePoints[i];
				const Vec3& OriginLocationB = OriginRopePoints[i + 1];
				const Vec3& TargetLocationA = TargetRopePoints[i];
				const Vec3& TargetLocationB = TargetRopePoints[i + 1];

				HitData Hit;
				SweepSegmentThroughShapes(
					Hit
					, SegmentPointA
					, SegmentPointB
					, OriginLocationA
					, OriginLocationB
					, TargetLocationA
					, TargetLocationB
					, NearbyShapes
					, i + 1
					, Debug
				);
				if (Hit.bIsHit)
				{
					if (Hit.bIsHitOnFirstTriangleSweep)
					{
						SegmentPointA.VertIndex = IndexNone;
					}
					else
					{
						SegmentPointB.VertIndex = IndexNone;
					}
					SegmentSweepHits.push_back(Hit);
				}
			}
			for (int32 i = Num(SegmentSweepHits) - 1; i >= 0; --i)
			{
				const HitData& Hit = SegmentSweepHits[i];
				Point Inserted(Hit);
				Inserted.Id = NextPointId++;
				RopePoints.insert(RopePoints.begin() + Hit.RopePointIndex, Inserted);
				OriginRopePoints.insert(OriginRopePoints.begin() + Hit.RopePointIndex, Hit.Location);
				TargetRopePoints.insert(TargetRopePoints.begin() + Hit.RopePointIndex, Hit.Location);
			}
			bIsAnyNewCollision = !SegmentSweepHits.empty();
			CollisionItr++;
		}
		return false;
	}

	bool Rope::PruningPhase(IDebugDraw* Debug)
	{
		std::vector<bool> PointsToRemove = GetAdjacentPointsOnSameVertexCone(RopePoints, NearbyShapes);
		for (int32 i = 1; i < Num(RopePoints) - 1; ++i)
		{
			if (PointsToRemove[i])
			{
				continue;
			}
			const Point& LastPoint = RopePoints[i - 1];
			const Point& CurrentPoint = RopePoints[i];
			const Point& NextPoint = RopePoints[i + 1];
			const CollisionShape& Shape = NearbyShapes[CurrentPoint.ShapeIndex];
			if (CurrentPoint.ShapeIndex == LastPoint.ShapeIndex && CurrentPoint.EdgeIndex == LastPoint.EdgeIndex)
			{
				PointsToRemove[i] = true;
				continue;
			}
			const Quat& EdgeRotation = Shape.EdgeRotations[CurrentPoint.EdgeIndex];
			const bool bIsRopeWrappingEdge = IsRopeWrappingEdge(
				LastPoint.Location
				, CurrentPoint.Location
				, NextPoint.Location
				, EdgeRotation
			);
			if (!bIsRopeWrappingEdge)
			{
				PointsToRemove[i] = true;
			}
		}
		for (int32 i = Num(RopePoints) - 2; i > 0; --i)
		{
			if (PointsToRemove[i])
			{
				SweepRemovePoint(RopePoints, i, NearbyShapes, NextPointId, Debug);
			}
		}
		return std::find(PointsToRemove.begin(), PointsToRemove.end(), true) != PointsToRemove.end();
	}

	void Rope::DrawDebugRope(IDebugDraw& Debug) const
	{
		// Draw rope segments
		for (int32 i = 0; i < Num(RopePoints); ++i)
		{
			Vec3 UpOffsetA;
			Vec3 UpOffsetB;
			if (RopePoints[i].ShapeIndex != IndexNone)
			{
				const CollisionShape& ShapeA = NearbyShapes[RopePoints[i].ShapeIndex];
				UpOffsetA = ShapeA.EdgeRotations[RopePoints[i].EdgeIndex].GetUpVector() * DistanceTolerance;
			}

			if (i + 1 < Num(RopePoints))
			{
				if (RopePoints[i + 1].ShapeIndex != IndexNone)
				{
					const CollisionShape& ShapeB = NearbyShapes[RopePoints[i + 1].ShapeIndex];
					UpOffsetB = ShapeB.EdgeRotations[RopePoints[i + 1].EdgeIndex].GetUpVector() * DistanceTolerance;
				}
				Debug.Line(
					RopePoints[i].Location + UpOffsetA
					, RopePoints[i + 1].Location + UpOffsetB
					, ColorBlack
				);
			}

			Debug.Sphere(RopePoints[i].Location + UpOffsetA, 1.0, 4, ColorBlack);
		}
	}

	void Rope::DrawDebugRopeTouchedShapeEdges(IDebugDraw& Debug) const
	{
		for (int32 i = 0; i < Num(RopePoints); ++i)
		{
			if (RopePoints[i].ShapeIndex != IndexNone)
			{
				const CollisionShape& Shape = NearbyShapes[RopePoints[i].ShapeIndex];
				const int32 EdgeIndex = RopePoints[i].EdgeIndex;
				const int32 EdgeVertIndexA = Shape.Edges[EdgeIndex].X;
				const int32 EdgeVertIndexB = Shape.Edges[EdgeIndex].Y;
				const Vec3 EdgeVertA = Shape.Vertices[EdgeVertIndexA];
				const Vec3 EdgeVertB = Shape.Vertices[EdgeVertIndexB];
				const Vec3 UpOffset = Shape.EdgeRotations[EdgeIndex].GetUpVector() * DistanceTolerance;
				Debug.Line(EdgeVertA + UpOffset, EdgeVertB + UpOffset, ColorBlue);
			}
		}
	}
}
