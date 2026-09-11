#include "TautRopeCore/Rope.h"

#include "TautRopeCore/Collision.h"
#include "TautRopeCore/Config.h"
#include "TautRopeCore/DebugDraw.h"
#include "TautRopeCore/Movement.h"
#include "TautRopeCore/Pruning.h"
#include "TautRopeCore/Seam.h"
#include "TautRopeCore/VertexHandling.h"
#include "TautRopeCore/Recording.h"

#include <algorithm>

namespace TautRope
{
	void Rope::AppendToNearbyShapes(const std::vector<CollisionShape>& Shapes)
	{
		NearbyShapes.insert(NearbyShapes.end(), Shapes.begin(), Shapes.end());
		WeldSeamsAcrossShapes(NearbyShapes);
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

	void Rope::RestoreState(const std::vector<RecordedPoint>& Points, int32 InNextPointId)
	{
		RopePoints.clear();
		RopePoints.reserve(Points.size());
		for (const RecordedPoint& Recorded : Points)
		{
			Point Restored;
			Restored.Location = Recorded.Location;
			Restored.ShapeIndex = Recorded.ShapeIndex;
			Restored.EdgeIndex = Recorded.EdgeIndex;
			Restored.VertIndex = Recorded.VertIndex;
			Restored.Id = Recorded.Id;
			RopePoints.push_back(Restored);
		}
		NextPointId = InNextPointId;
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
		// Runs of points whose edges fan out from one vertex are straightened as
		// a group: unfold the fan to 2D, draw the straight line between the
		// neighbouring anchors, and place every point in one solve. Solving them
		// one point at a time contracts the run by a neighbour-average step per
		// frame, which is the convergence crawl this replaces. Everything else
		// keeps the per-point solve.
		const std::vector<MovementGroup> MovementGroups = GetMovementGroups(RopePoints, NearbyShapes);
		int32 NextGroupIndex = 0;
		for (int32 i = 1; i < Num(RopePoints) - 1; ++i)
		{
			if (NextGroupIndex < Num(MovementGroups) && MovementGroups[NextGroupIndex].FirstPointIndex == i)
			{
				const MovementGroup& Group = MovementGroups[NextGroupIndex];
				++NextGroupIndex;
				if (SolveFanMovementGroup(RopePoints, RopeTargetLocations, Group, NearbyShapes[Group.ShapeIndex]))
				{
					i = Group.LastPointIndex;
					continue;
				}
				// Degenerate fan: fall through to the per-point solve below.
			}
			Point& PointB = RopePoints[i];
			if (PointB.VertIndex != IndexNone)
			{
				continue;
			}
			// A seam twin - the coincident point a tied cross-shape hit inserts on
			// the other hull's collinear edge - hands the solve its own position
			// back, so the pair never feels the rope's tension. Anchor through a
			// twin: coincident location AND collinear edge. A coincident corner on
			// a DIVERGING edge stays an anchor; skipping those fanned clusters out
			// across the double cone.
			const auto IsRedundantTwin = [&](const int32 NeighbourIndex) -> bool
			{
				const Point& Neighbour = RopePoints[NeighbourIndex];
				if (Neighbour.ShapeIndex == IndexNone || Neighbour.EdgeIndex == IndexNone)
				{
					return false;
				}
				if (static_cast<float>((RopeTargetLocations[NeighbourIndex] - PointB.Location).SizeSquared()) > DistanceToleranceSquared)
				{
					return false;
				}
				const CollisionShape& ShapeB = NearbyShapes[PointB.ShapeIndex];
				const Int2& EdgeB = ShapeB.Edges[PointB.EdgeIndex];
				const CollisionShape& ShapeN = NearbyShapes[Neighbour.ShapeIndex];
				const Int2& EdgeN = ShapeN.Edges[Neighbour.EdgeIndex];
				const Vec3 DirB = (ShapeB.Vertices[EdgeB.Y] - ShapeB.Vertices[EdgeB.X]).GetSafeNormal();
				const Vec3 DirN = (ShapeN.Vertices[EdgeN.Y] - ShapeN.Vertices[EdgeN.X]).GetSafeNormal();
				return Math::Abs(static_cast<float>(Vec3::Dot(DirB, DirN))) > 0.999f;
			};
			int32 AnchorAIndex = i - 1;
			while (AnchorAIndex > 0 && IsRedundantTwin(AnchorAIndex))
			{
				--AnchorAIndex;
			}
			int32 AnchorCIndex = i + 1;
			while (AnchorCIndex < Num(RopePoints) - 1 && IsRedundantTwin(AnchorCIndex))
			{
				++AnchorCIndex;
			}
			const Vec3& LocationA = RopeTargetLocations[AnchorAIndex];
			const Vec3& LocationC = RopeTargetLocations[AnchorCIndex];
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
			bool bAnyInserted = false;
			for (int32 i = Num(SegmentSweepHits) - 1; i >= 0; --i)
			{
				const HitData& Hit = SegmentSweepHits[i];

				// The sweep can reach several edges at once, which is what happens when
				// it crosses the vertex they share. Insert a point for each, ordered
				// along the rope by distance from the preceding point, so the chain
				// stays monotonic. Keeping only the nearest hit here is what let the
				// rope skip an edge and then meet it later as a fresh collision, with
				// the rope line already cutting through the solid.
				// A hit landing on top of either segment endpoint would insert a
				// zero-length segment: no new constraint, only a duplicate point.
				// Sweeps skip their own endpoints' edges but not a neighbour's, so
				// two coincident cross-shape edges otherwise ping-pong - each new
				// segment re-finds the other edge at the same spot, one insertion
				// per collision iteration until the cap.
				const Vec3& SegmentEndA = OriginRopePoints[Hit.RopePointIndex - 1];
				const Vec3& SegmentEndB = OriginRopePoints[Hit.RopePointIndex];
				const auto IsZeroLengthInsert = [&](const Vec3& Location) -> bool
				{
					return static_cast<float>((Location - SegmentEndA).SizeSquared()) <= DistanceToleranceSquared
						|| static_cast<float>((Location - SegmentEndB).SizeSquared()) <= DistanceToleranceSquared;
				};

				std::vector<Point> Group;
				Group.reserve(Hit.TiedHits.size() + 1);
				if (!IsZeroLengthInsert(Hit.Location))
				{
					Group.push_back(Point(Hit));
				}

				for (const TiedHit& Tied : Hit.TiedHits)
				{
					if (IsZeroLengthInsert(Tied.Location))
					{
						continue;
					}
					Point Extra(Hit);
					Extra.Location = Tied.Location;
					Extra.ShapeIndex = Tied.ShapeIndex;
					Extra.EdgeIndex = Tied.EdgeIndex;
					Extra.VertIndex = IndexNone;
					Group.push_back(Extra);
				}
				if (Group.empty())
				{
					continue;
				}
				bAnyInserted = true;

				const Vec3& PrecedingLocation = OriginRopePoints[Hit.RopePointIndex - 1];
				std::stable_sort(
					Group.begin()
					, Group.end()
					, [&PrecedingLocation](const Point& A, const Point& B)
					{
						return (A.Location - PrecedingLocation).SizeSquared()
							< (B.Location - PrecedingLocation).SizeSquared();
					}
				);

				for (int32 g = Num(Group) - 1; g >= 0; --g)
				{
					Point Inserted = Group[g];
					Inserted.Id = NextPointId++;
					RopePoints.insert(RopePoints.begin() + Hit.RopePointIndex, Inserted);
					OriginRopePoints.insert(OriginRopePoints.begin() + Hit.RopePointIndex, Inserted.Location);
					TargetRopePoints.insert(TargetRopePoints.begin() + Hit.RopePointIndex, Inserted.Location);
				}
			}
			// Progress is an insertion, not a hit: a hit whose whole group was
			// filtered as zero-length duplicates would otherwise be re-found and
			// spin this loop to its cap without changing anything.
			bIsAnyNewCollision = bAnyInserted;
			CollisionItr++;

			if (Num(RopePoints) > MostRopePoints)
			{
				MostRopePoints = Num(RopePoints);
			}
			if (Num(RopePoints) >= MaxRopePoints)
			{
				// Stop rather than grind. Without this a runaway does not fail, it
				// makes each frame slower than the last until the process looks hung.
				++RopePointCeilingHits;
				break;
			}
		}

		if (CollisionItr > MostCollisionIterations)
		{
			MostCollisionIterations = CollisionItr;
		}
		if (CollisionItr >= MaxCollisionIterations)
		{
			++CollisionIterationCapHits;
		}
		return false;
	}

	bool Rope::PruningPhase(IDebugDraw* Debug)
	{
		std::vector<bool> PointsToRemove = GetPointsToRemove(RopePoints, NearbyShapes);
		for (int32 i = Num(RopePoints) - 2; i > 0; --i)
		{
			if (PointsToRemove[i])
			{
				// The remove sweep inserts points and, unlike the collision phase,
				// had no share of the rope-point ceiling: the double-cone apex grew
				// the rope to 151,986 points inside two frames, which presents as
				// the editor freezing. Reaching this is a bug, never a heavy frame.
				if (Num(RopePoints) >= MaxRopePoints)
				{
					++RopePointCeilingHits;
					break;
				}
				int32 RemoveSweepIterations = 0;
				SweepRemovePoint(RopePoints, i, NearbyShapes, NextPointId, Debug, &RemoveSweepIterations);
				if (RemoveSweepIterations > MostRemoveSweepIterations)
				{
					MostRemoveSweepIterations = RemoveSweepIterations;
				}
				if (RemoveSweepIterations >= MaxRemoveSweepIterations)
				{
					++RemoveSweepIterationCapHits;
				}
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
