#include "TautRopeCore/Movement.h"

#include "TautRopeCore/Config.h"
#include "TautRopeCore/Point.h"

#include <cmath>

namespace TautRope
{
	MovementGroup::MovementGroup(
		const int32 InShapeIndex
		, const int32 InVertIndex
		, const int32 InFirstPointIndex
	)
		: ShapeIndex(InShapeIndex)
		, VertIndex(InVertIndex)
		, FirstPointIndex(InFirstPointIndex)
	{}

	std::vector<MovementGroup> GetMovementGroups(
		const std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	)
	{
		std::vector<MovementGroup> MovementGroups;
		const int32 NumPoints = Num(RopePoints);

		int32 RunStart = IndexNone;
		std::vector<int32> SharedVertCandidates;

		const auto CloseRun = [&](const int32 RunEnd)
		{
			if (RunStart != IndexNone && RunEnd - RunStart + 1 >= 2 && !SharedVertCandidates.empty())
			{
				MovementGroup Group(RopePoints[RunStart].ShapeIndex, SharedVertCandidates[0], RunStart);
				Group.LastPointIndex = RunEnd;
				for (int32 j = RunStart; j <= RunEnd; ++j)
				{
					Group.EdgeIndices.push_back(RopePoints[j].EdgeIndex);
				}
				MovementGroups.push_back(Group);
			}
			RunStart = IndexNone;
			SharedVertCandidates.clear();
		};

		for (int32 i = 1; i < NumPoints - 1; ++i)
		{
			const Point& RopePoint = RopePoints[i];
			// A point already sitting on a vertex is in the crossing state the
			// movement phase leaves alone, so it can anchor a fan but not ride one.
			if (RopePoint.ShapeIndex == IndexNone
				|| RopePoint.EdgeIndex == IndexNone
				|| RopePoint.VertIndex != IndexNone)
			{
				CloseRun(i - 1);
				continue;
			}
			const Int2& Edge = NearbyShapes[RopePoint.ShapeIndex].Edges[RopePoint.EdgeIndex];
			if (RunStart != IndexNone)
			{
				bool bIsNewEdge = RopePoints[RunStart].ShapeIndex == RopePoint.ShapeIndex;
				for (int32 j = RunStart; bIsNewEdge && j < i; ++j)
				{
					bIsNewEdge = RopePoints[j].EdgeIndex != RopePoint.EdgeIndex;
				}
				std::vector<int32> Narrowed;
				if (bIsNewEdge)
				{
					for (const int32 Candidate : SharedVertCandidates)
					{
						if (Candidate == Edge.X || Candidate == Edge.Y)
						{
							Narrowed.push_back(Candidate);
						}
					}
				}
				if (!Narrowed.empty())
				{
					SharedVertCandidates = Narrowed;
					continue;
				}
				CloseRun(i - 1);
			}
			RunStart = i;
			SharedVertCandidates = { Edge.X, Edge.Y };
		}
		CloseRun(NumPoints - 2);

		return MovementGroups;
	}

	bool SolveFanMovementGroup(
		std::vector<Point>& RopePoints
		, std::vector<Vec3>& RopeTargetLocations
		, const MovementGroup& Group
		, const CollisionShape& Shape
	)
	{
		const int32 First = Group.FirstPointIndex;
		const int32 Last = Group.LastPointIndex;
		const int32 NumEdges = Last - First + 1;
		const Vec3& FanVert = Shape.Vertices[Group.VertIndex];

		const Vec3 ToAnchorA = RopeTargetLocations[First - 1] - FanVert;
		const Vec3 ToAnchorB = RopeTargetLocations[Last + 1] - FanVert;
		const float RadiusA = static_cast<float>(ToAnchorA.Size());
		const float RadiusB = static_cast<float>(ToAnchorB.Size());
		if (RadiusA <= KindaSmallNumber || RadiusB <= KindaSmallNumber)
		{
			return false;
		}

		std::vector<Vec3> EdgeDirs(NumEdges);
		std::vector<float> EdgeLengths(NumEdges);
		for (int32 e = 0; e < NumEdges; ++e)
		{
			const Int2& Edge = Shape.Edges[Group.EdgeIndices[e]];
			const int32 OuterVertIndex = Edge.X == Group.VertIndex ? Edge.Y : Edge.X;
			const Vec3 AlongEdge = Shape.Vertices[OuterVertIndex] - FanVert;
			const float EdgeLength = static_cast<float>(AlongEdge.Size());
			if (EdgeLength <= KindaSmallNumber)
			{
				return false;
			}
			EdgeDirs[e] = AlongEdge / EdgeLength;
			EdgeLengths[e] = EdgeLength;
		}

		// Unfold: cumulative angles around the shared vertex, anchor A on the
		// +X axis. Angles at the vertex and distances from it are preserved, so
		// the straight 2D segment is the geodesic across the fan.
		const auto AngleBetween = [](const Vec3& U, const Vec3& W) -> double
		{
			return std::atan2(Vec3::Cross(U, W).Size(), Vec3::Dot(U, W));
		};
		std::vector<double> EdgeAngles(NumEdges);
		EdgeAngles[0] = AngleBetween(ToAnchorA, EdgeDirs[0]);
		for (int32 e = 1; e < NumEdges; ++e)
		{
			EdgeAngles[e] = EdgeAngles[e - 1] + AngleBetween(EdgeDirs[e - 1], EdgeDirs[e]);
		}
		const double AngleB = EdgeAngles[NumEdges - 1] + AngleBetween(EdgeDirs[NumEdges - 1], ToAnchorB);

		const double AnchorAX = static_cast<double>(RadiusA);
		const double SegX = static_cast<double>(RadiusB) * std::cos(AngleB) - AnchorAX;
		const double SegY = static_cast<double>(RadiusB) * std::sin(AngleB);

		// Used to get a normalized vector inbetween two unit length orthogonal vectors.
		constexpr float INV_SQRT2 = 0.70710678f;

		// Squared distance from a 2D point to the anchor segment, for the miss
		// rule below.
		const auto DistSquaredToSegment = [&](const double PX, const double PY) -> double
		{
			const double ToPX = PX - AnchorAX;
			const double ToPY = PY;
			const double SegLenSquared = SegX * SegX + SegY * SegY;
			const double T = SegLenSquared > 0.0
				? Math::Clamp((ToPX * SegX + ToPY * SegY) / SegLenSquared, 0.0, 1.0)
				: 0.0;
			const double DX = ToPX - T * SegX;
			const double DY = ToPY - T * SegY;
			return DX * DX + DY * DY;
		};

		for (int32 e = 0; e < NumEdges; ++e)
		{
			const double RayX = std::cos(EdgeAngles[e]);
			const double RayY = std::sin(EdgeAngles[e]);

			// Segment AnchorA + t * Seg against the ray s * Ray out of the vertex.
			// When the segment no longer crosses the edge, the point belongs on
			// whichever of the edge's two endpoints sits closest to the segment:
			// the shared vertex when the line has passed inside the fan (the rope
			// pinching onto the vertex), the outer end when it has passed beyond.
			float DistFromFanVert = 0.f;
			bool bSegmentCrossesEdge = false;
			const double Denominator = RayX * SegY - RayY * SegX;
			if (Math::Abs(Denominator) > static_cast<double>(SmallNumber))
			{
				const double AlongSegment = AnchorAX * RayY / Denominator;
				const double AlongRay = AnchorAX * SegY / Denominator;
				if (AlongSegment >= 0.0 && AlongSegment <= 1.0 && AlongRay > 0.0)
				{
					DistFromFanVert = static_cast<float>(AlongRay);
					bSegmentCrossesEdge = true;
				}
			}
			if (!bSegmentCrossesEdge)
			{
				const double OuterX = static_cast<double>(EdgeLengths[e]) * RayX;
				const double OuterY = static_cast<double>(EdgeLengths[e]) * RayY;
				DistFromFanVert = DistSquaredToSegment(0.0, 0.0) <= DistSquaredToSegment(OuterX, OuterY)
					? 0.f
					: EdgeLengths[e];
			}

			const int32 PointIndex = First + e;
			Point& RopePoint = RopePoints[PointIndex];
			const int32 EdgeIndex = Group.EdgeIndices[e];
			const Int2& Edge = Shape.Edges[EdgeIndex];
			const bool bFanVertIsEdgeX = Edge.X == Group.VertIndex;
			const float EdgeLength = EdgeLengths[e];
			const float DistAlongEdge = bFanVertIsEdgeX ? DistFromFanVert : EdgeLength - DistFromFanVert;

			// Same crossing rules and offsets as the per-point solve in
			// Rope::MovementPhase, with the distance measured from Edge.X.
			const bool bIsEdgeCornerAtVertexA = Shape.IsCornerVertex(Edge.X);
			const bool bIsEdgeCornerAtVertexB = Shape.IsCornerVertex(Edge.Y);
			const Vec3& EdgeVertA = Shape.Vertices[Edge.X];
			const Vec3& EdgeVertB = Shape.Vertices[Edge.Y];
			if (!bIsEdgeCornerAtVertexA && DistAlongEdge < DistanceTolerance)
			{
				RopePoint.VertIndex = Edge.X;
				const Quat& EdgeRotation = Shape.EdgeRotations[EdgeIndex];
				RopeTargetLocations[PointIndex] = EdgeVertA + (EdgeRotation.GetUpVector() - EdgeRotation.GetForwardVector()) * INV_SQRT2 * VertexCrossingOffset;
			}
			else if (!bIsEdgeCornerAtVertexB && DistAlongEdge > EdgeLength - DistanceTolerance)
			{
				RopePoint.VertIndex = Edge.Y;
				const Quat& EdgeRotation = Shape.EdgeRotations[EdgeIndex];
				RopeTargetLocations[PointIndex] = EdgeVertB + (EdgeRotation.GetUpVector() + EdgeRotation.GetForwardVector()) * INV_SQRT2 * VertexCrossingOffset;
			}
			else
			{
				RopePoint.VertIndex = IndexNone;
				const Vec3 EdgeDirFromX = bFanVertIsEdgeX ? EdgeDirs[e] : -EdgeDirs[e];
				RopeTargetLocations[PointIndex] = EdgeVertA + EdgeDirFromX * Math::Clamp(DistAlongEdge, DistanceTolerance, EdgeLength - DistanceTolerance);
			}
		}
		return true;
	}

	std::vector<int32> GetCandidateVerts(const Point& RopePoint, const CollisionShape& Shape)
	{
		if (RopePoint.VertIndex != IndexNone)
		{
			return { RopePoint.VertIndex };
		}
		else
		{
			const Int2& Edge = Shape.Edges[RopePoint.EdgeIndex];
			return { Edge.X, Edge.Y };
		}
	}

	Vec3 FindMinDistancePointBetweenABOnLineXY(
		const Vec3& A
		, const Vec3& B
		, const Vec3& LineX
		, const Vec3& LineY
		, float& OutDistAlongEdge
		, float& OutEdgeLength
	)
	{
		const Vec3 Edge = LineY - LineX;
		OutEdgeLength = static_cast<float>(Edge.Size());
		TAUTROPE_ENSURE(OutEdgeLength > KindaSmallNumber);

		const Vec3 EdgeDir = Edge / OutEdgeLength;

		// Projections of A and B onto the edge axis
		const float Alpha = static_cast<float>(Vec3::Dot(A - LineX, EdgeDir));
		const float Beta = static_cast<float>(Vec3::Dot(B - LineX, EdgeDir));

		// Perpendicular offsets
		const float rhoA = static_cast<float>(Vec3::Cross(A - LineX, EdgeDir).Size());
		const float rhoB = static_cast<float>(Vec3::Cross(B - LineX, EdgeDir).Size());

		if (rhoA < KindaSmallNumber && rhoB < KindaSmallNumber)
		{
			// Both lie essentially on the line -> midpoint
			OutDistAlongEdge = 0.5f * (Alpha + Beta);
		}
		else if (rhoA < KindaSmallNumber)
		{
			OutDistAlongEdge = Alpha;
		}
		else if (rhoB < KindaSmallNumber)
		{
			OutDistAlongEdge = Beta;
		}
		else
		{
			// Weighted average
			const float r = rhoA / (rhoA + rhoB);
			OutDistAlongEdge = (1.0f - r) * Alpha + r * Beta;
		}
		return LineX + EdgeDir * Math::Clamp(OutDistAlongEdge, DistanceTolerance, OutEdgeLength - DistanceTolerance);
	}
}
