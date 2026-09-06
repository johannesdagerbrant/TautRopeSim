#include "TautRopeCore/Movement.h"

#include "TautRopeCore/Config.h"
#include "TautRopeCore/Point.h"

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
		if (NumPoints < 3)
		{
			return MovementGroups;
		}

		MovementGroups.push_back(
			MovementGroup(RopePoints[1].ShapeIndex, RopePoints[1].VertIndex, 0)
		);

		for (int32 i = 1; i < NumPoints - 1; i++)
		{
			const Point& RopePoint = RopePoints[i];
			MovementGroup& CurrentGroup = MovementGroups.back();
			CurrentGroup.LastPointIndex = i;

			const CollisionShape& Shape = NearbyShapes[RopePoint.ShapeIndex];
			const std::vector<int32> CandidateVerts = GetCandidateVerts(RopePoint, Shape);
			const int32 GroupVertIndex = CurrentGroup.VertIndex;

			const bool bBelongsInLastVertexGroup =
				CurrentGroup.ShapeIndex == RopePoint.ShapeIndex &&
				Contains(CandidateVerts, GroupVertIndex) &&
				!Contains(CurrentGroup.EdgeIndices, RopePoint.EdgeIndex);

			if (bBelongsInLastVertexGroup)
			{
				CurrentGroup.EdgeIndices.push_back(RopePoint.EdgeIndex);
			}
			else
			{
				MovementGroups.push_back(
					MovementGroup(RopePoint.ShapeIndex, CandidateVerts[0], i)
				);
			}
		}

		if (!MovementGroups.empty())
		{
			MovementGroups[0].FirstPointIndex = 0;
			MovementGroups.back().LastPointIndex = NumPoints - 1;
		}

		return MovementGroups;
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
