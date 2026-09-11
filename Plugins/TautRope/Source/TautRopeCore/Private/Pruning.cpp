#include "TautRopeCore/Pruning.h"

#include "TautRopeCore/Config.h"
#include "TautRopeCore/Point.h"
#include "TautRopeCore/VertexHandling.h"

namespace TautRope
{
	std::vector<bool> GetPointsToRemove(
		const std::vector<Point>& RopePoints
		, const std::vector<CollisionShape>& NearbyShapes
	)
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
			const CollisionShape& Shape = NearbyShapes[CurrentPoint.ShapeIndex];
			if (CurrentPoint.ShapeIndex == LastPoint.ShapeIndex && CurrentPoint.EdgeIndex == LastPoint.EdgeIndex)
			{
				PointsToRemove[i] = true;
				continue;
			}
			// A wrap verdict needs a baseline that carries direction. A neighbour
			// sitting on top of the judged point - the cross-shape twin a seam
			// crossing inserts - gives it a zero-length one, and the degenerate
			// branch answers "wrapping" forever, which is the seam glue. Judge
			// against the nearest neighbours at a distinct location instead.
			int32 PrevIndex = i - 1;
			while (PrevIndex > 0
				&& static_cast<float>((RopePoints[PrevIndex].Location - CurrentPoint.Location).SizeSquared()) <= DistanceToleranceSquared)
			{
				--PrevIndex;
			}
			int32 NextIndex = i + 1;
			while (NextIndex < Num(RopePoints) - 1
				&& static_cast<float>((RopePoints[NextIndex].Location - CurrentPoint.Location).SizeSquared()) <= DistanceToleranceSquared)
			{
				++NextIndex;
			}
			// A wrap verdict against a neighbour that is being removed in this
			// same pass judges a rope that never exists, and the verdict flips
			// with the neighbour (Pruning_WrapVerdictDependsOnWhichNeighbourSurvives).
			// Defer it: next frame the point is judged against the survivors.
			if (PointsToRemove[PrevIndex] || PointsToRemove[NextIndex])
			{
				continue;
			}
			const Quat& EdgeRotation = Shape.EdgeRotations[CurrentPoint.EdgeIndex];
			const bool bIsRopeWrappingEdge = IsRopeWrappingEdge(
				RopePoints[PrevIndex].Location
				, CurrentPoint.Location
				, RopePoints[NextIndex].Location
				, EdgeRotation
			);
			if (!bIsRopeWrappingEdge)
			{
				PointsToRemove[i] = true;
			}
		}
		return PointsToRemove;
	}

	bool IsRopeWrappingEdge(
		const Vec3& PointLocationA
		, const Vec3& PointLocationB
		, const Vec3& PointLocationC
		, const Quat& EdgeRotation
	)
	{
		const Vec3 PlaneForward = EdgeRotation.GetForwardVector();   // local X
		const Vec3 PlaneDown = -EdgeRotation.GetUpVector();          // local -Z
		const Vec3 PlaneNormal = Vec3::Cross(PlaneForward, PlaneDown).GetSafeNormal();

		const float DistA = static_cast<float>(Vec3::Dot(PointLocationA - PointLocationB, PlaneNormal));
		const float DistC = static_cast<float>(Vec3::Dot(PointLocationC - PointLocationB, PlaneNormal));

		// If both points are clearly on the same side, not wrapping
		if (DistA * DistC > DistanceTolerance)
		{
			return false;
		}

		// Find intersection of AC with the plane
		const Vec3 AC = PointLocationC - PointLocationA;
		const float Denom = static_cast<float>(Vec3::Dot(PlaneNormal, AC));

		// If AC is nearly parallel to plane, treat as wrapping and do not prune
		if (Math::Abs(Denom) < DistanceTolerance)
		{
			return true;
		}

		const float t = -DistA / Denom;
		if (t < -DistanceTolerance || t > 1.f + DistanceTolerance)
		{
			return false; // intersection lies outside segment
		}

		const Vec3 IntersectionPoint = PointLocationA + t * AC;

		// Measure relative "down" direction to check wrapping
		const float AlongDown = static_cast<float>(Vec3::Dot(IntersectionPoint - PointLocationB, PlaneDown));

		// If rope passes behind or through the edge plane, not wrapping
		return AlongDown > -DistanceTolerance;
	}
}
