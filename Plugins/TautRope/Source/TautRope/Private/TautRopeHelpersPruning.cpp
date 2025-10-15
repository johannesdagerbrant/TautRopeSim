
#include "TautRopeHelpersPruning.h"
#include "TautRopeConfig.h"

namespace TautRope
{
	bool IsRopeWrappingEdge(
		const FVector& PointLocationA
		, const FVector& PointLocationB
		, const FVector& PointLocationC
		, const FQuat& EdgeRotation
	)
	{
		const FVector PlaneForward = EdgeRotation.GetForwardVector();   // local X
		const FVector PlaneDown = -EdgeRotation.GetUpVector();       // local -Z
		const FVector PlaneNormal = FVector::CrossProduct(PlaneForward, PlaneDown).GetSafeNormal();

		const float DistA = FVector::DotProduct(PointLocationA - PointLocationB, PlaneNormal);
		const float DistC = FVector::DotProduct(PointLocationC - PointLocationB, PlaneNormal);

		// If both points are clearly on the same side, not wrapping
		if (DistA * DistC > TAUT_ROPE_DISTANCE_TOLERANCE)
		{
			return false;
		}

		// --------------------------------------------------------------------
		// 4. Find intersection of AC with the plane
		// --------------------------------------------------------------------
		const FVector AC = PointLocationC - PointLocationA;
		const float Denom = FVector::DotProduct(PlaneNormal, AC);

		// If AC is nearly parallel to plane → treat as wrapping (don’t prune)
		if (FMath::Abs(Denom) < TAUT_ROPE_DISTANCE_TOLERANCE)
		{
			return true;
		}

		const float t = -DistA / Denom;
		if (t < -TAUT_ROPE_DISTANCE_TOLERANCE || t > 1.f + TAUT_ROPE_DISTANCE_TOLERANCE)
		{
			return false; // intersection lies outside segment
		}

		const FVector IntersectionPoint = PointLocationA + t * AC;

		// --------------------------------------------------------------------
		// 5. Measure relative "down" direction to check wrapping
		// --------------------------------------------------------------------
		const float AlongDown = FVector::DotProduct(IntersectionPoint - PointLocationB, PlaneDown);

		// If rope passes behind or through the edge plane → not wrapping
		return AlongDown > -TAUT_ROPE_DISTANCE_TOLERANCE;
	}
}