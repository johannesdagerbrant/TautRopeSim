#include "TautRopeCore/Pruning.h"

#include "TautRopeCore/Config.h"

namespace TautRope
{
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
