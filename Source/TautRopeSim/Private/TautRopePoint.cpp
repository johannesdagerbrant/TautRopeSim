#include "TautRopePoint.h"
#include "TautRopeHelpersCollision.h"

namespace TautRope
{
	FPoint::FPoint(const FVector& InLocation) : Location(InLocation) {};
	FPoint::FPoint(const FHitData& HitData)
		: Location(HitData.Location)
		, ShapeIndex(HitData.ShapeIndex)
		, EdgeIndex(HitData.EdgeIndex)
	{};
}