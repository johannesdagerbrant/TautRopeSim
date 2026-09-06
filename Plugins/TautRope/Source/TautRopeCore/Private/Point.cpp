#include "TautRopeCore/Point.h"
#include "TautRopeCore/Collision.h"

namespace TautRope
{
	Point::Point(const Vec3& InLocation)
		: Location(InLocation)
	{}

	Point::Point(const HitData& InHitData)
		: Location(InHitData.Location)
		, ShapeIndex(InHitData.ShapeIndex)
		, EdgeIndex(InHitData.EdgeIndex)
	{}
}
