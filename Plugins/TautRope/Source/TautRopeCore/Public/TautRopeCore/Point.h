#pragma once

#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

namespace TautRope
{
	struct HitData;

	struct TAUTROPE_CORE_API Point
	{
		Point() = default;
		Point(const Vec3& InLocation);
		Point(const HitData& InHitData);

		Vec3 Location;
		int32 ShapeIndex = IndexNone;
		int32 EdgeIndex = IndexNone;
		int32 VertIndex = IndexNone;

		// Stable for the lifetime of the point; assigned by Rope, never reused.
		int32 Id = IndexNone;
	};
}
