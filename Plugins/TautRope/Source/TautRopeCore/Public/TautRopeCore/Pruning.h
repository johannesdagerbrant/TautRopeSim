#pragma once

#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

namespace TautRope
{
	TAUTROPE_CORE_API bool IsRopeWrappingEdge(
		const Vec3& PointLocationA
		, const Vec3& PointLocationB
		, const Vec3& PointLocationC
		, const Quat& EdgeRotation
	);
}
