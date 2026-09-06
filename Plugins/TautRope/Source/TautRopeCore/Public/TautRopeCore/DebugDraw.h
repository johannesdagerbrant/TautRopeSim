#pragma once

#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

namespace TautRope
{
	// 0xAARRGGBB, so core needs no engine colour type.
	inline constexpr unsigned int ColorBlack = 0xFF000000u;
	inline constexpr unsigned int ColorBlue = 0xFF0000FFu;
	inline constexpr unsigned int ColorMagenta = 0xFFFF00FFu;

	// Implemented by the UE glue. The simulation only emits primitives; whether
	// anything is drawn, and in what world, is not core's concern. The replay
	// program passes nullptr.
	class IDebugDraw
	{
	public:
		virtual ~IDebugDraw() = default;

		virtual void Line(const Vec3& A, const Vec3& B, unsigned int Color) = 0;
		virtual void Sphere(const Vec3& Center, double Radius, int32 Segments, unsigned int Color) = 0;
		virtual void Triangle(const Vec3& A, const Vec3& B, const Vec3& C, unsigned int Color) = 0;

		// Queried deep inside the sweep loops, where the glue's CVars are not
		// reachable.
		virtual bool WantsSegmentSweep() const = 0;
		virtual bool WantsRemoveSweep() const = 0;
	};
}
