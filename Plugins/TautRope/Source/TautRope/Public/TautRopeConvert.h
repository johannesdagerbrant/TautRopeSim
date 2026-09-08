#pragma once

#include "CoreMinimal.h"
#include "TautRopeCore/Math.h"

// Conversions across the glue/core boundary. In one place because three glue
// files needed them, and three copies in anonymous namespaces collide the
// moment UBT compiles them in the same unity translation unit.
namespace TautRopeConvert
{
	FORCEINLINE FVector ToUE(const TautRope::Vec3& V)
	{
		return FVector(V.X, V.Y, V.Z);
	}

	FORCEINLINE FQuat ToUE(const TautRope::Quat& Q)
	{
		return FQuat(Q.X, Q.Y, Q.Z, Q.W);
	}

	FORCEINLINE TautRope::Vec3 ToCore(const FVector& V)
	{
		return TautRope::Vec3(V.X, V.Y, V.Z);
	}

	FORCEINLINE TautRope::Quat ToCore(const FQuat& Q)
	{
		return TautRope::Quat(Q.X, Q.Y, Q.Z, Q.W);
	}

	// Core emits colours as 0xAARRGGBB so it needs no engine colour type.
	FORCEINLINE FColor ToColor(unsigned int Color)
	{
		return FColor(
			static_cast<uint8>((Color >> 16) & 0xFFu)	// R
			, static_cast<uint8>((Color >> 8) & 0xFFu)	// G
			, static_cast<uint8>(Color & 0xFFu)			// B
			, static_cast<uint8>((Color >> 24) & 0xFFu)	// A
		);
	}
}
