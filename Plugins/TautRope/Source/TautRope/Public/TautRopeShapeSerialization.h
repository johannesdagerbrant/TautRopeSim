#pragma once

#include "CoreMinimal.h"
#include "TautRopeCore/CollisionShape.h"

#include <vector>

// Persists core shapes into a byte blob held by a UPROPERTY. A blob rather than
// a mirrored USTRUCT so the shape has exactly one definition, in TautRopeCore --
// a mirror would be a second copy of the layout to keep in sync.
namespace TautRopeShapeSerialization
{
	TAUTROPE_API void Save(const std::vector<TautRope::CollisionShape>& Shapes, TArray<uint8>& OutBytes);
	TAUTROPE_API void Load(std::vector<TautRope::CollisionShape>& OutShapes, const TArray<uint8>& Bytes);
}
