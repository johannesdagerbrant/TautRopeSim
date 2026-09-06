#pragma once

#include "CoreMinimal.h"
#include "TautRopeCore/CollisionShape.h"

struct FKConvexElem;
struct FKBoxElem;
class UPrimitiveComponent;

// Builds the simulation's collision shape from UE collision primitives. This is
// the only part of shape handling that genuinely needs Unreal: convex element
// data and the line traces that carve intermediate edges where neighbouring
// geometry intersects. The shape type itself belongs to TautRopeCore.
namespace TautRopeShapeBuilder
{
	// Convex geometry alone, transformed into world space.
	TAUTROPE_API TautRope::CollisionShape Build(
		const FKConvexElem& Convex
		, const FTransform& ComponentTransform
	);

	// As above, then subdivided along edges that other primitives intersect, so
	// the rope has edges to slide along where shapes meet.
	TAUTROPE_API TautRope::CollisionShape Build(
		const FKConvexElem& Convex
		, const UPrimitiveComponent* PrimComp
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
	);

	// Box simple collision is the common case for static meshes and converts to a
	// convex hull exactly, so it goes through the same path rather than being
	// silently ignored.
	TAUTROPE_API TautRope::CollisionShape Build(
		const FKBoxElem& Box
		, const UPrimitiveComponent* PrimComp
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
	);
}
