#pragma once

#include "CoreMinimal.h"
#include "TautRopeCollisionShape.h"
#include "TautRopeConfig.h"
#include "TautRopePoint.h"

#include "TautRope.generated.h"


USTRUCT()
struct TAUTROPE_API FTautRope
{
	GENERATED_BODY()

public:
	void AppendToNearbyShapes(const TConstArrayView<FTautRopeCollisionShape>& Shapes);

	TArray<FVector> GetRopePoints() const;

	void UpdateRope(
		const FVector& StartLocation
		, const FVector& EndLocation
		, const float MaxLength
#if TAUT_ROPE_DEBUG_DRAWING
		, const UWorld* World
#endif // TAUT_ROPE_DEBUG_DRAWING
	);

#if TAUT_ROPE_DEBUG_DRAWING
	void DrawDebug(const UWorld* World) const;
#endif // TAUT_ROPE_DEBUG_DRAWING

private:
	TArray<FVector> MovementPhase(
		const FVector& StartLocation
		, const FVector& EndLocation
		, const float MaxLength
	);
	bool CollisionPhase(
		TArray<FVector>& TargetRopePoints
#if TAUT_ROPE_DEBUG_DRAWING
		, const UWorld* World
#endif // TAUT_ROPE_DEBUG_DRAWING
	);
	bool PruningPhase(
#if TAUT_ROPE_DEBUG_DRAWING
		const UWorld* World
#endif // TAUT_ROPE_DEBUG_DRAWING
	);

#if TAUT_ROPE_DEBUG_DRAWING
	void DrawDebugRope(const UWorld* World) const;
	void DrawDebugRopeTouchedShapeEdges(const UWorld* World) const;
#endif // TAUT_ROPE_DEBUG_DRAWING

	TArray<TautRope::FPoint> RopePoints;
	TArray<FTautRopeCollisionShape> NearbyShapes;
};