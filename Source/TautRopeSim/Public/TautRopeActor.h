#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TautRopeCollisionShape.h"
#include "TautRopeConfig.h"
#include "TautRopePoint.h"
#include "TautRopeActor.generated.h"

UCLASS()
class TAUTROPESIM_API ATautRopeActor : public AActor
{
	GENERATED_BODY()

public:
	ATautRopeActor();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

private:
	TArray<FVector> MovementPhase();
	bool CollisionPhase(TArray<FVector>& TargetRopePoints);
	bool VertexPhase();
	bool PruningPhase();

#if TAUT_ROPE_DEBUG_DRAWING
	void DrawDebug() const;
	void DrawDebugRope() const;
	void DrawDebugRopeTouchedShapeEdges() const;
	void DrawDebugRopeShapes() const;
#endif // TAUT_ROPE_DEBUG_DRAWING

	UPROPERTY(VisibleAnywhere, Category = "Rope")
	USceneComponent* StartPoint;

	UPROPERTY(VisibleAnywhere, Category = "Rope")
	USceneComponent* EndPoint;

#if WITH_EDITORONLY_DATA
	UPROPERTY()
	class UBillboardComponent* StartPointBillboard;

	UPROPERTY()
	class UBillboardComponent* EndPointBillboard;
#endif

	TArray<TautRope::FPoint> RopePoints;
	TArray<TautRope::FRopeCollisionShape> NearbyShapes;
};