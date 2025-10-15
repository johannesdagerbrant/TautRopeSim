#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TautRopeCollisionShape.h"
#include "TautRopeConfig.h"
#include "TautRopePoint.h"
#include "TautRopeActor.generated.h"

class ATautRopeCollisionVolumeActor;

UCLASS(HideCategories = (
	"Actor"
	, "Input"
	, "Replication"
	, "Rendering"
	, "HLOD"
	, "Physics"
	, "Collision"
	, "Cooking"
	, "Networking"
	, "WorldPartition"
	, "LevelInstance"
	, "DataLayers"
	)
)
class TAUTROPE_API ATautRopeActor : public AActor
{
	GENERATED_BODY()

public:
	ATautRopeActor();

protected:
	virtual void BeginPlay() override;

public:
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Taut Rope")
	float MaxLength = 1000000.f;

private:
	TArray<FVector> MovementPhase();
	bool CollisionPhase(TArray<FVector>& TargetRopePoints);
	bool PruningPhase();

#if TAUT_ROPE_DEBUG_DRAWING
	void DrawDebug() const;
	void DrawDebugRope() const;
	void DrawDebugRopeTouchedShapeEdges() const;
#endif // TAUT_ROPE_DEBUG_DRAWING

	UPROPERTY(VisibleAnywhere, Category = "Taut Rope")
	USceneComponent* StartPoint;

	UPROPERTY(VisibleAnywhere, Category = "Taut Rope")
	USceneComponent* EndPoint;

#if WITH_EDITORONLY_DATA
	UPROPERTY()
	class UBillboardComponent* StartPointBillboard;

	UPROPERTY()
	class UBillboardComponent* EndPointBillboard;
#endif

	TArray<TautRope::FPoint> RopePoints;
	TArray<FTautRopeCollisionShape> NearbyShapes;
};