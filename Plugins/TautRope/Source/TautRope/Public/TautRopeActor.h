#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TautRopeActor.generated.h"

class ATautRopeCollisionVolumeActor;

struct FTautRope;

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
	float MaxLength = 500.f;

private:
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

	FTautRope TautRope;
};