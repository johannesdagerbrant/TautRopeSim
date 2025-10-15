// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Containers/ArrayView.h"
#include "TautRopeConfig.h"
#include "TautRopeCollisionShape.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "TautRopeCollisionVolumeActor.generated.h"

UCLASS(HideCategories = ("Actor", "Input", "Replication", "Rendering", "HLOD", "Physics", "Collision", "Cooking", "Networking", "WorldPartition", "LevelInstance", "DataLayers"))
class TAUTROPE_API ATautRopeCollisionVolumeActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ATautRopeCollisionVolumeActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
#if TAUT_ROPE_DEBUG_DRAWING
	virtual void Tick(float DeltaTime) override;
#if WITH_EDITOR
	virtual bool ShouldTickIfViewportsOnly() const override { return true; }
#endif // WITH_EDITOR
#endif // TAUT_ROPE_DEBUG_DRAWING

public:	
	// Primitive collision volume for sampling
	UPROPERTY(BlueprintReadOnly, Category = "Taut Rope Collision")
	TObjectPtr<UBoxComponent> CollisionVolume;

	/** Returns a const view of the static rope collision shapes found within the collision volume */
	TConstArrayView<FTautRopeCollisionShape> GetStaticShapes() const { return StaticShapes; }

#if WITH_EDITOR
	// Expose a button in the details panel to populate StaticShapes from simple collision of primitives within the collision volume
	UFUNCTION(CallInEditor, Category = "Taut Rope Collision")
	void PopulateStaticShapes();
#endif

private:
	// Stored data from simple collision of primitives within the collision volume
	UPROPERTY()
	TArray<FTautRopeCollisionShape> StaticShapes;
};
