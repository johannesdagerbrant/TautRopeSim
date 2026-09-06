// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "TautRopeConfig.h"
#include "TautRopeCore/CollisionShape.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"

#include <vector>

#include "TautRopeCollisionVolumeActor.generated.h"

UCLASS(HideCategories = ("Actor", "Input", "Replication", "Rendering", "HLOD", "Physics", "Collision", "Cooking", "Networking", "WorldPartition", "LevelInstance", "DataLayers"))
class TAUTROPE_API ATautRopeCollisionVolumeActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ATautRopeCollisionVolumeActor();

	virtual void PostLoad() override;

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

	/** The static rope collision shapes found within the collision volume */
	const std::vector<TautRope::CollisionShape>& GetStaticShapes() const { return StaticShapes; }

#if WITH_EDITOR
	// Expose a button in the details panel to populate StaticShapes from simple collision of primitives within the collision volume
	UFUNCTION(CallInEditor, Category = "Taut Rope Collision")
	void PopulateStaticShapes();
#endif

private:
	// Runtime form, rebuilt from SerializedShapes on load.
	std::vector<TautRope::CollisionShape> StaticShapes;

	// Persisted form. A blob rather than a mirrored USTRUCT, so the shape keeps a
	// single definition -- in TautRopeCore.
	UPROPERTY()
	TArray<uint8> SerializedShapes;
};
