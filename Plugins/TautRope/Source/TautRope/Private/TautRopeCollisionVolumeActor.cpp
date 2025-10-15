// Fill out your copyright notice in the Description page of Project Settings.


#include "TautRopeCollisionVolumeActor.h"

#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"


#if TAUT_ROPE_DEBUG_DRAWING
// 0 = off, 1 = on
static TAutoConsoleVariable<int32> CVarTautRopeCollisionVolumeDebugDraw(
	TEXT("TautRope.CollisionVolume.DebugDraw"),
	0,
	TEXT("Enable debug drawing for TautRopeCollisionVolumeActor (0=off, 1=on)"),
	ECVF_Cheat | ECVF_RenderThreadSafe
);
#endif

// Sets default values
ATautRopeCollisionVolumeActor::ATautRopeCollisionVolumeActor()
{
	CollisionVolume = CreateDefaultSubobject<UBoxComponent>(TEXT("CollisionVolume"));
	RootComponent = CollisionVolume;

#if TAUT_ROPE_DEBUG_DRAWING
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;
#else
	PrimaryActorTick.bCanEverTick = false;
	PrimaryActorTick.bStartWithTickEnabled = false;
#endif // TAUT_ROPE_DEBUG_DRAWING
}

// Called when the game starts or when spawned
void ATautRopeCollisionVolumeActor::BeginPlay()
{
	Super::BeginPlay();
}

#if TAUT_ROPE_DEBUG_DRAWING
void ATautRopeCollisionVolumeActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	UWorld* World = GetWorld();
	if (!IsValid(World))
	{
		return;
	}
	if (!IsValid(CollisionVolume))
	{
		return;
	}

	switch (World->WorldType)
	{
		case EWorldType::Editor:
		{
			if (!IsSelectedInEditor())
			{
				return;
			}
			break;
		}
		case EWorldType::PIE:
		case EWorldType::Game:
		case EWorldType::GamePreview:
		{
			if (CVarTautRopeCollisionVolumeDebugDraw.GetValueOnGameThread() < 1)
			{
				CollisionVolume->SetHiddenInGame(true);
				return;
			}
			CollisionVolume->SetHiddenInGame(false);
			break;
		}
		default:
		{
			return;
		}
	}

	for (const FTautRopeCollisionShape& Shape : StaticShapes)
	{
		Shape.DrawDebug(World);
	}
}
#endif // TAUT_ROPE_DEBUG_DRAWING

#if WITH_EDITOR
void ATautRopeCollisionVolumeActor::PopulateStaticShapes()
{
	StaticShapes.Empty();

	const UWorld* World = GetWorld();
	if (!IsValid(World))
	{
		return;
	}

	if (!IsValid(CollisionVolume))
	{
		return;
	}

	// Use the collision volume's world location, rotation, and box extent for the overlap
	const FVector BoxCenter = CollisionVolume->GetComponentLocation();
	const FVector BoxExtent = CollisionVolume->GetScaledBoxExtent();
	const FQuat BoxRotation = CollisionVolume->GetComponentQuat();

	TArray<FOverlapResult> Overlaps;
	GetWorld()->OverlapMultiByObjectType(
		Overlaps,
		BoxCenter,
		BoxRotation, // Use the component's rotation
		FCollisionObjectQueryParams(ECC_WorldStatic),
		FCollisionShape::MakeBox(BoxExtent)
	);
	if (Overlaps.IsEmpty())
	{
		return;
	}

	TArray<UPrimitiveComponent*> PrimComponents;
	PrimComponents.Reserve(Overlaps.Num());
	for (const FOverlapResult& Result : Overlaps)
	{
		UPrimitiveComponent* PrimComp = Result.Component.Get();
		if (IsValid(PrimComp))
		{
			PrimComponents.Add(PrimComp);
		}
	}
	for (UPrimitiveComponent* PrimComp : PrimComponents)
	{
		const UBodySetup* BodySetup = nullptr;
		const UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(PrimComp);
		if (IsValid(StaticMeshComponent))
		{
			if (StaticMeshComponent->GetStaticMesh())
			{
				BodySetup = StaticMeshComponent->GetStaticMesh()->GetBodySetup();
			}
		}
		else
		{
			// There is no const verson of GetBodySetup for UPrimitiveComponent
			UPrimitiveComponent* WritablePrimComp = const_cast<UPrimitiveComponent*>(PrimComp);
			BodySetup = WritablePrimComp->GetBodySetup();
		}
		if (!IsValid(BodySetup))
		{
			continue;
		}
		TArray<UPrimitiveComponent*> OtherPrimComponents = TArray(PrimComponents);
		OtherPrimComponents.Remove(PrimComp);
		for (const FKConvexElem& Convex : BodySetup->AggGeom.ConvexElems)
		{
			FTautRopeCollisionShape Shape = FTautRopeCollisionShape(Convex, PrimComp, OtherPrimComponents);
			StaticShapes.Add(MoveTemp(Shape));
		}
	}
}
#endif // WITH_EDITOR