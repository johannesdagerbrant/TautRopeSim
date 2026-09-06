// Fill out your copyright notice in the Description page of Project Settings.


#include "TautRopeCollisionVolumeActor.h"

#include "Engine/OverlapResult.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"
#include "TautRopeDebugDraw.h"
#include "TautRopeShapeBuilder.h"
#include "TautRopeShapeSerialization.h"

DEFINE_LOG_CATEGORY_STATIC(LogTautRopeVolume, Log, All);


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

void ATautRopeCollisionVolumeActor::PostLoad()
{
	Super::PostLoad();
	TautRopeShapeSerialization::Load(StaticShapes, SerializedShapes);
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

	FTautRopeDebugDraw DebugDraw(World);
	for (const TautRope::CollisionShape& Shape : StaticShapes)
	{
		DrawCollisionShape(DebugDraw, Shape);
	}
}
#endif // TAUT_ROPE_DEBUG_DRAWING

#if WITH_EDITOR
void ATautRopeCollisionVolumeActor::PopulateStaticShapes()
{
	StaticShapes.clear();

	const UWorld* World = GetWorld();
	if (!IsValid(World))
	{
		UE_LOG(LogTautRopeVolume, Warning, TEXT("%s: no world"), *GetName());
		return;
	}

	if (!IsValid(CollisionVolume))
	{
		UE_LOG(LogTautRopeVolume, Warning, TEXT("%s: no collision volume component"), *GetName());
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
		UE_LOG(LogTautRopeVolume, Warning,
			TEXT("%s: no WorldStatic primitives overlap the volume (centre %s, extent %s). Nothing to sample."),
			*GetName(), *BoxCenter.ToCompactString(), *BoxExtent.ToCompactString());
		return;
	}

	UE_LOG(LogTautRopeVolume, Display, TEXT("%s: %d overlapping primitives"), *GetName(), Overlaps.Num());

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
			UE_LOG(LogTautRopeVolume, Verbose, TEXT("  %s: no body setup, skipped"), *PrimComp->GetName());
			continue;
		}
		UE_LOG(LogTautRopeVolume, Display,
			TEXT("  %s: %d convex, %d box, %d sphere, %d capsule"),
			*PrimComp->GetName(),
			BodySetup->AggGeom.ConvexElems.Num(),
			BodySetup->AggGeom.BoxElems.Num(),
			BodySetup->AggGeom.SphereElems.Num(),
			BodySetup->AggGeom.SphylElems.Num());
		TArray<UPrimitiveComponent*> OtherPrimComponents = TArray(PrimComponents);
		OtherPrimComponents.Remove(PrimComp);
		for (const FKConvexElem& Convex : BodySetup->AggGeom.ConvexElems)
		{
			StaticShapes.push_back(TautRopeShapeBuilder::Build(Convex, PrimComp, OtherPrimComponents));
		}
		// Boxes are convex hulls with eight corners, so they are sampled too.
		for (const FKBoxElem& Box : BodySetup->AggGeom.BoxElems)
		{
			StaticShapes.push_back(TautRopeShapeBuilder::Build(Box, PrimComp, OtherPrimComponents));
		}
	}

	int32 TotalVertices = 0;
	int32 TotalEdges = 0;
	for (const TautRope::CollisionShape& Shape : StaticShapes)
	{
		TotalVertices += static_cast<int32>(Shape.Vertices.size());
		TotalEdges += static_cast<int32>(Shape.Edges.size());
	}

	if (!StaticShapes.empty())
	{
		TautRopeShapeSerialization::Save(StaticShapes, SerializedShapes);
		MarkPackageDirty();
	}

	if (StaticShapes.empty())
	{
		UE_LOG(LogTautRopeVolume, Warning,
			TEXT("%s: %d primitives overlapped but produced no shapes. Convex and box simple collision are sampled; sphere and capsule are not."),
			*GetName(), PrimComponents.Num());
	}
	else
	{
		UE_LOG(LogTautRopeVolume, Display,
			TEXT("%s: built %d shapes (%d vertices, %d edges) from %d primitives. Save the level to keep them."),
			*GetName(), static_cast<int32>(StaticShapes.size()), TotalVertices, TotalEdges, PrimComponents.Num());
	}
}
#endif // WITH_EDITOR