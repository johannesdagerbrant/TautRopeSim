#include "TautRopeActor.h"
#include "TautRope.h"
#include "TautRopeConfig.h"
#include "TautRopeCollisionVolumeActor.h"

#include "Components/SceneComponent.h"
#include "Components/BillboardComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/KismetSystemLibrary.h"

ATautRopeActor::ATautRopeActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;
	StartPoint = CreateDefaultSubobject<USceneComponent>(TEXT("StartPoint"));
	StartPoint->SetupAttachment(RootComponent);
	EndPoint = CreateDefaultSubobject<USceneComponent>(TEXT("EndPoint"));
	EndPoint->SetupAttachment(RootComponent);
	StartPoint->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	EndPoint->SetRelativeLocation(FVector(0.f, MaxLength, 0.f));

#if WITH_EDITORONLY_DATA
	static ConstructorHelpers::FObjectFinder<UTexture2D> StartSpriteFinder(TEXT("/Engine/EditorResources/Waypoint"));

	StartPointBillboard = CreateDefaultSubobject<UBillboardComponent>(TEXT("StartPointBillboard"));
	StartPointBillboard->SetupAttachment(StartPoint);
	if (StartSpriteFinder.Succeeded())
	{
		StartPointBillboard->SetSprite(StartSpriteFinder.Object);
	}
	StartPointBillboard->bIsEditorOnly = true;

	static ConstructorHelpers::FObjectFinder<UTexture2D> EndSpriteFinder(TEXT("/Engine/EditorResources/S_TargetPoint"));
	EndPointBillboard = CreateDefaultSubobject<UBillboardComponent>(TEXT("EndPointBillboard"));
	EndPointBillboard->SetupAttachment(EndPoint);
	if (EndSpriteFinder.Succeeded())
	{
		EndPointBillboard->SetSprite(EndSpriteFinder.Object);
	}
	EndPointBillboard->bIsEditorOnly = true;
#endif
}


void ATautRopeActor::BeginPlay()
{
    Super::BeginPlay();

    TArray<AActor*> OverlappingActors;
    UKismetSystemLibrary::SphereOverlapActors(
        GetWorld(),
		StartPoint->GetComponentLocation(),
		MaxLength,
		TArray<TEnumAsByte<EObjectTypeQuery>>(), // leave empty to include all
        ATautRopeCollisionVolumeActor::StaticClass(),
        TArray<AActor*>(), // optional ignore actors
        OverlappingActors
    );

    for (AActor* Actor : OverlappingActors)
    {
        const ATautRopeCollisionVolumeActor* TautRopeCollisionVolumeActor = Cast<ATautRopeCollisionVolumeActor>(Actor);
        if (IsValid(TautRopeCollisionVolumeActor))
        {
			TautRope.AppendToNearbyShapes(TautRopeCollisionVolumeActor->GetStaticShapes());
        }
    }
}

void ATautRopeActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	TautRope.UpdateRope(
		StartPoint->GetComponentLocation()
		, EndPoint->GetComponentLocation()
		, MaxLength
#if TAUT_ROPE_DEBUG_DRAWING
		, GetWorld()
#endif // TAUT_ROPE_DEBUG_DRAWING
		);

#if TAUT_ROPE_DEBUG_DRAWING
	TautRope.DrawDebug(GetWorld());
#endif // TAUT_ROPE_DEBUG_DRAWING
}