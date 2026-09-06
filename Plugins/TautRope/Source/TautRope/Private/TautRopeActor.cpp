#include "TautRopeActor.h"
#include "TautRopeConfig.h"
#include "TautRopeCollisionVolumeActor.h"
#include "TautRopeDebugDraw.h"
#include "TautRopeRecorder.h"

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

    Recorder.SetLabel(GetName());

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
			const TConstArrayView<FTautRopeCollisionShape> Shapes = TautRopeCollisionVolumeActor->GetStaticShapes();
			std::vector<TautRope::CollisionShape> CoreShapes;
			CoreShapes.reserve(Shapes.Num());
			for (const FTautRopeCollisionShape& Shape : Shapes)
			{
				CoreShapes.push_back(Shape.ToCore());
			}
			Rope.AppendToNearbyShapes(CoreShapes);
        }
    }
}

void ATautRopeActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Recorder.Flush();
	Super::EndPlay(EndPlayReason);
}

void ATautRopeActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

#if TAUT_ROPE_DEBUG_DRAWING
	FTautRopeDebugDraw DebugDraw(GetWorld());
	TautRope::IDebugDraw* const Debug = DebugDraw.IsUsable() ? &DebugDraw : nullptr;
#else
	TautRope::IDebugDraw* const Debug = nullptr;
#endif

	const FVector StartLocation = StartPoint->GetComponentLocation();
	const FVector EndLocation = EndPoint->GetComponentLocation();

	TautRope::FrameCapture* const Capture =
		Recorder.BeginFrame(Rope, StartLocation, EndLocation, MaxLength, DeltaTime);

	Rope.UpdateRope(
		TautRope::Vec3(StartLocation.X, StartLocation.Y, StartLocation.Z)
		, TautRope::Vec3(EndLocation.X, EndLocation.Y, EndLocation.Z)
		, MaxLength
		, Debug
		, Capture
	);

#if TAUT_ROPE_DEBUG_DRAWING
	if (Debug != nullptr)
	{
		if (FTautRopeDebugDraw::WantsRope())
		{
			Rope.DrawDebugRope(*Debug);
		}
		if (FTautRopeDebugDraw::WantsRopeTouchedEdges())
		{
			Rope.DrawDebugRopeTouchedShapeEdges(*Debug);
		}
	}
#endif // TAUT_ROPE_DEBUG_DRAWING
}
