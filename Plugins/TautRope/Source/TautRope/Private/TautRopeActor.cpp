#include "TautRopeActor.h"
#include "TautRopeCollisionVolumeActor.h"

#include "TautRopeHelpersCollision.h"
#include "TautRopeHelpersMovement.h"
#include "TautRopeHelpersPruning.h"
#include "TautRopeHelpersVertexHandling.h"

#include "Components/SceneComponent.h"
#include "Components/BillboardComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/ShapeComponent.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/StaticMesh.h"
#include "Kismet/KismetSystemLibrary.h"

#if TAUT_ROPE_DEBUG_DRAWING
static TAutoConsoleVariable<int32> CVarDrawDebugRope(
	TEXT("TautRope.DrawDebugRope"),
	0,
	TEXT("Draw rope debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

static TAutoConsoleVariable<int32> CVarDrawDebugRopeTouchedEdges(
	TEXT("TautRope.DrawDebugRopeTouchedEdges"),
	0,
	TEXT("Draw rope touched shape edges debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

static TAutoConsoleVariable<int32> CVarDrawDebugSegmentSweep(
	TEXT("TautRope.DrawDebugSegmentSweep"),
	0,
	TEXT("Draw segment sweep debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

static TAutoConsoleVariable<int32> CVarDrawDebugRemoveSweep(
	TEXT("TautRope.DrawDebugRemoveSweep"),
	0,
	TEXT("Draw remove sweep debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);
#endif // TAUT_ROPE_DEBUG_DRAWING

ATautRopeActor::ATautRopeActor()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;
	StartPoint = CreateDefaultSubobject<USceneComponent>(TEXT("StartPoint"));
	StartPoint->SetupAttachment(RootComponent);
	EndPoint = CreateDefaultSubobject<USceneComponent>(TEXT("EndPoint"));
	EndPoint->SetupAttachment(RootComponent);
	StartPoint->SetRelativeLocation(FVector(0.f, -50.f, 0.f));
	EndPoint->SetRelativeLocation(FVector(0.f, 50.f, 0.f));

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

    const float SearchRadius = 1000000.f;

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
			NearbyShapes.Append(TautRopeCollisionVolumeActor->GetStaticShapes());
        }
    }

    RopePoints =
    {
		TautRope::FPoint(StartPoint->GetComponentLocation())
        , TautRope::FPoint(EndPoint->GetComponentLocation())
    };
}

void ATautRopeActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	TArray<FVector> TargetPoints = MovementPhase();
	CollisionPhase(TargetPoints);
	PruningPhase();

#if TAUT_ROPE_DEBUG_DRAWING
	DrawDebug();
#endif // TAUT_ROPE_DEBUG_DRAWING
}

// Used to get a normalized vector inbetween two unit length orthogonal vectors.
constexpr float INV_SQRT2 = 0.70710678f; 

TArray<FVector> ATautRopeActor::MovementPhase()
{
	ensure(RopePoints.Num() >= 2);

	TArray<FVector> RopeTargetLocations;

	RopeTargetLocations.SetNum(RopePoints.Num());
	RopeTargetLocations[0] = StartPoint->GetComponentLocation();

	float RopeDistanceToSecondLastPoint = 0.f;
	for (int32 Index = 0; Index < RopePoints.Num() - 2; ++Index)
	{
		RopeDistanceToSecondLastPoint += FVector::Dist(
			RopePoints[Index].Location
			, RopePoints[Index + 1].Location
		);
	}
	const float AvaliableDistanceTowardsEndPoint = MaxLength - RopeDistanceToSecondLastPoint;
	const FVector SecondLastPointLocation = RopePoints[RopePoints.Num() - 2].Location;
	const FVector EndPointLocation = EndPoint->GetComponentLocation();
	const float DistanceToEndPoint = FVector::Dist(SecondLastPointLocation, EndPointLocation);
	const float ToEndPointAlpha = FMath::Clamp(AvaliableDistanceTowardsEndPoint / DistanceToEndPoint, 0.f, 1.f);
	const FVector LastPointLocation = FMath::Lerp(SecondLastPointLocation, EndPointLocation, ToEndPointAlpha);
	RopeTargetLocations.Last() = LastPointLocation;
	if (RopeTargetLocations.Num() == 2)
	{
		return RopeTargetLocations;
	}
	for (int32 i = 1; i < RopePoints.Num() - 1; ++i)
	{
		RopeTargetLocations[i] = RopePoints[i].Location;
	}
	// TODO: Grouping of rope points that belong to the same vertex fan of edges so we can draw a straight line across multiple edges in 2d space,
	for (int32 i = 1; i < RopePoints.Num() - 1; ++i)
	{
		TautRope::FPoint& PointB = RopePoints[i];
		if (PointB.VertIndex != INDEX_NONE)
		{
			continue;
		}
		const FVector& LocationA = RopeTargetLocations[i - 1];
		const FVector& LocationC = RopeTargetLocations[i + 1];
		const FTautRopeCollisionShape& Shape = NearbyShapes[PointB.ShapeIndex];
		const FIntVector2& Edge = Shape.Edges[PointB.EdgeIndex];
		const bool bIsEdgeCornerAtVertexA = NearbyShapes[PointB.ShapeIndex].IsCornerVertex(Edge.X);
		const bool bIsEdgeCornerAtVertexB = NearbyShapes[PointB.ShapeIndex].IsCornerVertex(Edge.Y);
		const FVector& EdgeVertA = Shape.Vertices[Edge.X];
		const FVector& EdgeVertB = Shape.Vertices[Edge.Y];
		float OutDistAlongEdge = 0.f;
		float OutEdgeLength = 0.f;
		const FVector PrevRopeTargetLocation = RopeTargetLocations[i];
		const FVector RopeTargetLocation = TautRope::FindMinDistancePointBetweenABOnLineXY(LocationA, LocationC, EdgeVertA, EdgeVertB, OutDistAlongEdge, OutEdgeLength);

		if (!bIsEdgeCornerAtVertexA && OutDistAlongEdge < TAUT_ROPE_DISTANCE_TOLERANCE)
		{
			PointB.VertIndex = Edge.X;
			const FQuat& EdgeRotation = Shape.EdgeRotations[PointB.EdgeIndex];
			RopeTargetLocations[i] = EdgeVertA + (EdgeRotation.GetUpVector() - EdgeRotation.GetForwardVector()) * INV_SQRT2 * TAUT_ROPE_VERTEX_CROSSING_OFFSET;
		}
		else if (!bIsEdgeCornerAtVertexB && OutDistAlongEdge > OutEdgeLength - TAUT_ROPE_DISTANCE_TOLERANCE)
		{
			PointB.VertIndex = Edge.Y;
			const FQuat& EdgeRotation = Shape.EdgeRotations[PointB.EdgeIndex];
			RopeTargetLocations[i] = EdgeVertB + (EdgeRotation.GetUpVector() + EdgeRotation.GetForwardVector()) * INV_SQRT2 * TAUT_ROPE_VERTEX_CROSSING_OFFSET;
		}
		else
		{
			PointB.VertIndex = INDEX_NONE;
			RopeTargetLocations[i] = RopeTargetLocation;
		}
	}
	return RopeTargetLocations;
}

bool ATautRopeActor::CollisionPhase(TArray<FVector>& TargetRopePoints)
{
	TArray<FVector> OriginRopePoints;
	OriginRopePoints.SetNum(RopePoints.Num());
	for (int32 i = 0; i < RopePoints.Num(); ++i)
	{
		OriginRopePoints[i] = RopePoints[i].Location;
	}

	int32 CollisionItr = 0;
	bool bIsAnyNewCollision = true;
	while (bIsAnyNewCollision && CollisionItr < TAUT_ROPE_MAX_COLLISION_ITERATIONS)
	{
		TArray<TautRope::FHitData> SegmentSweepHits;
		for (int32 i = 0; i < RopePoints.Num() - 1; ++i)
		{
			TautRope::FPoint& SegmentPointA = RopePoints[i];
			TautRope::FPoint& SegmentPointB = RopePoints[i + 1];
			const FVector& OriginLocationA = OriginRopePoints[i];
			const FVector& OriginLocationB = OriginRopePoints[i + 1];
			const FVector& TargetLocationA = TargetRopePoints[i];
			const FVector& TargetLocationB = TargetRopePoints[i + 1];

			TautRope::FHitData HitData;
			TautRope::SweepSegmentThroughShapes(
				HitData
				, SegmentPointA
				, SegmentPointB
				, OriginLocationA
				, OriginLocationB
				, TargetLocationA
				, TargetLocationB
				, NearbyShapes
				, i + 1
#if TAUT_ROPE_DEBUG_DRAWING
				, GetWorld()
				, CVarDrawDebugSegmentSweep.GetValueOnGameThread() != 0
#endif
			);
			if (HitData.bIsHit)
			{
				if (HitData.bIsHitOnFirstTriangleSweep)
				{
					SegmentPointA.VertIndex = INDEX_NONE;
				}
				else
				{
					SegmentPointB.VertIndex = INDEX_NONE;
				}
				SegmentSweepHits.Add(MoveTemp(HitData));
			}
		}
		for (int32 i = SegmentSweepHits.Num() - 1; i >= 0; --i)
		{
			const TautRope::FHitData& HitData = SegmentSweepHits[i];
			RopePoints.Insert(TautRope::FPoint(HitData), HitData.RopePointIndex);
			OriginRopePoints.Insert(HitData.Location, HitData.RopePointIndex);
			TargetRopePoints.Insert(HitData.Location, HitData.RopePointIndex);
		}
		bIsAnyNewCollision = !SegmentSweepHits.IsEmpty();
		CollisionItr++;
	}
	return false;
}

bool ATautRopeActor::PruningPhase()
{
	TBitArray<> PointsToRemove = TautRope::GetAdjacentPointsOnSameVertexCone(RopePoints, NearbyShapes);
	for (int32 i = 1; i < RopePoints.Num() - 1; ++i)
	{
		if (PointsToRemove[i])
		{
			continue;
		}
		const TautRope::FPoint& LastPoint = RopePoints[i - 1];
		const TautRope::FPoint& Point = RopePoints[i];
		const TautRope::FPoint& NextPoint = RopePoints[i + 1];
		const FTautRopeCollisionShape& Shape = NearbyShapes[Point.ShapeIndex];
		if (Point.ShapeIndex == LastPoint.ShapeIndex && Point.EdgeIndex == LastPoint.EdgeIndex)
		{
			PointsToRemove[i] = true;
			continue;
		}
		const FQuat& EdgeRotation = Shape.EdgeRotations[Point.EdgeIndex];
		const bool bIsRopeWrappingEdge = TautRope::IsRopeWrappingEdge(
			LastPoint.Location
			, Point.Location
			, NextPoint.Location
			, EdgeRotation
		);
		if (!bIsRopeWrappingEdge)
		{
			PointsToRemove[i] = true;
		}
	}
	for (int32 i = RopePoints.Num() - 2; i > 0; --i)
	{
		if (PointsToRemove[i])
		{
			TautRope::SweepRemovePoint(
				RopePoints
				, i
				, NearbyShapes
#if TAUT_ROPE_DEBUG_DRAWING
				, GetWorld()
				, CVarDrawDebugRemoveSweep.GetValueOnGameThread() != 0
#endif
			);
		}
	}
	return PointsToRemove.Contains(true);
}


#if TAUT_ROPE_DEBUG_DRAWING
void ATautRopeActor::DrawDebug() const
{
	if (CVarDrawDebugRope.GetValueOnGameThread() != 0)
	{
		DrawDebugRope();
	}

	if (CVarDrawDebugRopeTouchedEdges.GetValueOnGameThread() != 0)
	{
		DrawDebugRopeTouchedShapeEdges();
	}
}

void ATautRopeActor::DrawDebugRope() const
{
	constexpr float RopeDebugRadius = 1.5f;
	// Draw rope segments
	for (int32 i = 0; i < RopePoints.Num(); ++i)
	{
		FVector UpOffsetA = FVector::ZeroVector;
		FVector UpOffsetB = FVector::ZeroVector;
		if (RopePoints[i].ShapeIndex != INDEX_NONE)
		{
			const FTautRopeCollisionShape& ShapeA = NearbyShapes[RopePoints[i].ShapeIndex];
			UpOffsetA = ShapeA.EdgeRotations[RopePoints[i].EdgeIndex].GetUpVector() * TAUT_ROPE_DISTANCE_TOLERANCE;
		}

		if (RopePoints.IsValidIndex(i + 1))
		{
			if (RopePoints[i + 1].ShapeIndex != INDEX_NONE)
			{
				const FTautRopeCollisionShape& ShapeB = NearbyShapes[RopePoints[i + 1].ShapeIndex];
				UpOffsetB = ShapeB.EdgeRotations[RopePoints[i + 1].EdgeIndex].GetUpVector() * TAUT_ROPE_DISTANCE_TOLERANCE;
			}
			DrawDebugLine(
				GetWorld()
				, RopePoints[i].Location + UpOffsetA
				, RopePoints[i + 1].Location + UpOffsetB
				, FColor::Black
				, false
				, -1.f
				, 0
			);
		}

		DrawDebugSphere(
			GetWorld()
			, RopePoints[i].Location + UpOffsetA
			, 1.f
			, 4
			, FColor::Black
		);

	}
}

void ATautRopeActor::DrawDebugRopeTouchedShapeEdges() const
{
	for (int32 i = 0; i < RopePoints.Num(); ++i)
	{
		if (RopePoints[i].ShapeIndex != INDEX_NONE)
		{
			const FTautRopeCollisionShape& Shape = NearbyShapes[RopePoints[i].ShapeIndex];
			const int32 EdgeIndex = RopePoints[i].EdgeIndex;
			const int32 EdgeVertIndexA = Shape.Edges[EdgeIndex].X;
			const int32 EdgeVertIndexB = Shape.Edges[EdgeIndex].Y;
			const FVector EdgeVertA = Shape.Vertices[EdgeVertIndexA];
			const FVector EdgeVertB = Shape.Vertices[EdgeVertIndexB];
			const FVector UpOffset = Shape.EdgeRotations[EdgeIndex].GetUpVector() * TAUT_ROPE_DISTANCE_TOLERANCE;
			DrawDebugLine(
				GetWorld()
				, EdgeVertA + UpOffset
				, EdgeVertB + UpOffset
				, FColor::Blue
			);
		}
	}
}
#endif // TAUT_ROPE_DEBUG_DRAWING