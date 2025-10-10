#include "TautRopeActor.h"

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

static TAutoConsoleVariable<int32> CVarDrawDebugRopeShapes(
	TEXT("TautRope.DrawDebugRopeShapes"),
	0,
	TEXT("Draw rope collision shapes debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

static TAutoConsoleVariable<int32> CVarDrawDebugSweep(
	TEXT("TautRope.DrawDebugSweep"),
	0,
	TEXT("Draw sweep debug visualization.\n")
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

    TArray<FOverlapResult> Overlaps;
    GetWorld()->OverlapMultiByObjectType(
        Overlaps,
        GetActorLocation(),
        FQuat::Identity,
		FCollisionObjectQueryParams(ECC_WorldStatic),
		FCollisionShape::MakeSphere(SearchRadius)
    );

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

	NearbyShapes.Empty();
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
			TautRope::FRopeCollisionShape Shape = TautRope::FRopeCollisionShape(Convex, PrimComp, OtherPrimComponents);
			NearbyShapes.Add(MoveTemp(Shape));
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
	VertexPhase();
	PruningPhase();

#if TAUT_ROPE_DEBUG_DRAWING
	DrawDebug();
#endif // TAUT_ROPE_DEBUG_DRAWING
}

TArray<FVector> ATautRopeActor::MovementPhase()
{
	TArray<FVector> RopeTargetLocations;
	RopeTargetLocations.SetNum(RopePoints.Num());
	RopeTargetLocations[0] = StartPoint->GetComponentLocation();
	RopeTargetLocations.Last() = EndPoint->GetComponentLocation();
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
			const bool bIsCornerVertex = NearbyShapes[PointB.ShapeIndex].IsCornerVertex(PointB.VertIndex);
			if (!bIsCornerVertex)
			{
				continue;
			}
		}
		const FVector& LocationA = RopeTargetLocations[i - 1];
		const FVector& LocationC = RopeTargetLocations[i + 1];
		const TautRope::FRopeCollisionShape& Shape = NearbyShapes[PointB.ShapeIndex];
		const FIntVector2& Edge = Shape.Edges[PointB.EdgeIndex];
		const FVector& EdgeVertA = Shape.Vertices[Edge.X];
		const FVector& EdgeVertB = Shape.Vertices[Edge.Y];
		float OutDistAlongEdge = 0.f;
		float OutEdgeLength = 0.f;
		const FVector PrevRopeTargetLocation = RopeTargetLocations[i];
		const FVector RopeTargetLocation = TautRope::FindMinDistancePointBetweenABOnLineXY(LocationA, LocationC, EdgeVertA, EdgeVertB, OutDistAlongEdge, OutEdgeLength);
		if (OutDistAlongEdge < TAUT_ROPE_DISTANCE_TOLERANCE)
		{
			PointB.VertIndex = Edge.X;
			RopeTargetLocations[i] = EdgeVertA;
		}
		else if (OutDistAlongEdge > OutEdgeLength - TAUT_ROPE_DISTANCE_TOLERANCE)
		{
			PointB.VertIndex = Edge.Y;
			RopeTargetLocations[i] = EdgeVertB;
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
			TautRope::SweepSegmentThroughShapes(HitData, SegmentPointA, SegmentPointB, OriginLocationA, OriginLocationB, TargetLocationA, TargetLocationB, NearbyShapes, i + 1);
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
#if TAUT_ROPE_DEBUG_DRAWING
			if (CVarDrawDebugSweep.GetValueOnGameThread() != 0)
			{
				TautRope::DebugDrawSegmentSweep(
					GetWorld()
					, OriginLocationA
					, OriginLocationB
					, TargetLocationA
					, TargetLocationB
					, HitData
				);
			}
#endif // TAUT_ROPE_DEBUG_DRAWING
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

bool ATautRopeActor::VertexPhase()
{
	TBitArray<> PointsOnOrAdjecentToShapeVert = TautRope::GetAdjacentPointsOnSameVertexCone(RopePoints, NearbyShapes);
	for (int32 i = RopePoints.Num() - 1; i >= 0; --i)
	{
		if (PointsOnOrAdjecentToShapeVert[i])
		{
			TautRope::SweepRemovePoint(
				RopePoints
				, i
				, NearbyShapes
#if TAUT_ROPE_DEBUG_DRAWING
				, GetWorld()
				, CVarDrawDebugSweep.GetValueOnGameThread() != 0
#endif
			);
		}
	}
	TautRope::LetPointsOnVertexSlideOntoNewEdge(RopePoints, NearbyShapes);
	return true;
}

bool ATautRopeActor::PruningPhase()
{
	TBitArray<> PointsToRemove;
	PointsToRemove.Init(false, RopePoints.Num());
	for (int32 i = 1; i < RopePoints.Num() - 1; ++i)
	{
		const TautRope::FPoint& LastPoint = RopePoints[i - 1];
		const TautRope::FPoint& Point = RopePoints[i];
		const TautRope::FPoint& NextPoint = RopePoints[i + 1];
		const TautRope::FRopeCollisionShape& Shape = NearbyShapes[Point.ShapeIndex];
		if (Point.VertIndex != INDEX_NONE)
		{
			if (Shape.IsCornerVertex(Point.VertIndex))
			{
				if (
					FVector::DistSquared(Point.Location, LastPoint.Location) < TAUT_ROPE_DISTANCE_TOLERANCE_SQUARED
					|| FVector::DistSquared(Point.Location, NextPoint.Location) < TAUT_ROPE_DISTANCE_TOLERANCE_SQUARED
				)
				{
					PointsToRemove[i] = true;
					continue;
				}
			}
			else
			{
				PointsToRemove[i] = true;
				continue;
			}
		}
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
				, CVarDrawDebugSweep.GetValueOnGameThread() != 0
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

	if (CVarDrawDebugRopeShapes.GetValueOnGameThread() != 0)
	{
		DrawDebugRopeShapes();
	}
}

void ATautRopeActor::DrawDebugRope() const
{
	constexpr float RopeDebugRadius = 1.5f;
	// Draw rope segments
	for (int32 i = 0; i < RopePoints.Num(); ++i)
	{
		FVector EdgeUpA = FVector::ZeroVector;
		FVector EdgeUpB = FVector::ZeroVector;
		if (RopePoints[i].ShapeIndex != INDEX_NONE)
		{
			const TautRope::FRopeCollisionShape& ShapeA = NearbyShapes[RopePoints[i].ShapeIndex];
			EdgeUpA = ShapeA.EdgeRotations[RopePoints[i].EdgeIndex].GetUpVector();
		}

		if (RopePoints.IsValidIndex(i + 1))
		{
			if (RopePoints[i + 1].ShapeIndex != INDEX_NONE)
			{
				const TautRope::FRopeCollisionShape& ShapeB = NearbyShapes[RopePoints[i + 1].ShapeIndex];
				EdgeUpB = ShapeB.EdgeRotations[RopePoints[i + 1].EdgeIndex].GetUpVector();
			}
			DrawDebugLine(
				GetWorld()
				, RopePoints[i].Location
				, RopePoints[i + 1].Location
				, FColor::Black
				, false
				, -1.f
				, 0
				, RopeDebugRadius * 2.f
			);
		}
		DrawDebugSphere(
			GetWorld()
			, RopePoints[i].Location
			, RopeDebugRadius
			, 8
			, FColor::Black
			, false
			, -1.f
			, 0
			, RopeDebugRadius * 2.f
		);
	}
}

void ATautRopeActor::DrawDebugRopeTouchedShapeEdges() const
{
	for (int32 i = 0; i < RopePoints.Num(); ++i)
	{
		if (RopePoints[i].ShapeIndex != INDEX_NONE)
		{
			const TautRope::FRopeCollisionShape& Shape = NearbyShapes[RopePoints[i].ShapeIndex];
			const int32 EdgeVertIndexA = Shape.Edges[RopePoints[i].EdgeIndex].X;
			const int32 EdgeVertIndexB = Shape.Edges[RopePoints[i].EdgeIndex].Y;
			const FVector EdgeVertA = Shape.Vertices[EdgeVertIndexA];
			const FVector EdgeVertB = Shape.Vertices[EdgeVertIndexB];
			DrawDebugLine(
				GetWorld()
				, EdgeVertA
				, EdgeVertB
				, FColor::Blue
			);
		}
	}
}

void ATautRopeActor::DrawDebugRopeShapes() const
{
	for (const TautRope::FRopeCollisionShape& Shape : NearbyShapes)
	{
		Shape.DrawDebug(GetWorld());
	}
}
#endif // TAUT_ROPE_DEBUG_DRAWING