// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

// The Unreal glue for the rope system: actors, shape extraction from UE
// collision primitives, CVars and debug rendering. The simulation itself lives
// in TautRopeCore, which knows nothing about Unreal.
public class TautRope : ModuleRules
{
	public TautRope(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"TautRopeCore"
			}
			);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine"
			}
			);
	}
}
