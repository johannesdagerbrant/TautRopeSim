using UnrealBuildTool;

// The simulation lives here so the standalone replay program can compile the
// same sources with no engine present. No source file in this module may
// include a UE header; the one dependency below exists for allocator parity
// inside Unreal and is invisible to the standalone build.
public class TautRopeCore : ModuleRules
{
	public TautRopeCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.NoPCHs;
		bRequiresImplementModule = false;

		// Core is depended on for one reason: allocator parity. UBT only wires up
		// a module's operator new/delete overrides when CORE_API is defined, which
		// happens only for modules that depend on Core. Without it this module
		// allocates on the CRT heap while the glue frees on FMemory's, and every
		// std::vector handed across the boundary is a cross-heap free.
		// No source file here includes a UE header, so the standalone replay build
		// is unaffected -- it never sees this dependency.
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core"
			}
			);

		// Non-monolithic targets build each module as a DLL, so the simulation
		// symbols need exporting. Monolithic links statically and wants neither.
		if (Target.LinkType != TargetLinkType.Monolithic)
		{
			PublicDefinitions.Add("TAUTROPE_CORE_SHARED=1");
			PrivateDefinitions.Add("TAUTROPE_CORE_EXPORTS=1");
		}
	}
}
