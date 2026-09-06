using UnrealBuildTool;

// The simulation lives here so the standalone replay program can compile the
// same sources with no engine present. Nothing in this module may include a UE
// header, and it deliberately declares no dependencies -- not even Core.
public class TautRopeCore : ModuleRules
{
	public TautRopeCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.NoPCHs;
		bRequiresImplementModule = false;

		// Non-monolithic targets build each module as a DLL, so the simulation
		// symbols need exporting. Monolithic links statically and wants neither.
		if (Target.LinkType != TargetLinkType.Monolithic)
		{
			PublicDefinitions.Add("TAUTROPE_CORE_SHARED=1");
			PrivateDefinitions.Add("TAUTROPE_CORE_EXPORTS=1");
		}
	}
}
