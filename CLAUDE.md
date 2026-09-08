# TautRopeSim

## Git workflow

Johan owns this repo. Commit and push directly to `main` — do not create branches
or open pull requests unless explicitly asked.

## Build

Unreal Engine 5.8 (`C:\Program Files\Epic Games\UE_5.8`). Requires MSVC >= 14.50,
supplied by VS 2026 Build Tools; the targets pin
`WindowsPlatform.Compiler = WindowsCompiler.VisualStudio2026` because UBT otherwise
falls back to an older toolset that cannot compile the 5.8 engine headers.

Build the editor target:

```
"C:\Program Files\Epic Games\UE_5.8\Engine\Build\BatchFiles\Build.bat" ^
  TautRopeSimEditor Win64 Development -Project="C:\Projects\TautRopeSim\TautRopeSim.uproject"
```

Regenerate VS Code project files with `-projectfiles -vscode`.

## Layout

The rope system lives in the `TautRope` plugin (`Plugins/TautRope`), not in the
`TautRopeSim` game module.

## Working on the rope simulation

See `Plugins/TautRope/AGENTS.md`. It covers the flight recorder, the headless
replay, the debugging loop, and the traps -- several of which have already cost
a crash and a wrong-looking divergence.
