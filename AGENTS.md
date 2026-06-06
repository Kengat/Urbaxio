# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

Urbaxio is a VR-enabled 3D CAD modeling application built with C++17, OpenGL, CUDA, and OpenXR. It features B-Rep geometry (via OpenCASCADE), GPU-accelerated voxel sculpting, and a custom immediate-mode VR UI system.

## Build Commands

```bash
# Configure with CMake preset (from project root)
cmake --preset vs2022

# Build (from build directory)
cmake --build . --config Debug
cmake --build . --config Release

# Run the executable
./bin/Debug/Urbaxio.exe
./bin/Release/Urbaxio.exe
```

### Snapshot Generation (for AI context)

```bash
# Generate full project snapshot
cmake --build . --target generate_snapshot

# Generate partial snapshot (reads from snapshot_files.txt)
cmake --build . --target generate_partial_snapshot

# Generate project structure
cmake --build . --target generate_structure
```

## Architecture

### Library Structure

```
Urbaxio/
├── cad_kernel/     # OpenCASCADE wrapper (B-Rep operations, triangulation)
├── engine/         # Core engine library
│   ├── commands/   # Undo/redo command pattern (ICommand, CommandManager)
│   ├── geometry/   # Polymorphic geometry (IGeometry → BRepGeometry, MeshGeometry, VolumetricGeometry)
│   └── gpu/        # CUDA module (sculpting, hash grid, marching cubes)
└── shell/          # Application layer
    ├── tools/      # Interactive tools (ITool → SelectTool, PushPullTool, GpuSculptTool, etc.)
    └── ui/         # VR widget system (IVRWidget → VRPanel, VRButtonWidget, etc.)
```

### Key Design Patterns

**Geometry Polymorphism**: `SceneObject` holds an `IGeometry*` that can be:
- `BRepGeometry` - Parametric CAD (OpenCASCADE TopoDS_Shape)
- `MeshGeometry` - Triangle mesh only
- `VolumetricGeometry` - Voxel/SDF data (for sculpting)

**Tool System**: Tools implement `ITool` interface with `Activate()`, `Deactivate()`, `OnLeftMouseDown()`, `OnUpdate()`, `RenderPreview()`. The `ToolContext` struct bundles scene/camera/selection state.

**VR UI**: Widget hierarchy under `IVRWidget` with `Update(Ray, triggerState)` and `Render()`. Panels (`VRPanel`) contain widgets, managed by `VRUIManager`.

**Command Pattern**: All geometry modifications go through `CommandManager` for undo/redo. Commands capture state via `Scene::CaptureState()` / `RestoreState()`.

### GPU Module (CUDA)

The `urbaxio_gpu` library (built only with CUDA toolkit) provides:
- `DynamicGpuHashGrid` - Sparse voxel storage with hash-based block allocation
- Sculpting kernels (`GpuSculptKernels.cu`)
- Marching cubes mesh extraction (`HashGridMeshing.cu`, `GpuMeshingKernels.cu`)

GPU code targets CUDA architecture 75 (RTX 20xx series) by default.

### Rendering Pipeline

- `Renderer` class handles OpenGL state, shader programs, and static geometry batching
- Multiview rendering for VR (`RenderFrameMultiview`) using `GL_OVR_multiview2`
- `VRManager` wraps OpenXR for headset/controller tracking and swapchain management

## Dependencies

Managed via vcpkg (C:/vcpkg):
- SDL2, OpenGL/GLAD, Dear ImGui
- OpenXR, GLM, Eigen3, libigl
- OpenVDB (with NanoVDB), TBB
- OpenCASCADE (hardcoded path: C:/vcpkg/installed/x64-windows)

## Code Conventions

- Namespace: `Urbaxio::Engine` for core, `Urbaxio::Tools` for tools, `Urbaxio::UI` for widgets
- Scene coordinates use Z-up (matching VR standing reference)
- Float tolerance for vertex comparison: `SCENE_POINT_EQUALITY_TOLERANCE = 1e-4f`
- GPU-managed objects (`isGpuManaged()`) have their mesh updated through `MeshManager`, not direct buffer updates
