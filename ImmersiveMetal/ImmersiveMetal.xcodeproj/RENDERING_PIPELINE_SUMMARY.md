# Metal 3D Graphics Rendering Pipeline Summary for Vision Pro

This document provides a high-level overview of how your Metal 3D graphics rendering pipeline works for visionOS spatial computing.

## Pipeline Flow Overview

### 1. **Initialization Phase** (`SpatialRenderer` constructor)
- **Metal Device Setup**: Get GPU device from CompositorServices
- **Command Queue**: Create GPU command execution queue
- **Resource Creation**: Load 3D meshes and textures
- **Pipeline Compilation**: Create vertex and fragment shader pipelines

### 2. **Per-Frame Rendering** (`drawAndPresent`)
- **Timing**: Calculate animation time deltas
- **3D Transforms**: Update object positions and rotations
- **Stereoscopic Setup**: Calculate matrices for left/right eye cameras
- **GPU Commands**: Record rendering commands for execution
- **Presentation**: Display final frames to Vision Pro

### 3. **3D Transformation Pipeline** (Vertex Shaders)
Every vertex goes through this transformation sequence:
1. **Model Transform**: Object space → World space (position/rotate objects)
2. **View Transform**: World space → Camera space (camera viewpoint)  
3. **Projection Transform**: Camera space → Screen space (perspective/FOV)

### 4. **Pixel Generation** (Fragment Shaders)
- **Texture Sampling**: Read colors from texture images
- **Portal Effects**: Blend virtual/real world in mixed mode
- **Environment Mapping**: 360° HDR background rendering

## Key 3D Graphics Concepts

### **Stereoscopic Rendering**
- Vision Pro renders separate images for left and right eyes
- Creates depth perception through parallax
- Two rendering modes:
  - **Dedicated**: Separate pass per eye (simpler)
  - **Layered**: Single pass with vertex amplification (more efficient)

### **Model-View-Projection (MVP) Pipeline**
```
Vertex Position → Model Matrix → View Matrix → Projection Matrix → Screen Position
```
This is the core of 3D graphics - every vertex follows this path.

### **Depth Testing**
- Uses Z-buffer to determine which objects are in front
- Reversed-Z for better precision at far distances
- Environment renders at maximum depth (infinitely far)

### **Mixed Reality Portal**
- Environment fades based on viewing direction
- Smooth transitions prevent jarring edges
- Configurable cutoff angles from UI

## Rendering Order (Back-to-Front)

1. **Clear Buffers**: Reset color and depth
2. **Environment Sphere**: 360° HDR background (far depth)
3. **3D Content**: Globe and other objects (standard depth)
4. **Present**: Display to Vision Pro screens

## Performance Optimizations

### **Vertex Amplification**
- Single vertex shader execution outputs to both eyes
- Reduces vertex processing by ~50%
- Enabled in layered rendering mode

### **Texture Optimization**
- HDR environment maps for realistic lighting
- Linear filtering for smooth texture sampling
- Efficient memory layout for GPU access

### **Culling**
- Back-face culling: Don't render triangles facing away
- Frustum culling: Don't render objects outside camera view

## Data Flow

```
CPU (Swift/ObjC++) ←→ GPU (Metal Shaders)
        ↓                    ↓
   Command Buffer    →    Vertex Shader
   Mesh Data        →    Fragment Shader
   Camera Matrices  →    Transform Pipeline
   Textures         →    Texture Sampling
        ↓                    ↓
   Frame Timing     →    Screen Output
```

## Key Files and Their Roles

- **`SpatialRenderer.mm`**: Main rendering coordinator
- **`SceneShaders.metal`**: 3D object vertex/fragment shaders
- **`EnvironmentShaders.metal`**: 360° environment rendering
- **`Mesh.mm`**: 3D geometry and texture management
- **`SpatialRenderingEngine.mm`**: ARKit integration and frame timing

This pipeline demonstrates modern 3D graphics techniques optimized specifically for Vision Pro's dual 4K displays and spatial computing requirements.