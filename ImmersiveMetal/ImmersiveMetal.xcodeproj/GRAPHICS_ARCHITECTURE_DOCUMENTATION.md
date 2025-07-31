# Metal 3D Graphics Rendering Architecture for visionOS

This documentation explains how the Metal graphics rendering system works for the Vision Pro, emphasizing the 3D graphics pipeline and how each component contributes to the final rendered display.

## Overview

This project demonstrates advanced Metal rendering techniques specifically designed for visionOS spatial computing. It renders a 3D textured sphere (globe) within a 360-degree environment, supporting both full immersion and mixed reality modes with sophisticated stereoscopic rendering optimizations.

## Architecture Components

### 1. Application Layer (`App.swift` & `ContentView.swift`)

**Purpose**: SwiftUI interface that manages immersion states and launches the Metal rendering pipeline.

**Key 3D Graphics Contributions**:
- **Immersion Control**: Switches between `.full` (complete virtual environment) and `.mixed` (blended with passthrough)
- **Layer Configuration**: Sets up Metal rendering parameters including:
  - Foveation support for performance optimization
  - Color format (rgba16Float for HDR rendering)
  - Layout selection (layered vs dedicated rendering modes)
- **Portal Cutoff**: Controls the visibility angle of the environment in mixed mode

```swift
// Critical configuration for 3D rendering
configuration.layout = supportedLayouts.contains(.layered) ? .layered : .dedicated
configuration.isFoveationEnabled = supportsFoveation
configuration.colorFormat = .rgba16Float
```

### 2. Spatial Rendering Engine (`SpatialRenderingEngine.mm`)

**Purpose**: Manages the rendering lifecycle and ARKit integration for spatial tracking.

**Key 3D Graphics Contributions**:
- **ARKit Integration**: Provides world tracking for 6DOF (6 degrees of freedom) head tracking
- **Frame Timing**: Synchronizes rendering with display refresh and prediction
- **Pose Estimation**: Creates accurate device anchors for stereoscopic rendering
- **Threading**: Manages dedicated render thread for consistent frame rates

**Critical Functions**:
```mm
ar_device_anchor_t createPoseForTiming(cp_frame_timing_t timing) {
    // Queries ARKit for precise head position at predicted display time
    // Essential for accurate stereoscopic rendering
}
```

### 3. Core Renderer (`SpatialRenderer.h` & `SpatialRenderer.mm`)

**Purpose**: The heart of the 3D graphics pipeline - manages Metal resources, render pipelines, and frame rendering.

#### 3D Graphics Architecture:

**Resource Management**:
- **Globe Mesh**: A textured sphere representing the main 3D object
- **Environment Mesh**: A large inverted sphere for 360° background
- **Textures**: Earth texture ("bluemarble.png") and HDR environment ("studio.hdr")

**Render Pipeline Setup**:
```mm
void makeRenderPipelines() {
    // Creates separate pipelines for:
    // 1. Content rendering (the 3D globe)
    // 2. Environment rendering (360° background)
    // 3. Supports both dedicated and layered rendering modes
}
```

**Stereoscopic Rendering Modes**:

1. **Dedicated Layout**: Renders each eye separately
   - One render pass per eye
   - Traditional approach, simpler but less efficient

2. **Layered/Shared Layout**: Uses vertex amplification
   - Single render pass outputs to both eyes
   - More efficient, uses vertex shaders to duplicate geometry

**Frame Rendering Process** (`drawAndPresent`):
1. **Time Management**: Updates scene time for animations
2. **View Matrix Calculation**: Computes stereoscopic camera matrices
3. **Model Transforms**: Rotates the globe over time
4. **Environment Setup**: Configures portal cutoff for mixed mode
5. **Multi-view Rendering**: Renders to both eyes with appropriate view transforms

### 4. 3D Mesh System (`Mesh.h` & `Mesh.mm`)

**Purpose**: Manages 3D geometry, textures, and vertex data.

#### Key Classes:

**Base Mesh Class**:
- Defines vertex descriptor (position, normal, texture coordinates)
- Manages model transformation matrix
- Abstract interface for rendering

**TexturedMesh Class**:
- Loads 3D models using ModelIO
- Manages diffuse textures
- Implements standard mesh rendering

**SpatialEnvironmentMesh Class** (specialized for 360° environments):
- Creates large inverted sphere for environment mapping
- Supports portal cutoff angles for mixed reality
- Uses equirectangular texture mapping
- Implements smooth falloff at portal edges

```mm
// Vertex descriptor defines 3D vertex structure
vertexDescriptor.attributes[0].format = MTLVertexFormatFloat3; // position
vertexDescriptor.attributes[1].format = MTLVertexFormatFloat3; // normal
vertexDescriptor.attributes[2].format = MTLVertexFormatFloat2; // texture coordinates
```

### 5. Shader System (`SceneShaders.metal` & `EnvironmentShaders.metal`)

**Purpose**: GPU programs that define how vertices are transformed and pixels are colored.

#### Scene Shaders (`SceneShaders.metal`):

**Vertex Shaders**:
- `vertex_main`: Supports vertex amplification for efficient multi-view rendering
- `vertex_dedicated_main`: Traditional single-view rendering
- Transforms vertices through model-view-projection pipeline
- Handles normal transformation for lighting calculations

**Fragment Shader**:
- Samples diffuse texture using interpolated UV coordinates
- Applies texture coordinates flipping for ModelIO compatibility

```metal
// Core 3D transformation pipeline
out.position = pose.projectionMatrix * pose.viewMatrix * instance.modelMatrix * float4(in.position, 1.0f);
out.viewNormal = (pose.viewMatrix * instance.modelMatrix * float4(in.normal, 0.0f)).xyz;
```

#### Environment Shaders (`EnvironmentShaders.metal`):

**Advanced Features**:
- **Equirectangular Mapping**: Converts 3D directions to 2D texture coordinates
- **Portal Rendering**: Smooth falloff at portal edges for mixed reality
- **View Direction Calculation**: Computes world-space view vectors for environment lookup

```metal
// Converts 3D direction to equirectangular UV coordinates
static float2 EquirectUVFromCubeDirection(float3 v) {
    const float2 scales { 0.1591549f, 0.3183099f };
    float2 uv = float2(atan2(-v.x, v.z), asin(-v.y)) * scales + biases;
    return uv;
}
```

**Portal Effect**:
```metal
// Smooth falloff for mixed reality portal effect
half portalOpacity = half(1.0f - smoothstep(cutoffStart, cutoffEnd, forwardness));
```

### 6. Shader Data Structures (`ShaderTypes.h`)

**Purpose**: Defines data structures shared between CPU and GPU code.

**PoseConstants**: Camera and projection matrices for each eye
**InstanceConstants**: Per-object transformation matrices  
**EnvironmentConstants**: Environment-specific rendering parameters

## 3D Graphics Rendering Pipeline Flow

### Frame Rendering Sequence:

1. **ARKit Tracking**: Get precise head pose for current frame
2. **View Matrix Setup**: Calculate left/right eye camera matrices
3. **Projection Setup**: Configure perspective projection for each eye
4. **Depth Buffer Setup**: Initialize Z-buffer for proper 3D depth sorting
5. **Environment Pass**:
   - Render large inverted sphere
   - Sample HDR environment texture
   - Apply portal cutoff for mixed mode
   - Use greater depth comparison (reversed Z)
6. **Content Pass**:
   - Render 3D globe with Earth texture
   - Apply lighting calculations using normals
   - Standard depth testing
7. **Present**: Display final frames to Vision Pro displays

### Optimization Techniques:

**Vertex Amplification**: 
- Single vertex shader execution creates geometry for both eyes
- Reduces vertex processing workload by ~50%

**Layered Rendering**:
- Outputs to texture array instead of separate textures
- More efficient memory usage and bandwidth

**Foveation**:
- Reduces rendering quality in peripheral vision
- Maintains high quality in central vision area

**Reversed Z-Buffer**:
- Uses greater depth comparison instead of less
- Improves depth precision for distant objects

## Mixed Reality Features

**Portal Cutoff System**:
- Smoothly blends virtual environment with real world
- Configurable cutoff angle from UI
- Smooth falloff prevents harsh edges
- Maintains depth relationships between virtual and real objects

**Passthrough Integration**:
- Virtual objects properly occlude real objects when closer
- Environment fades based on viewing direction
- Maintains spatial relationships in mixed mode

## Performance Considerations

**Multi-threading**:
- Dedicated render thread prevents UI blocking
- Efficient command buffer management

**Memory Management**:
- Reuses vertex and index buffers
- Efficient texture loading and management

**Frame Pacing**:
- Synchronizes with display refresh rate
- Predictive timing for reduced latency

This architecture demonstrates sophisticated 3D graphics techniques specifically optimized for spatial computing, providing smooth, immersive experiences while maintaining the performance requirements of Vision Pro's dual 4K displays.