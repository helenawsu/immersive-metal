# Detailed Code Walkthrough: Metal 3D Graphics Rendering

This document provides line-by-line explanations of the most critical rendering functions, helping you understand exactly what happens during 3D graphics rendering.

## 1. Main Rendering Function: `drawAndPresent`

**Location**: `SpatialRenderer.mm`

This is the core function called every frame to render the 3D scene.

```mm
void SpatialRenderer::drawAndPresent(cp_frame_t frame, cp_drawable_t drawable) {
    // === TIMING AND ANIMATION ===
    CFTimeInterval renderTime = CACurrentMediaTime();
    CFTimeInterval timestep = MIN(renderTime - _lastRenderTime, 1.0 / 60.0);
    _sceneTime += timestep;
    // Updates scene time for animations, capped at 60fps to prevent large jumps
```

### Globe Animation and Positioning

```mm
    // === 3D TRANSFORMATION MATH ===
    float c = cos(_sceneTime * 0.5f);  // Cosine for X rotation
    float s = sin(_sceneTime * 0.5f);  // Sine for X rotation
    
    // Create 4x4 transformation matrix for the globe
    simd_float4x4 modelTransform = simd_matrix(
        simd_make_float4(   c, 0.0f,    -s, 0.0f),  // X-axis (rotated)
        simd_make_float4(0.0f, 1.0f,  0.0f, 0.0f),  // Y-axis (unchanged)
        simd_make_float4(   s, 0.0f,     c, 0.0f),  // Z-axis (rotated)
        simd_make_float4(0.0f, estimatedHeadHeight, -1.5f, 1.0f)  // Translation
    );
    // This creates a rotation around Y-axis + translation to position globe
    // estimatedHeadHeight (1.25m) positions globe at comfortable viewing height
    // -1.5f Z places globe 1.5 meters in front of user
```

### Environment Portal Configuration

```mm
    // === MIXED REALITY SETUP ===
    if (_configuration.immersionStyle == SRImmersionStyleMixed) {
        _environmentMesh->setCutoffAngle(_configuration.portalCutoffAngle);
        // In mixed mode, limit environment visibility to create "portal" effect
    } else {
        _environmentMesh->setCutoffAngle(180);
        // Full immersion: show entire 360° environment
    }
```

### Stereoscopic View Setup

```mm
    // === MULTI-VIEW RENDERING SETUP ===
    size_t viewCount = cp_drawable_get_view_count(drawable);  // Usually 2 (left/right eye)
    
    std::array<MTLViewport, 2> viewports {};
    std::array<PoseConstants, 2> poseConstants {};
    std::array<PoseConstants, 2> poseConstantsForEnvironment {};
    
    for (int i = 0; i < viewCount; ++i) {
        // Get viewport (screen region) for this eye
        viewports[i] = viewportForViewIndex(drawable, i);
        
        // Calculate camera matrices for this eye
        poseConstants[i] = poseConstantsForViewIndex(drawable, i);
        
        // Special matrices for environment (remove translation)
        poseConstantsForEnvironment[i] = poseConstantsForViewIndex(drawable, i);
        // Remove translation so environment appears infinitely far away
        poseConstantsForEnvironment[i].viewMatrix.columns[3] = simd_make_float4(0.0, 0.0, 0.0, 1.0);
    }
```

### Rendering Mode Selection

The function then branches based on the rendering layout:

#### Dedicated Layout (Separate Render Passes)

```mm
    if (layout == cp_layer_renderer_layout_dedicated) {
        // === DEDICATED RENDERING: ONE PASS PER EYE ===
        for (int i = 0; i < viewCount; ++i) {
            MTLRenderPassDescriptor *renderPassDescriptor = createRenderPassDescriptor(drawable, i);
            id<MTLRenderCommandEncoder> renderCommandEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
            
            // === DEPTH AND CULLING SETUP ===
            [renderCommandEncoder setCullMode:MTLCullModeBack];  // Don't render back faces
```

#### Environment Rendering

```mm
            // === ENVIRONMENT RENDERING ===
            [renderCommandEncoder setFrontFacingWinding:MTLWindingClockwise];
            // Environment sphere has inverted normals (clockwise = front face)
            
            [renderCommandEncoder setDepthStencilState:_backgroundDepthStencilState];
            // Use background depth state (renders at far distance)
            
            [renderCommandEncoder setRenderPipelineState:_environmentRenderPipelineState];
            // Use environment shaders
            
            _environmentMesh->draw(renderCommandEncoder, &poseConstantsForEnvironment[i], 1);
            // Render the 360° environment sphere
```

#### Content Rendering

```mm
            // === MAIN CONTENT RENDERING ===
            [renderCommandEncoder setFrontFacingWinding:MTLWindingCounterClockwise];
            // Normal objects use counter-clockwise winding
            
            [renderCommandEncoder setDepthStencilState:_contentDepthStencilState];
            // Use standard depth testing
            
            [renderCommandEncoder setRenderPipelineState:_contentRenderPipelineState];
            // Use main content shaders
            
            _globeMesh->draw(renderCommandEncoder, &poseConstants[i], 1);
            // Render the textured globe
```

#### Layered Layout (Vertex Amplification)

```mm
    } else {
        // === LAYERED RENDERING: SINGLE PASS WITH VERTEX AMPLIFICATION ===
        MTLRenderPassDescriptor *renderPassDescriptor = createRenderPassDescriptor(drawable, 0);
        id<MTLRenderCommandEncoder> renderCommandEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
        
        // === MULTI-VIEW SETUP ===
        [renderCommandEncoder setViewports:viewports.data() count:viewCount];
        // Set both eye viewports simultaneously
        
        [renderCommandEncoder setVertexAmplificationCount:viewCount viewMappings:nil];
        // Enable vertex amplification: vertex shader runs once, outputs to multiple views
```

## 2. Pose Calculation: `poseConstantsForViewIndex`

This function calculates the camera matrices for stereoscopic rendering:

```mm
PoseConstants SpatialRenderer::poseConstantsForViewIndex(cp_drawable_t drawable, size_t index) {
    PoseConstants outPose;
    
    // === GET DEVICE TRACKING ===
    ar_device_anchor_t anchor = cp_drawable_get_device_anchor(drawable);
    simd_float4x4 poseTransform = ar_anchor_get_origin_from_anchor_transform(anchor);
    // Get current head position/orientation from ARKit
    
    cp_view_t view = cp_drawable_get_view(drawable, index);
    // Get view info for this eye (left=0, right=1)
    
    // === PROJECTION MATRIX CALCULATION ===
    if (@available(visionOS 2.0, *)) {
        outPose.projectionMatrix = cp_drawable_compute_projection(drawable,
                                                                  cp_axis_direction_convention_right_up_back,
                                                                  index);
        // Use built-in projection calculation (newer method)
    } else {
        // === MANUAL PROJECTION CALCULATION ===
        simd_float4 tangents = cp_view_get_tangents(view);
        simd_float2 depth_range = cp_drawable_get_depth_range(drawable);
        
        SPProjectiveTransform3D projectiveTransform = SPProjectiveTransform3DMakeFromTangents(
            tangents[0],        // left tan
            tangents[1],        // right tan  
            tangents[2],        // top tan
            tangents[3],        // bottom tan
            depth_range[1],     // near plane
            depth_range[0],     // far plane
            true                // reversed Z
        );
        // Create perspective projection from field-of-view tangents
        outPose.projectionMatrix = matrix_float4x4_from_double4x4(projectiveTransform.matrix);
    }
    
    // === VIEW MATRIX CALCULATION ===
    simd_float4x4 cameraMatrix = simd_mul(poseTransform, cp_view_get_transform(view));
    // Combine head tracking with eye offset
    
    outPose.viewMatrix = simd_inverse(cameraMatrix);
    // Invert to get view matrix (world-to-camera transform)
    
    return outPose;
}
```

## 3. Vertex Shader: `vertex_main`

**Location**: `SceneShaders.metal`

This is the GPU function that transforms vertices for 3D rendering:

```metal
[[vertex]]
LayeredVertexOut vertex_main(VertexIn in [[stage_in]],
                             constant PoseConstants *poses [[buffer(1)]],
                             constant InstanceConstants &instance [[buffer(2)]],
                             uint amplificationID [[amplification_id]])
{
    // === GET CAMERA DATA FOR THIS EYE ===
    constant auto &pose = poses[amplificationID];
    // amplificationID is 0 for left eye, 1 for right eye
    
    LayeredVertexOut out;
    
    // === CORE 3D TRANSFORMATION PIPELINE ===
    out.position = pose.projectionMatrix *           // 3. Project to screen space
                   pose.viewMatrix *                 // 2. Transform to camera space  
                   instance.modelMatrix *            // 1. Transform to world space
                   float4(in.position, 1.0f);       // Input vertex position
    
    // This is the standard MVP (Model-View-Projection) transformation chain
    // Model: Local object space → World space
    // View: World space → Camera space  
    // Projection: Camera space → Screen space
    
    // === NORMAL TRANSFORMATION ===
    out.viewNormal = (pose.viewMatrix * instance.modelMatrix * float4(in.normal, 0.0f)).xyz;
    // Transform normal to view space for lighting calculations
    // Note: normals use 0.0 w-component (vectors, not points)
    
    // === TEXTURE COORDINATE SETUP ===
    out.texCoords = in.texCoords;
    out.texCoords.x = 1.0f - out.texCoords.x;  // Flip horizontally for ModelIO compatibility
    
    // === MULTI-VIEW OUTPUT SETUP ===
    if (useLayeredRendering) {
        out.renderTargetIndex = amplificationID;  // Output to correct texture array slice
    }
    out.viewportIndex = amplificationID;          // Output to correct viewport region
    
    return out;
}
```

## 4. Environment Fragment Shader: `fragment_environment`

**Location**: `EnvironmentShaders.metal`

This shader creates the 360° environment with portal effects:

```metal
[[fragment]]
half4 fragment_environment(EnvironmentFragmentIn in [[stage_in]],
                           constant EnvironmentConstants &environment,
                           texture2d<half, access::sample> environmentTexture [[texture(0)]])
{
    constexpr sampler environmentSampler(coord::normalized, filter::linear, 
                                       mip_filter::none, address::repeat);
    
    // === VIEW DIRECTION ANALYSIS ===
    float3 V = normalize(in.worldViewDirection);
    // Get normalized view direction in world space
    
    float forwardness = -V.z;
    // How much we're looking forward (Z is negative forward in this coordinate system)
    
    // === PORTAL CUTOFF CALCULATION ===
    float cutoffStart = environment.portalCutoffAngles[0];  // Inner edge
    float cutoffEnd = environment.portalCutoffAngles[1];    // Outer edge
    
    half portalOpacity = half(1.0f - smoothstep(cutoffStart, cutoffEnd, forwardness));
    // Smooth falloff: 1.0 when looking forward, 0.0 when looking backward
    // smoothstep creates smooth transition between cutoffStart and cutoffEnd
    
    // === EQUIRECTANGULAR TEXTURE SAMPLING ===
    float3 N = normalize(in.modelNormal);
    // Get surface normal (pointing inward for environment sphere)
    
    float2 texCoords = EquirectUVFromCubeDirection(N);
    // Convert 3D direction to 2D texture coordinates
    
    half4 color = environmentTexture.sample(environmentSampler, texCoords);
    // Sample the HDR environment texture
    
    // === FINAL OUTPUT ===
    return half4(color.rgb * portalOpacity, portalOpacity);
    // Multiply color by portal opacity for smooth blending
    // Alpha channel controls transparency for mixed reality
}
```

### Equirectangular Mapping Function

```metal
static float2 EquirectUVFromCubeDirection(float3 v) {
    const float2 scales { 0.1591549f, 0.3183099f };  // 1/(2π) and 1/π
    const float2 biases { 0.5f, 0.5f };              // Center coordinates
    
    // Convert 3D direction to spherical coordinates
    float2 uv = float2(atan2(-v.x, v.z),    // Longitude (horizontal angle)
                       asin(-v.y)) *         // Latitude (vertical angle)
                scales + biases;
    // atan2(-v.x, v.z): horizontal rotation around Y-axis
    // asin(-v.y): vertical angle from equator
    // scales: convert radians to [0,1] range
    // biases: shift to [0,1] from [-0.5,0.5]
    
    return uv;
}
```

## 5. Mesh Drawing: `TexturedMesh::draw`

**Location**: `Mesh.mm`

This function sets up GPU resources and issues draw calls:

```mm
void TexturedMesh::draw(id<MTLRenderCommandEncoder> renderCommandEncoder, 
                       PoseConstants *poseConstants, size_t poseCount) {
    // === INSTANCE DATA SETUP ===
    InstanceConstants instanceConstants;
    instanceConstants.modelMatrix = modelMatrix();
    // Get current transformation matrix for this object
    
    // === VERTEX DATA BINDING ===
    MTKSubmesh *submesh = _mesh.submeshes.firstObject;
    MTKMeshBuffer *vertexBuffer = _mesh.vertexBuffers.firstObject;
    
    [renderCommandEncoder setVertexBuffer:vertexBuffer.buffer 
                                   offset:vertexBuffer.offset 
                                  atIndex:0];
    // Bind vertex data (positions, normals, texture coordinates)
    
    [renderCommandEncoder setVertexBytes:poseConstants 
                                  length:sizeof(PoseConstants) * poseCount 
                                 atIndex:1];
    // Bind camera matrices for each eye
    
    [renderCommandEncoder setVertexBytes:&instanceConstants 
                                  length:sizeof(instanceConstants) 
                                 atIndex:2];
    // Bind object transformation matrix
    
    // === TEXTURE BINDING ===
    [renderCommandEncoder setFragmentTexture:_texture atIndex:0];
    // Bind diffuse texture for fragment shader
    
    // === DRAW CALL ===
    [renderCommandEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                     indexCount:submesh.indexCount
                                      indexType:submesh.indexType
                                    indexBuffer:submesh.indexBuffer.buffer
                              indexBufferOffset:submesh.indexBuffer.offset];
    // Issue GPU draw command:
    // - MTLPrimitiveTypeTriangle: render triangles
    // - indexCount: number of vertex indices
    // - indexBuffer: contains vertex indices for triangles
}
```

## Key 3D Graphics Concepts Demonstrated

### 1. **Model-View-Projection (MVP) Pipeline**
Every vertex goes through this transformation sequence to convert from 3D object coordinates to 2D screen coordinates.

### 2. **Stereoscopic Rendering** 
The system renders the same scene from two slightly different camera positions (left and right eye) to create depth perception.

### 3. **Vertex Amplification**
An optimization technique where a single vertex shader execution outputs geometry for both eyes, reducing GPU workload.

### 4. **Environment Mapping**
The 360° background uses equirectangular projection to map a 2D HDR image onto a 3D sphere.

### 5. **Portal Effects**
In mixed reality mode, the environment gradually fades based on view direction, creating a smooth transition between virtual and real worlds.

### 6. **Depth Testing**
The system uses reversed Z-buffer for better depth precision, with the environment rendered first (as background) and objects rendered front-to-back.