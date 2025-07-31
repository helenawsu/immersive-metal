#include "SpatialRenderer.h"
#include "Mesh.h"
#include "ShaderTypes.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <Spatial/Spatial.h>

#include <vector>

static simd_float4x4 matrix_float4x4_from_double4x4(simd_double4x4 m) {
    return simd_matrix(simd_make_float4(m.columns[0][0], m.columns[0][1], m.columns[0][2], m.columns[0][3]),
                       simd_make_float4(m.columns[1][0], m.columns[1][1], m.columns[1][2], m.columns[1][3]),
                       simd_make_float4(m.columns[2][0], m.columns[2][1], m.columns[2][2], m.columns[2][3]),
                       simd_make_float4(m.columns[3][0], m.columns[3][1], m.columns[3][2], m.columns[3][3]));
}

SpatialRenderer::SpatialRenderer(cp_layer_renderer_t layerRenderer, SRConfiguration *configuration) :
    _layerRenderer { layerRenderer },
    _configuration { configuration },
    _sceneTime(0.0),
    _lastRenderTime(CACurrentMediaTime())
{
    // === METAL DEVICE SETUP ===
    // Get the Metal device from the CompositorServices layer renderer
    // This device represents the GPU and is used to create all Metal resources
    _device = cp_layer_renderer_get_device(layerRenderer);
    
    // Create a command queue - this is where we submit GPU work
    // The command queue executes commands in order and manages GPU scheduling
    _commandQueue = [_device newCommandQueue];

    // === 3D GRAPHICS PIPELINE INITIALIZATION ===
    // Step 1: Load and prepare 3D geometry and textures
    makeResources();

    // Step 2: Create the rendering pipeline states that define how vertices and pixels are processed
    makeRenderPipelines();
}

void SpatialRenderer::makeResources() {
    // === 3D GEOMETRY CREATION ===
    // Create a Metal buffer allocator to manage GPU memory for mesh data
    MTKMeshBufferAllocator *bufferAllocator = [[MTKMeshBufferAllocator alloc] initWithDevice:_device];
    
    // === MAIN 3D OBJECT: TEXTURED GLOBE ===
    // Create a sphere mesh using ModelIO - this will be our main 3D content
    // - Radius: 0.5 units (1 meter diameter)
    // - 24x24 segments: provides smooth curves while keeping vertex count reasonable
    // - Triangles: standard primitive type for 3D graphics
    // - inwardNormals: NO means normals point outward (standard for solid objects)
    MDLMesh *sphereMesh = [MDLMesh newEllipsoidWithRadii:simd_make_float3(0.5, 0.5, 0.5)
                                          radialSegments:24
                                        verticalSegments:24
                                            geometryType:MDLGeometryTypeTriangles
                                           inwardNormals:NO
                                              hemisphere:NO
                                               allocator:bufferAllocator];
    
    // Convert ModelIO mesh to Metal mesh and apply Earth texture
    // This creates vertex/index buffers on GPU and loads/processes the texture
    _globeMesh = std::make_unique<TexturedMesh>(sphereMesh, @"bluemarble.png", _device);

    // === 360° ENVIRONMENT BACKGROUND ===
    // Create a large inverted sphere (3 meter radius) for the environment
    // This provides a 360° HDR background that appears infinitely far away
    // The "studio.hdr" is an equirectangular HDR image for realistic lighting
    _environmentMesh = std::make_unique<SpatialEnvironmentMesh>(@"studio.hdr", 3.0, _device);
}

void SpatialRenderer::makeRenderPipelines() {
    NSError *error = nil;
    
    // === VISION PRO DISPLAY CONFIGURATION ===
    // Get the layer configuration which defines how we render to the Vision Pro displays
    cp_layer_renderer_configuration_t layerConfiguration = cp_layer_renderer_get_configuration(_layerRenderer);
    cp_layer_renderer_layout layout = cp_layer_renderer_configuration_get_layout(layerConfiguration);

    // === RENDER PIPELINE DESCRIPTOR SETUP ===
    // This defines the structure of our rendering pipeline
    MTLRenderPipelineDescriptor *pipelineDescriptor = [MTLRenderPipelineDescriptor new];
    
    // Configure pixel formats to match Vision Pro's display capabilities
    // Color: High dynamic range 16-bit float for realistic lighting
    // Depth: 32-bit float for precise depth testing in VR/AR
    pipelineDescriptor.colorAttachments[0].pixelFormat = cp_layer_renderer_configuration_get_color_format(layerConfiguration);
    pipelineDescriptor.depthAttachmentPixelFormat = cp_layer_renderer_configuration_get_depth_format(layerConfiguration);

    // Load our Metal shader library (compiled shaders)
    id<MTLLibrary> library = [_device newDefaultLibrary];
    id<MTLFunction> vertexFunction, fragmentFunction;

    // === STEREOSCOPIC RENDERING MODE DETECTION ===
    // Vision Pro supports different rendering approaches for efficiency:
    // - Dedicated: Separate render pass for each eye (traditional, simpler)
    // - Layered: Single pass with vertex amplification (more efficient)
    BOOL layoutIsDedicated = (layout == cp_layer_renderer_layout_dedicated);
    BOOL layoutIsLayered = (layout == cp_layer_renderer_layout_layered);

    // Function constants allow compile-time optimization in shaders
    // This tells shaders which rendering mode to use
    MTLFunctionConstantValues *functionConstants = [MTLFunctionConstantValues new];
    [functionConstants setConstantValue:&layoutIsLayered type:MTLDataTypeBool withName:@"useLayeredRendering"];

    {
        // === MAIN CONTENT PIPELINE (3D Objects like the Globe) ===
        
        // Select appropriate vertex shader based on rendering mode
        // Dedicated: renders one eye at a time
        // Layered: uses vertex amplification to render both eyes simultaneously
        vertexFunction = [library newFunctionWithName: layoutIsDedicated ? @"vertex_dedicated_main" : @"vertex_main"
                                       constantValues:functionConstants
                                                error:&error];
        fragmentFunction = [library newFunctionWithName:@"fragment_main"];
        
        pipelineDescriptor.vertexFunction = vertexFunction;
        pipelineDescriptor.fragmentFunction = fragmentFunction;
        
        // Set vertex descriptor to match our mesh data structure
        // (position, normal, texture coordinates)
        pipelineDescriptor.vertexDescriptor = _globeMesh->vertexDescriptor();
        
        if (!layoutIsDedicated) {
            // === VERTEX AMPLIFICATION SETUP ===
            // For layered rendering, we need special configuration:
            // - Triangle topology: we're rendering triangular meshes
            // - Amplification count: each vertex gets duplicated for both eyes
            pipelineDescriptor.inputPrimitiveTopology = MTLPrimitiveTopologyClassTriangle;
            pipelineDescriptor.maxVertexAmplificationCount = 2;  // Left and right eye
        }

        // Create the compiled pipeline state for content rendering
        _contentRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:pipelineDescriptor error:&error];
        if (_contentRenderPipelineState == nil) {
            NSLog(@"Error occurred when creating render pipeline state: %@", error);
        }
    }
    {
        // === ENVIRONMENT PIPELINE (360° Background) ===
        
        // Environment uses different shaders optimized for large background spheres
        vertexFunction = [library newFunctionWithName:layoutIsDedicated ? @"vertex_dedicated_environment" : @"vertex_environment"
                                       constantValues:functionConstants
                                                error:&error];
        fragmentFunction = [library newFunctionWithName:@"fragment_environment"];
        
        pipelineDescriptor.vertexFunction = vertexFunction;
        pipelineDescriptor.fragmentFunction = fragmentFunction;
        pipelineDescriptor.vertexDescriptor = _environmentMesh->vertexDescriptor();
        
        if (!layoutIsDedicated) {
            pipelineDescriptor.inputPrimitiveTopology = MTLPrimitiveTopologyClassTriangle;
            pipelineDescriptor.maxVertexAmplificationCount = 2;
        }

        // Create the compiled pipeline state for environment rendering
        _environmentRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:pipelineDescriptor error:&error];
        if (_environmentRenderPipelineState == nil) {
            NSLog(@"Error occurred when creating render pipeline state: %@", error);
        }
    }

    // === DEPTH-STENCIL STATES FOR 3D DEPTH TESTING ===
    
    // Content depth state: Standard 3D depth testing for solid objects
    MTLDepthStencilDescriptor *depthDescriptor = [MTLDepthStencilDescriptor new];
    depthDescriptor.depthWriteEnabled = YES;  // Write depth values for proper occlusion
    depthDescriptor.depthCompareFunction = MTLCompareFunctionGreater;  // Reversed-Z for better precision
    _contentDepthStencilState = [_device newDepthStencilStateWithDescriptor:depthDescriptor];

    // Background depth state: Environment renders at maximum depth (infinitely far)
    depthDescriptor.depthWriteEnabled = YES;
    depthDescriptor.depthCompareFunction = MTLCompareFunctionGreater;  // Same as content for consistency
    _backgroundDepthStencilState = [_device newDepthStencilStateWithDescriptor:depthDescriptor];
}

void SpatialRenderer::drawAndPresent(cp_frame_t frame, cp_drawable_t drawable) {
    // === FRAME TIMING AND ANIMATION ===
    // Calculate time delta for smooth animation independent of frame rate
    CFTimeInterval renderTime = CACurrentMediaTime();
    CFTimeInterval timestep = MIN(renderTime - _lastRenderTime, 1.0 / 60.0);  // Cap at 60fps to prevent huge jumps
    _sceneTime += timestep;

    // Get the current rendering layout configuration
    cp_layer_renderer_configuration_t layerConfiguration = cp_layer_renderer_get_configuration(_layerRenderer);
    cp_layer_renderer_layout layout = cp_layer_renderer_configuration_get_layout(layerConfiguration);

    // === 3D OBJECT POSITIONING AND ANIMATION ===
    // Position the globe at a comfortable viewing distance and height
#if TARGET_OS_SIMULATOR
    const float estimatedHeadHeight = 0.0;  // Simulator doesn't need head height adjustment
#else
    const float estimatedHeadHeight = 1.25;  // ~4 feet - comfortable standing height
#endif

    // Create animated rotation around Y-axis using time-based trigonometry
    float c = cos(_sceneTime * 0.5f);  // Cosine component for rotation
    float s = sin(_sceneTime * 0.5f);  // Sine component for rotation
    
    // Build 4x4 transformation matrix combining rotation and translation
    // This is a standard Model matrix in the Model-View-Projection pipeline
    simd_float4x4 modelTransform = simd_matrix(
        simd_make_float4(   c, 0.0f,    -s, 0.0f),  // X-axis rotated by angle
        simd_make_float4(0.0f, 1.0f,  0.0f, 0.0f),  // Y-axis unchanged (rotation axis)
        simd_make_float4(   s, 0.0f,     c, 0.0f),  // Z-axis rotated by angle
        simd_make_float4(0.0f, estimatedHeadHeight, -1.5f, 1.0f)  // Translation: 1.5m forward, head height
    );
    _globeMesh->setModelMatrix(modelTransform);

    // === MIXED REALITY PORTAL CONFIGURATION ===
    // Configure environment visibility based on immersion mode
    if (_configuration.immersionStyle == SRImmersionStyleMixed) {
        // Mixed mode: Create a "portal" effect by limiting environment visibility
        // This allows real-world passthrough to show through at the edges
        _environmentMesh->setCutoffAngle(_configuration.portalCutoffAngle);
    } else {
        // Full immersion: Show complete 360° environment
        _environmentMesh->setCutoffAngle(180);
    }

    // === GPU COMMAND BUFFER CREATION ===
    // Create a command buffer to batch all our rendering commands
    // This gets submitted to the GPU as a single unit of work
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];

    // === STEREOSCOPIC VIEW SETUP ===
    // Vision Pro renders to both eyes simultaneously for 3D depth perception
    size_t viewCount = cp_drawable_get_view_count(drawable);  // Typically 2 (left + right eye)

    // Pre-calculate view matrices and viewports for both eyes
    std::array<MTLViewport, 2> viewports {};
    std::array<PoseConstants, 2> poseConstants {};
    std::array<PoseConstants, 2> poseConstantsForEnvironment {};
    
    for (int i = 0; i < viewCount; ++i) {
        // Get the screen region (viewport) for this eye
        viewports[i] = viewportForViewIndex(drawable, i);

        // Calculate View and Projection matrices for this eye's camera
        // These transform 3D world coordinates to 2D screen coordinates
        poseConstants[i] = poseConstantsForViewIndex(drawable, i);

        // Special case for environment: remove translation component
        // This makes the environment appear infinitely far away (like a skybox)
        poseConstantsForEnvironment[i] = poseConstantsForViewIndex(drawable, i);
        poseConstantsForEnvironment[i].viewMatrix.columns[3] = simd_make_float4(0.0, 0.0, 0.0, 1.0);
    }

    // === RENDERING MODE SELECTION ===
    // Choose between two rendering strategies based on hardware capabilities
    
    if (layout == cp_layer_renderer_layout_dedicated) {
        // === DEDICATED LAYOUT: SEPARATE RENDER PASS PER EYE ===
        // Traditional approach: render left eye, then right eye
        // Simpler but less efficient - requires more GPU memory bandwidth
        
        for (int i = 0; i < viewCount; ++i) {
            // Create render pass for this specific eye
            MTLRenderPassDescriptor *renderPassDescriptor = createRenderPassDescriptor(drawable, i);
            id<MTLRenderCommandEncoder> renderCommandEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

            // === 3D GRAPHICS STATE SETUP ===
            [renderCommandEncoder setCullMode:MTLCullModeBack];  // Don't render back-facing triangles (optimization)

            // === ENVIRONMENT RENDERING (Background Layer) ===
            // Render the 360° environment sphere first (background)
            [renderCommandEncoder setFrontFacingWinding:MTLWindingClockwise];    // Environment sphere has inverted normals
            [renderCommandEncoder setDepthStencilState:_backgroundDepthStencilState];  // Use far depth values
            [renderCommandEncoder setRenderPipelineState:_environmentRenderPipelineState];  // Use environment shaders
            _environmentMesh->draw(renderCommandEncoder, &poseConstantsForEnvironment[i], 1);

            // === CONTENT RENDERING (Foreground Objects) ===
            // Render the 3D globe and any other foreground objects
            [renderCommandEncoder setFrontFacingWinding:MTLWindingCounterClockwise];  // Standard winding order
            [renderCommandEncoder setDepthStencilState:_contentDepthStencilState];    // Standard depth testing
            [renderCommandEncoder setRenderPipelineState:_contentRenderPipelineState];  // Use content shaders
            _globeMesh->draw(renderCommandEncoder, &poseConstants[i], 1);

            [renderCommandEncoder endEncoding];
        }
    } else {
        // === LAYERED LAYOUT: SINGLE PASS WITH VERTEX AMPLIFICATION ===
        // Advanced technique: render both eyes in a single pass
        // More efficient - reduces vertex processing workload and memory bandwidth
        
        MTLRenderPassDescriptor *renderPassDescriptor = createRenderPassDescriptor(drawable, 0);
        id<MTLRenderCommandEncoder> renderCommandEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];

        // === MULTI-VIEW RENDERING SETUP ===
        // Configure the GPU to output to both eye viewports simultaneously
        [renderCommandEncoder setViewports:viewports.data() count:viewCount];
        [renderCommandEncoder setVertexAmplificationCount:viewCount viewMappings:nil];
        // Vertex amplification: Each vertex shader execution produces output for both eyes

        [renderCommandEncoder setCullMode:MTLCullModeBack];

        // === ENVIRONMENT RENDERING ===
        [renderCommandEncoder setFrontFacingWinding:MTLWindingClockwise];
        [renderCommandEncoder setDepthStencilState:_backgroundDepthStencilState];
        [renderCommandEncoder setRenderPipelineState:_environmentRenderPipelineState];
        _environmentMesh->draw(renderCommandEncoder, poseConstantsForEnvironment.data(), viewCount);

        // === CONTENT RENDERING ===
        [renderCommandEncoder setFrontFacingWinding:MTLWindingCounterClockwise];
        [renderCommandEncoder setDepthStencilState:_contentDepthStencilState];
        [renderCommandEncoder setRenderPipelineState:_contentRenderPipelineState];
        _globeMesh->draw(renderCommandEncoder, poseConstants.data(), viewCount);

        [renderCommandEncoder endEncoding];
    }

    // === FRAME PRESENTATION ===
    // Tell CompositorServices to display the rendered frame
    // This integrates with Vision Pro's display pipeline and handles timing
    cp_drawable_encode_present(drawable, commandBuffer);

    // === GPU COMMAND SUBMISSION ===
    // Submit all batched commands to the GPU for execution
    [commandBuffer commit];
}

MTLRenderPassDescriptor* SpatialRenderer::createRenderPassDescriptor(cp_drawable_t drawable, size_t index) {
    cp_layer_renderer_configuration_t layerConfiguration = cp_layer_renderer_get_configuration(_layerRenderer);
    cp_layer_renderer_layout layout = cp_layer_renderer_configuration_get_layout(layerConfiguration);

    MTLRenderPassDescriptor *passDescriptor = [[MTLRenderPassDescriptor alloc] init];

    passDescriptor.colorAttachments[0].texture = cp_drawable_get_color_texture(drawable, index);
    if (_configuration.immersionStyle == SRImmersionStyleMixed) {
        passDescriptor.colorAttachments[0].loadAction = MTLLoadActionClear;
        passDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(0.0, 0.0, 0.0, 0.0);
    }
    passDescriptor.colorAttachments[0].storeAction = MTLStoreActionStore;

    passDescriptor.depthAttachment.texture = cp_drawable_get_depth_texture(drawable, index);
    passDescriptor.depthAttachment.loadAction = MTLLoadActionClear;
    passDescriptor.depthAttachment.clearDepth = 0.0;
    passDescriptor.depthAttachment.storeAction = MTLStoreActionStore;

    switch (layout) {
        case cp_layer_renderer_layout_layered:
            passDescriptor.renderTargetArrayLength = cp_drawable_get_view_count(drawable);
            break;
        case cp_layer_renderer_layout_shared:
            // Even though we don't use an array texture as the render target in "shared" layout, we're
            // obligated to set the render target array length because the index is set by the vertex shader.
            passDescriptor.renderTargetArrayLength = 1;
            break;
        case cp_layer_renderer_layout_dedicated:
            break;
    }

    if (cp_drawable_get_rasterization_rate_map_count(drawable) > 0) {
        passDescriptor.rasterizationRateMap = cp_drawable_get_rasterization_rate_map(drawable, index);
    }

    return passDescriptor;
}

MTLViewport SpatialRenderer::viewportForViewIndex(cp_drawable_t drawable, size_t index) {
    cp_view_t view = cp_drawable_get_view(drawable, index);
    cp_view_texture_map_t texture_map = cp_view_get_view_texture_map(view);
    return cp_view_texture_map_get_viewport(texture_map);
}

PoseConstants SpatialRenderer::poseConstantsForViewIndex(cp_drawable_t drawable, size_t index) {
    PoseConstants outPose;

    // === DEVICE TRACKING AND HEAD POSE ===
    // Get the current tracked position/orientation of the Vision Pro headset
    // This anchor represents where the user's head is in 3D space
    ar_device_anchor_t anchor = cp_drawable_get_device_anchor(drawable);
    
    // Convert anchor to 4x4 transformation matrix
    // This transforms from device coordinate space to world coordinate space
    simd_float4x4 poseTransform = ar_anchor_get_origin_from_anchor_transform(anchor);

    // Get the view information for this specific eye (left=0, right=1)
    cp_view_t view = cp_drawable_get_view(drawable, index);

    // === PROJECTION MATRIX CALCULATION ===
    // The projection matrix converts from 3D camera space to 2D screen space
    // It handles perspective projection and maps to the correct viewport
    
    if (@available(visionOS 2.0, *)) {
        // Modern API: Use built-in projection calculation
        // This handles field-of-view, aspect ratio, and near/far planes automatically
        outPose.projectionMatrix = cp_drawable_compute_projection(drawable,
                                                                  cp_axis_direction_convention_right_up_back,
                                                                  index);
    } else {
        // Legacy API: Manual projection calculation for older visionOS versions
        
        // Get field-of-view tangents for this eye
        // Tangents define the viewing frustum (pyramid) in normalized coordinates
        simd_float4 tangents = cp_view_get_tangents(view);
        
        // Get depth range (near and far clipping planes)
        simd_float2 depth_range = cp_drawable_get_depth_range(drawable);
        
        // Create perspective projection using field-of-view tangents
        // This is the mathematical formula that creates the perspective effect
        SPProjectiveTransform3D projectiveTransform = SPProjectiveTransform3DMakeFromTangents(
            tangents[0],        // Left edge tangent
            tangents[1],        // Right edge tangent  
            tangents[2],        // Top edge tangent
            tangents[3],        // Bottom edge tangent
            depth_range[1],     // Near clipping plane distance
            depth_range[0],     // Far clipping plane distance
            true                // Use reversed Z-buffer for better precision
        );
        
        // Convert from double precision to single precision for GPU efficiency
        outPose.projectionMatrix = matrix_float4x4_from_double4x4(projectiveTransform.matrix);
    }

    // === VIEW MATRIX CALCULATION ===
    // The view matrix transforms from world space to camera space
    // It positions the virtual camera for this eye
    
    // Combine head tracking with per-eye offset
    // poseTransform: where the head is in world space
    // cp_view_get_transform(view): offset from head center to this eye
    simd_float4x4 cameraMatrix = simd_mul(poseTransform, cp_view_get_transform(view));
    
    // Invert to get view matrix (camera-to-world becomes world-to-camera)
    // This is the standard convention in 3D graphics
    outPose.viewMatrix = simd_inverse(cameraMatrix);
    
    return outPose;
}
