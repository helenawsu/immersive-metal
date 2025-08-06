#include "SpatialRenderer.h"
#include "Mesh.h"
#include "ShaderTypes.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <Spatial/Spatial.h>

#include <vector>
#include <array>
#include <cstdlib>  // For rand() and RAND_MAX

// === CONFIGURABLE INSTANCE COUNT ===
// Change this value to render any number of boxes you want!
static const int NUM_INSTANCES = 50000;  // Easy to change - just modify this number!

// === PLACEMENT CONFIGURATION ===
static const bool CIRCULAR_PLACEMENT = true;  // Set to true for 360° box placement

// === HAND REPULSION SETTINGS ===
static const float REPULSION_RADIUS = 0.3f;    // Radius around hands that affects particles (30cm)
static const float REPULSION_STRENGTH = 5.0f;  // How strong the repulsion force is

// Helper function to create translation matrix
static simd_float4x4 simd_matrix4x4_translation(simd_float3 translation) {
    return simd_matrix(
        simd_make_float4(1.0f, 0.0f, 0.0f, 0.0f),
        simd_make_float4(0.0f, 1.0f, 0.0f, 0.0f),
        simd_make_float4(0.0f, 0.0f, 1.0f, 0.0f),
        simd_make_float4(translation.x, translation.y, translation.z, 1.0f)
    );
}

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

void SpatialRenderer::updateHandPositions(const std::vector<simd_float3>& handPositions) {
    // === THREAD-SAFE HAND POSITION UPDATE ===
    // Update the internal hand positions vector with new data from ARKit
    _handPositions = handPositions;
    
    // Debug logging
    if (!handPositions.empty()) {
        NSLog(@"SpatialRenderer: Updated with %zu hand positions", handPositions.size());
        for (size_t i = 0; i < handPositions.size(); ++i) {
            const simd_float3& pos = handPositions[i];
            NSLog(@"  Hand %zu: (%.3f, %.3f, %.3f)", i, pos.x, pos.y, pos.z);
        }
    }
}

void SpatialRenderer::makeResources() {
    // === 3D GEOMETRY CREATION ===
    // Create a Metal buffer allocator to manage GPU memory for mesh data
    MTKMeshBufferAllocator *bufferAllocator = [[MTKMeshBufferAllocator alloc] initWithDevice:_device];
    
    // === INSTANCED RENDERING SETUP ===
    // Instead of creating separate mesh objects, we create ONE mesh
    // and render it multiple times with different transform matrices
    
    // Create ONE small box mesh for particles (all instances will use this same geometry)
    MDLMesh *boxMesh = [MDLMesh newBoxWithDimensions:simd_make_float3(0.005, 0.005, 0.005)
                                             segments:simd_make_uint3(1, 1, 1)
                                         geometryType:MDLGeometryTypeTriangles
                                        inwardNormals:NO
                                            allocator:bufferAllocator];
    
    // Create the single mesh object that all instances will share
    _boxMesh = std::make_unique<TexturedMesh>(boxMesh, @"iridescent.jpg", _device);
    
    // Create a LARGER box mesh specifically for glow effect
    MDLMesh *glowBoxMesh = [MDLMesh newBoxWithDimensions:simd_make_float3(0.1, 0.1, 0.1)
                                                segments:simd_make_uint3(1, 1, 1)
                                            geometryType:MDLGeometryTypeTriangles
                                           inwardNormals:NO
                                               allocator:bufferAllocator];
    
    // Create the glow mesh object (20x larger than particles)
    _glowMesh = std::make_unique<TexturedMesh>(glowBoxMesh, @"iridescent.jpg", _device);
    
    // === INSTANCE DATA SETUP ===
    // Create CPU array to hold transform matrices for each instance
    _instanceTransforms.resize(NUM_INSTANCES);
    

    // === GPU PARTICLE SYSTEM INITIALIZATION ===
    // Create GPU buffers for particle data - much more efficient than CPU vectors
    _particlePositionsBuffer = [_device newBufferWithLength:sizeof(simd_float3) * NUM_INSTANCES 
                                                     options:MTLResourceStorageModeShared];
    _particleVelocitiesBuffer = [_device newBufferWithLength:sizeof(simd_float3) * NUM_INSTANCES 
                                                      options:MTLResourceStorageModeShared];
    _particleConstantsBuffer = [_device newBufferWithLength:sizeof(ParticleConstants) 
                                                    options:MTLResourceStorageModeShared];
    
    // Initialize particle positions and velocities directly in GPU buffers
    simd_float3 *positions = (simd_float3 *)[_particlePositionsBuffer contents];
    simd_float3 *velocities = (simd_float3 *)[_particleVelocitiesBuffer contents];
    
    // === HAND TRACKING INITIALIZATION ===
    _handPositions.clear();  // Start with no hands detected
    // Create buffer for up to 2 hands (left + right)
    _handPositionsBuffer = [_device newBufferWithLength:sizeof(simd_float3) * 2 
                                                 options:MTLResourceStorageModeShared];
    
    for (int i = 0; i < NUM_INSTANCES; ++i) {
        float x = -0.5f + (static_cast<float>(rand()) / RAND_MAX) * 1.0f;  // Random between -0.5 and 0.5
        float y = 0.5 + (static_cast<float>(rand()) / RAND_MAX) * 1.0f;    // Random between 0.5 and 1.5
        float z = -0.5f + (static_cast<float>(rand()) / RAND_MAX) * 1.0f;  // Random between -0.5 and 0.5
        positions[i] = simd_make_float3(x, y, z);
        
        x = -1.0f + (static_cast<float>(rand()) / RAND_MAX) * 2.0f;  // Random between -1 and 1
        y = (-1.0f + (static_cast<float>(rand()) / RAND_MAX)) * 2.0f;
        z = -1.0f + (static_cast<float>(rand()) / RAND_MAX) * 2.0f;  // Random between -1 and 1
        velocities[i] = simd_make_float3(x, y, z);
    }

    
    // Create GPU buffer to hold the transform matrices
    // This buffer gets updated each frame and sent to the vertex shader
    _instanceBuffer = [_device newBufferWithLength:sizeof(simd_float4x4) * NUM_INSTANCES 
                                            options:MTLResourceStorageModeShared];

    // === GLOW EFFECT CONSTANTS BUFFER ===
    // Create GPU buffer for glow effect parameters
    _glowConstantsBuffer = [_device newBufferWithLength:sizeof(GlowConstants)
                                                options:MTLResourceStorageModeShared];
    
    // Initialize glow constants with default values
    GlowConstants *glowConstants = (GlowConstants *)[_glowConstantsBuffer contents];
    glowConstants->glowScale = 5.0f;                                    // Scale glow 1.5x larger than particles
    glowConstants->glowIntensity = 3.8f;                               // 80% intensity
    glowConstants->glowColor = simd_make_float3(0.3f, 0.7f, 1.0f);    // Cyan-blue glow
    glowConstants->glowFalloff = 1.5f;                                 // Quadratic falloff

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
        // === MAIN CONTENT PIPELINE WITH ALPHA BLENDING FOR GLOW ===
        
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
        pipelineDescriptor.vertexDescriptor = _boxMesh->vertexDescriptor();
        
        // === ENABLE ALPHA BLENDING FOR RADIAL GLOW FALLOFF ===
        pipelineDescriptor.colorAttachments[0].blendingEnabled = YES;
        
        // Alpha blending: new_color = src_color * src_alpha + dst_color * (1 - src_alpha)
        pipelineDescriptor.colorAttachments[0].rgbBlendOperation = MTLBlendOperationAdd;
        pipelineDescriptor.colorAttachments[0].alphaBlendOperation = MTLBlendOperationAdd;
        
        // Use source alpha for transparency control
        pipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        pipelineDescriptor.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
        
        // Destination blending for transparency
        pipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        pipelineDescriptor.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        
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
        
        // Reset blending state for subsequent pipelines
        pipelineDescriptor.colorAttachments[0].blendingEnabled = NO;
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
    
    {
        // === GLOW EFFECT PIPELINE ===
        // Creates additive glow effects using scaled geometry instances
        
        // Select appropriate glow shaders based on rendering mode
        vertexFunction = [library newFunctionWithName: layoutIsDedicated ? @"vertex_dedicated_glow_main" : @"vertex_glow_main"
                                       constantValues:functionConstants
                                                error:&error];
        fragmentFunction = [library newFunctionWithName:@"fragment_glow"];
        
        pipelineDescriptor.vertexFunction = vertexFunction;
        pipelineDescriptor.fragmentFunction = fragmentFunction;
        
        // Use same vertex descriptor as content (both are box meshes)
        pipelineDescriptor.vertexDescriptor = _boxMesh->vertexDescriptor();
        
        // === ADDITIVE BLENDING SETUP ===
        // Enable blending for glow effect accumulation
        pipelineDescriptor.colorAttachments[0].blendingEnabled = YES;
        
        // Additive blending: new_color = src_color * src_alpha + dst_color * 1
        // This allows multiple glow effects to accumulate naturally
        pipelineDescriptor.colorAttachments[0].rgbBlendOperation = MTLBlendOperationAdd;
        pipelineDescriptor.colorAttachments[0].alphaBlendOperation = MTLBlendOperationAdd;
        
        // Source: Use source alpha for intensity control
        pipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        pipelineDescriptor.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorSourceAlpha;
        
        // Destination: Add to existing color (accumulate glow)
        pipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOne;
        pipelineDescriptor.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
        
        if (!layoutIsDedicated) {
            pipelineDescriptor.inputPrimitiveTopology = MTLPrimitiveTopologyClassTriangle;
            pipelineDescriptor.maxVertexAmplificationCount = 2;
        }

        // Create the glow pipeline state
        _glowRenderPipelineState = [_device newRenderPipelineStateWithDescriptor:pipelineDescriptor error:&error];
        if (_glowRenderPipelineState == nil) {
            NSLog(@"Error occurred when creating glow render pipeline state: %@", error);
        }
        
        // Reset blending state for subsequent pipelines
        pipelineDescriptor.colorAttachments[0].blendingEnabled = NO;
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
    
    // === GLOW DEPTH STATE ===
    // Glow effects render behind solid particles with depth testing but no depth writes
    depthDescriptor.depthWriteEnabled = NO;   // Don't write to depth buffer (allows overlapping glow)
    depthDescriptor.depthCompareFunction = MTLCompareFunctionLessEqual;  // Render behind or at same depth as particles
    _glowDepthStencilState = [_device newDepthStencilStateWithDescriptor:depthDescriptor];
    
    // === PARTICLE COMPUTE PIPELINE ===
    // Create compute pipeline for GPU particle physics
    id<MTLFunction> particleComputeFunction = [library newFunctionWithName:@"updateParticles"];
    if (particleComputeFunction != nil) {
        _particleComputePipelineState = [_device newComputePipelineStateWithFunction:particleComputeFunction error:&error];
        if (_particleComputePipelineState == nil) {
            NSLog(@"Error creating particle compute pipeline: %@", error);
        }
    } else {
        NSLog(@"Error: Could not find updateParticles compute function in shader library");
    }
}

void SpatialRenderer::drawAndPresent(cp_frame_t frame, cp_drawable_t drawable) {
    // === FRAME TIMING AND ANIMATION ===
    // Calculate time delta for smooth animation independent of frame rate
    CFTimeInterval renderTime = CACurrentMediaTime();
    CFTimeInterval timestep = MIN(renderTime - _lastRenderTime, 1.0 / 60.0);  // Cap at 60fps to prevent huge jumps
    _sceneTime += timestep;
    _lastRenderTime = renderTime;

    // === GET HEAD POSITION ===
    // Extract the head position for particle center attraction
    ar_device_anchor_t anchor = cp_drawable_get_device_anchor(drawable);
    simd_float4x4 poseTransform = ar_anchor_get_origin_from_anchor_transform(anchor);
    simd_float3 headPosition = simd_make_float3(poseTransform.columns[3].x, 
                                                poseTransform.columns[3].y, 
                                                poseTransform.columns[3].z);

    // Get the current rendering layout configuration
    cp_layer_renderer_configuration_t layerConfiguration = cp_layer_renderer_get_configuration(_layerRenderer);
    cp_layer_renderer_layout layout = cp_layer_renderer_configuration_get_layout(layerConfiguration);

    // === GPU COMMAND BUFFER CREATION ===
    // Create a command buffer to batch all our rendering commands
    // This gets submitted to the GPU as a single unit of work
    id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];

    // === GPU PARTICLE PHYSICS COMPUTE ===
    // Replace CPU loop with GPU compute shader for massive parallelization
    
    // Update hand positions in GPU buffer
    if (!_handPositions.empty()) {
        simd_float3 *gpuHandPositions = (simd_float3 *)[_handPositionsBuffer contents];
        for (size_t i = 0; i < std::min(_handPositions.size(), size_t(2)); ++i) {
            gpuHandPositions[i] = _handPositions[i];
        }
    }
    
    // Prepare particle physics constants
    ParticleConstants *constants = (ParticleConstants *)[_particleConstantsBuffer contents];
    constants->headPosition = headPosition;
    constants->handCount = static_cast<uint32_t>(_handPositions.size());
    constants->deltaTime = timestep;
    constants->repulsionRadius = REPULSION_RADIUS;
    constants->repulsionStrength = REPULSION_STRENGTH;
    constants->driftStrength = 0.1f;
    constants->damping = 0.98f;
    constants->centerPullStrength = 0.01f;
    constants->boundary = 3.0f;
    
    // Create compute command encoder for particle physics
    id<MTLComputeCommandEncoder> computeEncoder = [commandBuffer computeCommandEncoder];
    [computeEncoder setComputePipelineState:_particleComputePipelineState];
    
    // Bind GPU buffers to compute shader
    [computeEncoder setBuffer:_particlePositionsBuffer offset:0 atIndex:0];
    [computeEncoder setBuffer:_particleVelocitiesBuffer offset:0 atIndex:1];
    [computeEncoder setBuffer:_handPositionsBuffer offset:0 atIndex:2];
    [computeEncoder setBuffer:_particleConstantsBuffer offset:0 atIndex:3];
    [computeEncoder setBuffer:_instanceBuffer offset:0 atIndex:4];  // Add instance transforms buffer
    
    // Calculate optimal thread group size for GPU
    NSUInteger threadsPerGroup = _particleComputePipelineState.maxTotalThreadsPerThreadgroup;
    threadsPerGroup = std::min(threadsPerGroup, NSUInteger(256)); // Cap at 256 for better occupancy
    
    MTLSize threadsPerThreadgroup = MTLSizeMake(threadsPerGroup, 1, 1);
    MTLSize threadgroupsPerGrid = MTLSizeMake((NUM_INSTANCES + threadsPerGroup - 1) / threadsPerGroup, 1, 1);
    
    // Dispatch GPU compute - processes ALL particles in parallel!
    [computeEncoder dispatchThreadgroups:threadgroupsPerGrid threadsPerThreadgroup:threadsPerThreadgroup];
    [computeEncoder endEncoding];
    
    // === EXPLICIT GPU SYNCHRONIZATION ===
    // Add memory barrier to ensure compute writes are visible to vertex shader
    // This is more explicit and can help with debugging/profiling
    id<MTLBlitCommandEncoder> blitEncoder = [commandBuffer blitCommandEncoder];
    [blitEncoder endEncoding];  // Empty blit encoder acts as memory barrier
    
    // === GPU PARTICLE PHYSICS AND TRANSFORM GENERATION COMPLETE ===
    // GPU computes both particle physics AND generates transform matrices
    // Memory barrier ensures vertex shader sees the updated transforms
    // No CPU-GPU roundtrip needed!

    // === MIXED REALITY PORTAL CONFIGURATION ===
    // Configure environment visibility based on immersion mode
    if (_configuration.immersionStyle == SRImmersionStyleMixed) {
        // Mixed mode: Create a "portal" effect by limiting environment visibility
        // This allows real-world passthrough to show through at the edges
        _environmentMesh->setCutoffAngle(_configuration.portalCutoffAngle);
    } else {
        // Full immersion: Show complete 360° environment with no cutoff
        // Set to 180 degrees to show the full hemisphere (no edge clipping)
        _environmentMesh->setCutoffAngle(180);
    }

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

            // CONTENT RENDERING MOVED AFTER GLOW - see above

            // === CONTENT RENDERING WITH BUILT-IN GLOW ===
            // Render particles with radial falloff glow effect built into fragment shader
            [renderCommandEncoder setFrontFacingWinding:MTLWindingCounterClockwise];  // Standard winding order
            [renderCommandEncoder setDepthStencilState:_contentDepthStencilState];    // Standard depth testing
            [renderCommandEncoder setRenderPipelineState:_contentRenderPipelineState];  // Use content shaders
            
            // Single instanced draw call for all boxes with built-in glow
            _boxMesh->drawInstanced(renderCommandEncoder, &poseConstants[i], 1, _instanceBuffer, NUM_INSTANCES);

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

        // CONTENT RENDERING MOVED AFTER GLOW - see above

        // === CONTENT RENDERING WITH BUILT-IN GLOW ===
        // Render particles with radial falloff glow effect built into fragment shader
        [renderCommandEncoder setFrontFacingWinding:MTLWindingCounterClockwise];
        [renderCommandEncoder setDepthStencilState:_contentDepthStencilState];
        [renderCommandEncoder setRenderPipelineState:_contentRenderPipelineState];
        
        // Single instanced draw call for all boxes with built-in glow (both eyes simultaneously)
        _boxMesh->drawInstanced(renderCommandEncoder, poseConstants.data(), viewCount, _instanceBuffer, NUM_INSTANCES);

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
