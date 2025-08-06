#pragma once

#include "Mesh.h"
#include "ShaderTypes.h"
#include "SpatialRenderingEngine.h"

#include <memory>
#include <vector>

#import <CompositorServices/CompositorServices.h>
#import <Metal/Metal.h>

// === MAIN 3D RENDERER FOR VISION PRO ===
// Orchestrates the complete 3D graphics pipeline using Metal
class SpatialRenderer {
public:
    SpatialRenderer(cp_layer_renderer_t layerRenderer, SRConfiguration *configuration);

    void drawAndPresent(cp_frame_t frame, cp_drawable_t drawable);
    void updateHandPositions(const std::vector<simd_float3>& handPositions);

private:
    void makeResources();
    void makeRenderPipelines();
    MTLRenderPassDescriptor* createRenderPassDescriptor(cp_drawable_t drawable, size_t index);
    MTLViewport viewportForViewIndex(cp_drawable_t drawable, size_t index);
    PoseConstants poseConstantsForViewIndex(cp_drawable_t drawable, size_t index);

    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLRenderPipelineState> _environmentRenderPipelineState;
    id<MTLRenderPipelineState> _contentRenderPipelineState;
    id<MTLDepthStencilState> _contentDepthStencilState;
    id<MTLDepthStencilState> _backgroundDepthStencilState;
    
    // === COMPUTE SHADER PIPELINE FOR PARTICLE PHYSICS ===
    id<MTLComputePipelineState> _particleComputePipelineState;
    
    cp_layer_renderer_t _layerRenderer;
    
    // === SIMPLE INSTANCED RENDERING ===
    // Single mesh rendered multiple times with different transforms
    std::unique_ptr<TexturedMesh> _boxMesh;        // ONE mesh for all instances
    id<MTLBuffer> _instanceBuffer;                 // GPU buffer with transform matrices
    std::vector<simd_float4x4> _instanceTransforms; // CPU array with transforms
    
    // === GPU PARTICLE SYSTEM ===
    id<MTLBuffer> _particlePositionsBuffer;        // GPU buffer for particle positions
    id<MTLBuffer> _particleVelocitiesBuffer;       // GPU buffer for particle velocities
    id<MTLBuffer> _handPositionsBuffer;            // GPU buffer for hand positions
    id<MTLBuffer> _particleConstantsBuffer;        // GPU buffer for physics constants
    
    // === HAND TRACKING ===
    std::vector<simd_float3> _handPositions;       // Current hand positions for repulsion
    
    std::unique_ptr<SpatialEnvironmentMesh> _environmentMesh;
    SRConfiguration *_configuration;
    CFTimeInterval _sceneTime;
    CFTimeInterval _lastRenderTime;
};
