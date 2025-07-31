
#include <metal_stdlib>
using namespace metal;

constant bool useLayeredRendering [[function_constant(0)]];

struct VertexIn {
    float3 position  [[attribute(0)]];
    float3 normal    [[attribute(1)]];
    float2 texCoords [[attribute(2)]];
};

struct VertexOut {
    float4 position [[position]];
    float3 viewNormal;
    float2 texCoords;
};

struct LayeredVertexOut {
    float4 position [[position]];
    float3 viewNormal;
    float2 texCoords;
    uint renderTargetIndex [[render_target_array_index]];
    uint viewportIndex [[viewport_array_index]];
};

struct FragmentIn {
    float4 position [[position]];
    float3 viewNormal;
    float2 texCoords;
    uint renderTargetIndex [[render_target_array_index]];
    uint viewportIndex [[viewport_array_index]];
};

struct PoseConstants {
    float4x4 projectionMatrix;
    float4x4 viewMatrix;
};

struct InstanceConstants {
    float4x4 modelMatrix;
};

[[vertex]]
LayeredVertexOut vertex_main(VertexIn in [[stage_in]],
                             constant PoseConstants *poses [[buffer(1)]],
                             constant InstanceConstants &instance [[buffer(2)]],
                             uint amplificationID [[amplification_id]])
{
    // === VERTEX AMPLIFICATION SETUP ===
    // amplificationID tells us which eye we're rendering for:
    // 0 = left eye, 1 = right eye
    // With vertex amplification, this shader runs once but outputs to both eyes
    constant auto &pose = poses[amplificationID];
    
    LayeredVertexOut out;
    
    // === CORE 3D TRANSFORMATION PIPELINE (MVP) ===
    // This is the heart of 3D graphics: transforming a vertex from object space to screen space
    // The transformations happen in this specific order:
    
    // Step 1: MODEL TRANSFORMATION
    // instance.modelMatrix transforms from local object space to world space
    // This positions, rotates, and scales the object in the 3D world
    
    // Step 2: VIEW TRANSFORMATION  
    // pose.viewMatrix transforms from world space to camera space
    // This positions the virtual camera and determines what we're looking at
    
    // Step 3: PROJECTION TRANSFORMATION
    // pose.projectionMatrix transforms from camera space to screen space
    // This creates the perspective effect and maps 3D coordinates to 2D screen
    
    out.position = pose.projectionMatrix *           // 3. 3D → 2D screen projection
                   pose.viewMatrix *                 // 2. World → Camera transformation
                   instance.modelMatrix *            // 1. Object → World transformation
                   float4(in.position, 1.0f);       // Input vertex (w=1 for points)
    
    // === NORMAL VECTOR TRANSFORMATION ===
    // Surface normals are used for lighting calculations
    // They need special transformation because they're vectors (not points)
    // Note: w=0.0 because normals are directions, not positions
    out.viewNormal = (pose.viewMatrix * instance.modelMatrix * float4(in.normal, 0.0f)).xyz;
    
    // === TEXTURE COORDINATE PROCESSING ===
    // Pass through texture coordinates for fragment shader to sample textures
    out.texCoords = in.texCoords;
    
    // Flip horizontally to match ModelIO's coordinate system
    // Different 3D modeling tools use different UV conventions
    out.texCoords.x = 1.0f - out.texCoords.x;
    
    // === MULTI-VIEW OUTPUT ROUTING ===
    // Tell the GPU which eye's render target to output to
    if (useLayeredRendering) {
        // Layered: Output to texture array slice (0=left eye, 1=right eye)
        out.renderTargetIndex = amplificationID;
    }
    
    // Set viewport index for viewport array (used in shared/layered modes)
    out.viewportIndex = amplificationID;
    
    return out;
}

[[vertex]]
VertexOut vertex_dedicated_main(VertexIn in [[stage_in]],
                                constant PoseConstants *poses [[buffer(1)]],
                                constant InstanceConstants &instance [[buffer(2)]])
{
    constant auto &pose = poses[0];
    
    VertexOut out;
    out.position = pose.projectionMatrix * pose.viewMatrix * instance.modelMatrix * float4(in.position, 1.0f);
    out.viewNormal = (pose.viewMatrix * instance.modelMatrix * float4(in.normal, 0.0f)).xyz;
    out.texCoords = in.texCoords;
    out.texCoords.x = 1.0f - out.texCoords.x; // Flip uvs horizontally to match Model I/O
    return out;
}

[[fragment]]
half4 fragment_main(FragmentIn in [[stage_in]],
                    texture2d<half, access::sample> texture [[texture(0)]])
{
    // === TEXTURE SAMPLING ===
    // Create a sampler that defines how to read the texture
    // - normalized coordinates: UV coords are [0,1] instead of pixel coordinates  
    // - linear filtering: smooth interpolation between pixels for better quality
    // - no mip filtering: not using mip maps in this simple example
    // - repeat addressing: UV coords outside [0,1] wrap around
    constexpr sampler environmentSampler(coord::normalized, filter::linear, mip_filter::none, address::repeat);
    
    // Sample the diffuse texture at the interpolated UV coordinates
    // This reads the color from the texture and returns it as the final pixel color
    // The texture coordinates were set up by the vertex shader and interpolated across the triangle
    half4 color = texture.sample(environmentSampler, in.texCoords);
    
    return color;
}
