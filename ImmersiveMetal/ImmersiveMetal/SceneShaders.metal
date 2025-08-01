
#include <metal_stdlib>
using namespace metal;

// Vision Pro rendering mode selection (set by CPU)
constant bool useLayeredRendering [[function_constant(0)]];

// === VERTEX INPUT: CPU → GPU Data ===
struct VertexIn {
    float3 position  [[attribute(0)]]; // 3D position in local space
    float3 normal    [[attribute(1)]]; // Surface normal vector
    float2 texCoords [[attribute(2)]]; // UV texture coordinates
};

// === VERTEX OUTPUT: Dedicated Rendering ===
struct VertexOut {
    float4 position [[position]];      // Screen position (clip space)
    float3 viewNormal;                 // Normal in camera space
    float2 texCoords;                  // UV coordinates
};

// === VERTEX OUTPUT: Layered Rendering (Stereo) ===
struct LayeredVertexOut {
    float4 position [[position]];
    float3 viewNormal;
    float2 texCoords;
    uint renderTargetIndex [[render_target_array_index]]; // Which eye (0/1)
    uint viewportIndex [[viewport_array_index]];          // Viewport routing
};

// === FRAGMENT INPUT: Interpolated Vertex Data ===
struct FragmentIn {
    float4 position [[position]];     // Pixel screen position
    float3 viewNormal;                // Interpolated normal
    float2 texCoords;                 // Interpolated UV coordinates
    uint renderTargetIndex [[render_target_array_index]];
    uint viewportIndex [[viewport_array_index]];
};

// === GPU TRANSFORMATION DATA ===
struct PoseConstants {
    float4x4 projectionMatrix;        // Camera → Screen transformation
    float4x4 viewMatrix;              // World → Camera transformation
};

// Removed InstanceConstants struct - now using direct float4x4 array for better performance

// === VERTEX SHADER: LAYERED RENDERING WITH TRUE GPU INSTANCING ===
// Renders multiple instances of the same mesh with different transform matrices
// This version uses a single draw call with instanceCount > 1
[[vertex]]
LayeredVertexOut vertex_main(VertexIn in [[stage_in]],
                             constant PoseConstants *poses [[buffer(1)]],
                             constant float4x4 *instanceMatrices [[buffer(2)]], // Array of instance transform matrices
                             uint amplificationID [[amplification_id]],
                             uint instanceID [[instance_id]])  // GPU-provided instance index (0, 1, 2, 3...)
{
    // === VERTEX AMPLIFICATION SETUP ===
    // amplificationID tells us which eye we're rendering for:
    // 0 = left eye, 1 = right eye
    constant auto &pose = poses[amplificationID];
    
    // === TRUE GPU INSTANCING ===
    // instanceID is automatically provided by GPU for each instance in the draw call
    // GPU handles all instances in parallel - much more efficient!
    constant auto &instanceMatrix = instanceMatrices[instanceID];
    
    LayeredVertexOut out;
    
    // === CORE 3D TRANSFORMATION PIPELINE (MVP) ===
    // Using the per-instance transform matrix directly
    out.position = pose.projectionMatrix *           // 3. 3D → 2D screen projection
                   pose.viewMatrix *                 // 2. World → Camera transformation
                   instanceMatrix *                  // 1. Object → World (per-instance)
                   float4(in.position, 1.0f);       // Input vertex (w=1 for points)
    
    // === NORMAL VECTOR TRANSFORMATION ===
    out.viewNormal = (pose.viewMatrix * instanceMatrix * float4(in.normal, 0.0f)).xyz;
    
    // === TEXTURE COORDINATE PROCESSING ===
    out.texCoords = in.texCoords;
    out.texCoords.x = 1.0f - out.texCoords.x;
    
    // === MULTI-VIEW OUTPUT ROUTING ===
    if (useLayeredRendering) {
        out.renderTargetIndex = amplificationID;
    }
    out.viewportIndex = amplificationID;
    
    return out;
}

// === VERTEX SHADER: DEDICATED RENDERING WITH TRUE GPU INSTANCING ===
// Traditional approach: renders one eye at a time with true GPU instanced geometry
[[vertex]]
VertexOut vertex_dedicated_main(VertexIn in [[stage_in]],
                                constant PoseConstants *poses [[buffer(1)]],
                                constant float4x4 *instanceMatrices [[buffer(2)]], // Array of instance transform matrices
                                uint instanceID [[instance_id]])  // GPU-provided instance index
{
    constant auto &pose = poses[0];
    constant auto &instanceMatrix = instanceMatrices[instanceID];  // Use instanceID to select transform
    
    VertexOut out;
    out.position = pose.projectionMatrix * pose.viewMatrix * instanceMatrix * float4(in.position, 1.0f);
    out.viewNormal = (pose.viewMatrix * instanceMatrix * float4(in.normal, 0.0f)).xyz;
    out.texCoords = in.texCoords;
    out.texCoords.x = 1.0f - out.texCoords.x; // Flip uvs horizontally to match Model I/O
    return out;
}

// === FRAGMENT SHADER: PIXEL COLOR CALCULATION ===
// Runs once per pixel to determine final color output  
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
