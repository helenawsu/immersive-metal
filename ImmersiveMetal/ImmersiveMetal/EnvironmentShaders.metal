#include <metal_stdlib>
using namespace metal;

#include "ShaderTypes.h"

constant bool useLayeredRendering [[function_constant(0)]];

struct VertexIn {
    float3 position  [[attribute(0)]];
    float3 normal    [[attribute(1)]];
    float2 texCoords [[attribute(2)]];
};

struct VertexOut {
    float4 position [[position]];
    float3 modelNormal;
    float2 texCoords;
    float3 worldViewDirection;
};

struct LayeredVertexOut {
    float4 position [[position]];
    float3 modelNormal;
    float2 texCoords;
    float3 worldViewDirection;
    uint renderTargetIndex [[render_target_array_index]];
    uint viewportIndex [[viewport_array_index]];
};

struct EnvironmentFragmentIn {
    float4 position [[position]];
    float3 modelNormal;
    float2 texCoords;
    float3 worldViewDirection;
    uint renderTargetIndex [[render_target_array_index]];
    uint viewportIndex [[viewport_array_index]];
};

[[vertex]]
LayeredVertexOut vertex_environment(VertexIn in [[stage_in]],
                                    constant PoseConstants *poses [[buffer(1)]],
                                    constant EnvironmentConstants &environment [[buffer(2)]],
                                    uint amplificationID [[amplification_id]])
{
    // === VERTEX AMPLIFICATION FOR ENVIRONMENT ===
    // Get camera matrices for this eye (0=left, 1=right)
    constant auto &pose = poses[amplificationID];

    // === ENVIRONMENT SPHERE POSITIONING ===
    // Transform vertex from local object space to world space
    float4 modelPosition = float4(in.position, 1.0f);
    float4 worldPosition = environment.modelMatrix * modelPosition;

    LayeredVertexOut out;
    
    // === STANDARD MVP TRANSFORMATION ===
    // Project environment sphere vertices to screen space
    out.position = pose.projectionMatrix * pose.viewMatrix * worldPosition;
    
    // === INVERTED NORMALS FOR ENVIRONMENT SPHERE ===
    // Environment sphere has inward-facing normals (we're inside looking out)
    // Negate the normal so it points toward the camera
    out.modelNormal = -in.normal;
    
    // === TEXTURE COORDINATES ===
    // Pass through UV coordinates for equirectangular texture sampling
    out.texCoords = in.texCoords;
    
    // === VIEW DIRECTION CALCULATION ===
    // Calculate the direction from camera to this point on the environment sphere
    // This is used in the fragment shader for environment mapping
    out.worldViewDirection = normalize(worldPosition.xyz);

    // === MULTI-VIEW OUTPUT SETUP ===
    if (useLayeredRendering) {
        out.renderTargetIndex = amplificationID;  // Output to correct texture array slice
    }
    out.viewportIndex = amplificationID;  // Output to correct viewport region
    
    return out;
}

[[vertex]]
VertexOut vertex_dedicated_environment(VertexIn in [[stage_in]],
                                       constant PoseConstants *poses [[buffer(1)]],
                                       constant EnvironmentConstants &environment [[buffer(2)]])
{
    constant auto &pose = poses[0];

    float4 modelPosition = float4(in.position, 1.0f);
    float4 worldPosition = environment.modelMatrix * modelPosition;

    VertexOut out;
    out.position = pose.projectionMatrix * pose.viewMatrix * worldPosition;
    out.modelNormal = -in.normal;
    out.texCoords = in.texCoords;
    out.worldViewDirection = normalize(worldPosition.xyz);
    return out;
}

static float2 EquirectUVFromCubeDirection(float3 v) {
    // === EQUIRECTANGULAR PROJECTION MATH ===
    // Convert a 3D direction vector to 2D texture coordinates
    // This maps a sphere to a rectangular image (like world map projections)
    
    // Mathematical constants for coordinate conversion:
    // 1/(2π) = 0.1591549 - converts horizontal angle from radians to [0,1] range
    // 1/π = 0.3183099 - converts vertical angle from radians to [0,1] range  
    const float2 scales { 0.1591549f, 0.3183099f };
    const float2 biases { 0.5f, 0.5f };  // Center the coordinates in [0,1] range
    
    // Convert 3D direction to spherical coordinates:
    // atan2(-v.x, v.z): Calculate horizontal angle (longitude) around Y-axis
    //   - Returns angle in radians from -π to π
    //   - Measures rotation from +Z axis toward -X axis
    //   - Negative v.x flips the direction to match texture layout
    //
    // asin(-v.y): Calculate vertical angle (latitude) from equator
    //   - Returns angle in radians from -π/2 to π/2  
    //   - Negative v.y because Y+ is up but texture V+ is down
    //
    // Note: This assumes +Z is forward in the coordinate system
    float2 uv = float2(atan2(-v.x, v.z),    // Longitude (horizontal wrap-around)
                       asin(-v.y)) *         // Latitude (vertical up-down)
                scales + biases;
    
    // Result: UV coordinates in [0,1] range for texture sampling
    // (0,0) = left edge, bottom of equirectangular image  
    // (1,1) = right edge, top of equirectangular image
    
    return uv;
}

[[fragment]]
half4 fragment_environment(EnvironmentFragmentIn in [[stage_in]],
                           constant EnvironmentConstants &environment,
                           texture2d<half, access::sample> environmentTexture [[texture(0)]])
{
    // === TEXTURE SAMPLER SETUP ===
    // Configure how we sample the HDR environment texture
    constexpr sampler environmentSampler(coord::normalized, filter::linear, 
                                       mip_filter::none, address::repeat);

    // === MIXED REALITY PORTAL EFFECT ===
    // Calculate view direction to determine portal visibility
    float3 V = normalize(in.worldViewDirection);
    
    // Measure how much we're looking "forward" (negative Z direction)
    // -1.0 = looking completely backward, +1.0 = looking completely forward
    float forwardness = -V.z;
    
    // Get portal cutoff angles from the CPU
    // These define the transition zone between visible environment and passthrough
    float cutoffStart = environment.portalCutoffAngles[0];  // Inner edge (sharp cutoff)
    float cutoffEnd = environment.portalCutoffAngles[1];    // Outer edge (full transparency)
    
    // Calculate smooth opacity falloff using smoothstep
    // - When looking forward (forwardness near 1.0): portalOpacity = 1.0 (fully visible)
    // - When looking backward (forwardness near -1.0): portalOpacity = 0.0 (fully transparent) 
    // - In between: smooth transition prevents jarring edges
    half portalOpacity = half(1.0f - smoothstep(cutoffStart, cutoffEnd, forwardness));

    // === ENVIRONMENT TEXTURE SAMPLING ===
    // Convert 3D surface normal to 2D texture coordinates
    // The normal points inward toward the camera from the environment sphere surface
    float3 N = normalize(in.modelNormal);
    
    // Use equirectangular projection to map sphere to rectangular texture
    // This is the same projection used for 360° photos and HDR environment maps
    float2 texCoords = EquirectUVFromCubeDirection(N);
    
    // Sample the HDR environment texture (typically .hdr or .exr format)
    // This contains the 360° lighting information for realistic reflections
    half4 color = environmentTexture.sample(environmentSampler, texCoords);
    
    // === FINAL OUTPUT WITH PORTAL BLENDING ===
    // Multiply RGB by portal opacity for smooth blending
    // Set alpha channel to portalOpacity for proper compositing with real world
    // In mixed mode: portalOpacity controls how much virtual environment vs passthrough is visible
    return half4(color.rgb * portalOpacity, portalOpacity);
}
