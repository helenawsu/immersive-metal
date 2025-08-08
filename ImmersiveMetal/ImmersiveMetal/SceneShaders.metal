
#include <metal_stdlib>
using namespace metal;

#include "ShaderTypes.h"

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
    constexpr sampler environmentSampler(coord::normalized, filter::linear, mip_filter::none, address::repeat);
    
    // Sample the diffuse texture at the interpolated UV coordinates
    half4 color = texture.sample(environmentSampler, in.texCoords);
    
    return color;
}

// === SIMPLE GLOW FRAGMENT SHADER ===
// Creates transparent red glow cubes for debugging visibility
[[fragment]]
half4 fragment_glow(FragmentIn in [[stage_in]])
{
    return half4(0.0, 1.0, 1.0, 0.5); // cyan
}

// === TRAIL FRAGMENT SHADER WITH AGE-BASED FADING ===
// Note: Fragment shaders cannot access instance_id directly
// Trail aging is handled by passing different alpha values through vertex data
// or by using different draw calls with different constants
[[fragment]]
half4 fragment_trail(FragmentIn in [[stage_in]],
                     texture2d<half, access::sample> texture [[texture(0)]],
                     constant TrailConstants &trailConstants [[buffer(0)]])
{
    // Sample base texture
    constexpr sampler textureSampler(coord::normalized, filter::linear, address::repeat);
//    half4 color = texture.sample(textureSampler, in.texCoords);
    half4 color = half4(0.0, 1.0, 1.0, 0.5);
    
    // Apply base alpha fade - specific aging is handled by vertex shader or instance constants
    // For now, use a uniform fade that can be modulated per-instance at draw time
    color.a *= half(trailConstants.fadeRate);
    
    return color; //faded cyan

}

// === GPU PARTICLE PHYSICS COMPUTE SHADER ===
// Handles particle movement, hand repulsion, and orbital motion on the GPU
// Each thread processes one particle in parallel




[[kernel]]
void updateParticles(device float3 *positions [[buffer(0)]],
                     device float3 *velocities [[buffer(1)]],
                     constant float3 *handPositions [[buffer(2)]],
                     constant ParticleConstants &constants [[buffer(3)]],
                     device float4x4 *instanceTransforms [[buffer(4)]],  // Instance transforms output
                     device float3 *trailHistory [[buffer(5)]],          // Trail position history
                     device float4x4 *trailTransforms [[buffer(6)]],     // Trail instance transforms
                     constant TrailConstants &trailConstants [[buffer(7)]], // Trail config
                     uint index [[thread_position_in_grid]])
{
    // Bounds check - important for GPU safety
    if (index >= constants.boundary * 1000) return; // Using boundary as rough particle count check
    
    // Get current particle data
    float3 position = positions[index];
    float3 velocity = velocities[index];
    
    // === HAND REPULSION FORCES ===
    float3 repulsionForce = float3(0.0f);
    
    for (uint handIndex = 0; handIndex < constants.handCount; ++handIndex) {
        float3 handToParticle = position - handPositions[handIndex];
        float distance = length(handToParticle);
        
        // Apply repulsion if particle is within repulsion radius
        if (distance < constants.repulsionRadius && distance > 0.001f) {
            // Normalize direction vector
            float3 direction = handToParticle / distance;
            
            // Calculate falloff: stronger force when closer to hand
            float falloff = 1.0f - (distance / constants.repulsionRadius);
            falloff = falloff * falloff;  // Square for more dramatic effect
            
            // Apply repulsion force
            repulsionForce += direction * constants.repulsionStrength * falloff;
        }
    }
    
    // === IMPROVED RANDOM DRIFT ===
    // Use better hash function for particle index + time
    uint seed = index * 747796405u + 2891336453u;
    uint time_seed = uint(constants.deltaTime * 1000000.0f); // Use frame time as extra randomness
    seed ^= time_seed;
    
    // Generate three different random values
    float3 drift;
    
    // First random value
    seed = seed * 747796405u + 2891336453u;
    seed = ((seed >> ((seed >> 28u) + 4u)) ^ seed) * 277803737u;
    drift.x = (float(seed & 0xFFFFu) / 65535.0f - 0.5f) * constants.driftStrength;
    
    // Second random value
    seed = seed * 747796405u + 2891336453u;
    seed = ((seed >> ((seed >> 28u) + 4u)) ^ seed) * 277803737u;
    drift.y = (float(seed & 0xFFFFu) / 65535.0f - 0.5f) * constants.driftStrength;
    
    // Third random value
    seed = seed * 747796405u + 2891336453u;
    seed = ((seed >> ((seed >> 28u) + 4u)) ^ seed) * 277803737u;
    drift.z = (float(seed & 0xFFFFu) / 65535.0f - 0.5f) * constants.driftStrength;
    
    // === APPLY PHYSICS ===
    // Apply gentle damping to prevent runaway velocities
    velocity = velocity * constants.damping + drift;
    
    // Add stronger orbital motion around head position (matching CPU version)
    float3 toCenter = constants.headPosition - position;
    float distanceToHead = length(toCenter);
    
    // Stronger center pull that increases with distance from head (like CPU version)
    if (distanceToHead > 0.001f) {  // Avoid division by zero
        float3 centerDirection = toCenter / distanceToHead;
        // Pull gets stronger the farther away particles are (distance-scaled)
        float pullForce = constants.centerPullStrength * distanceToHead;
        velocity += centerDirection * pullForce;
    }
    
    // Add repulsion forces
    velocity += repulsionForce;
    
    // Update position with proper timestep
    position += velocity * constants.deltaTime;
    
    // === BOUNDARY CONSTRAINTS ===
    // Bounce off invisible walls to keep particles in view
    if (position.x > constants.boundary || position.x < -constants.boundary) {
        velocity.x *= -0.8f;  // Reverse and dampen
        // Clamp position within bounds to prevent escape
        position.x = clamp(position.x, -constants.boundary, constants.boundary);
    }
    if (position.y > constants.boundary || position.y < -constants.boundary) {
        velocity.y *= -0.8f;
        position.y = clamp(position.y, -constants.boundary, constants.boundary);
    }
    if (position.z > constants.boundary || position.z < -constants.boundary) {
        velocity.z *= -0.8f;
        position.z = clamp(position.z, -constants.boundary, constants.boundary);
    }
    
    // === TRAIL POSITION HISTORY UPDATE ===
    // Shift trail history (move older positions down the chain)
    uint trailBaseIndex = index * trailConstants.trailLength;
    for (int i = int(trailConstants.trailLength) - 1; i > 0; i--) {
        trailHistory[trailBaseIndex + uint(i)] = trailHistory[trailBaseIndex + uint(i - 1)];
    }
    // Store current position as most recent trail segment
    trailHistory[trailBaseIndex] = position;
    
    // === TRAIL TRANSFORM MATRIX GENERATION ===
    for (uint trailIndex = 0; trailIndex < trailConstants.trailLength; trailIndex++) {
        float3 trailPos = trailHistory[trailBaseIndex + trailIndex];
        
        // Calculate age-based scaling
        float age = float(trailIndex) / float(trailConstants.trailLength);
        float scale = trailConstants.sizeScale * (1.0f - age * 0.3f); // Slight size fade
        
        // Generate scaled transformation matrix
        uint trailInstanceIndex = index * trailConstants.trailLength + trailIndex;
        trailTransforms[trailInstanceIndex] = float4x4(
            float4(scale, 0.0, 0.0, 0.0),
            float4(0.0, scale, 0.0, 0.0),
            float4(0.0, 0.0, scale, 0.0),
            float4(trailPos, 1.0)
        );
    }

    // === GENERATE TRANSFORMATION MATRIX ON GPU ===
    // Create translation matrix directly from updated position
    // This eliminates the CPU->GPU roundtrip completely!
    instanceTransforms[index] = float4x4(
        float4(1.0, 0.0, 0.0, 0.0),        // First column: X-axis (identity)
        float4(0.0, 1.0, 0.0, 0.0),        // Second column: Y-axis (identity)
        float4(0.0, 0.0, 1.0, 0.0),        // Third column: Z-axis (identity)
        float4(position, 1.0)              // Fourth column: Translation (W=1)
    );
    
    // Write back updated values
    positions[index] = position;
    velocities[index] = velocity;
}
