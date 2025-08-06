#pragma once

#include <simd/simd.h>

// === CPU-GPU DATA STRUCTURES ===
// These structs define data passed from CPU to GPU shaders

// Camera and view transformation data (per eye)
struct PoseConstants {
    simd_float4x4 projectionMatrix; // 3D camera space → 2D screen space  
    simd_float4x4 viewMatrix;       // World space → camera space
};

// Per-object transformation data
struct InstanceConstants {
    simd_float4x4 modelMatrix;      // Local object space → world space
};

// Environment sphere specific data
struct EnvironmentConstants {
    simd_float4x4 modelMatrix;      // Environment sphere transformation
    simd_float4x4 environmentRotation; // Optional texture rotation
    simd_float2 portalCutoffAngles; // Mixed reality portal: x=inner, y=outer cutoff cosines
};

// Particle physics constants for GPU compute shader
struct ParticleConstants {
    simd_float3 headPosition;       // Current head position for orbital attraction
    uint32_t handCount;             // Number of active hands
    float deltaTime;                // Frame time delta for smooth animation
    float repulsionRadius;          // Radius around hands that affects particles
    float repulsionStrength;        // How strong the repulsion force is
    float driftStrength;            // Random drift force strength
    float damping;                  // Velocity damping factor
    float centerPullStrength;       // Attraction to head position
    float boundary;                 // Boundary for particle containment
};

// Glow effect constants for GPU shaders
struct GlowConstants {
    float glowScale;                // Scale factor for glow geometry (1.2 - 2.0)
    float glowIntensity;            // Overall glow brightness multiplier
    simd_float3 glowColor;          // Glow color tint (RGB)
    float glowFalloff;              // Falloff exponent for radial gradient
};
