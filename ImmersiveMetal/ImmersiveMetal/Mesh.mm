
#include "Mesh.h"

#import <ModelIO/ModelIO.h>

// === TEXTURE LOADING UTILITY ===
// Loads image files from app bundle and converts them to Metal textures for GPU use
static id<MTLTexture> _Nullable CreateTextureFromImage(NSString *imageName, id<MTLDevice> device, NSError **error) {
    MTKTextureLoader *textureLoader = [[MTKTextureLoader alloc] initWithDevice:device];
    NSURL *imageURL = [[NSBundle mainBundle] URLForResource:imageName withExtension:nil];
    CGImageSourceRef imageSource = CGImageSourceCreateWithURL((__bridge CFURLRef)imageURL, NULL);
    if (imageSource) {
        CGImageRef image = CGImageSourceCreateImageAtIndex(imageSource, 0, NULL);
        if (image) {
            // Upload image data to GPU memory as a Metal texture
            id<MTLTexture> texture = [textureLoader newTextureWithCGImage:image options:nil error:error];
            CGImageRelease(image);
            return texture;
        }
        CFRelease(imageSource);
    }
    return nil;
}

MTLVertexDescriptor *Mesh::vertexDescriptor() const {
    // === VERTEX DATA LAYOUT DEFINITION ===
    // This describes the structure of our vertex data for the GPU
    // It tells Metal how to interpret the bytes in our vertex buffer
    MTLVertexDescriptor *vertexDescriptor = [MTLVertexDescriptor new];
    
    // === VERTEX ATTRIBUTE 0: POSITION ===
    // 3D position coordinates (x, y, z) as 32-bit floats
    // Maps to [[attribute(0)]] in the vertex shader
    vertexDescriptor.attributes[0].format = MTLVertexFormatFloat3;  // 3 floats (12 bytes)
    vertexDescriptor.attributes[0].bufferIndex = 0;                 // From buffer slot 0
    vertexDescriptor.attributes[0].offset = 0;                      // Starts at byte 0
    
    // === VERTEX ATTRIBUTE 1: NORMAL ===  
    // 3D surface normal vector (x, y, z) as 32-bit floats
    // Used for lighting calculations and surface orientation
    // Maps to [[attribute(1)]] in the vertex shader
    vertexDescriptor.attributes[1].format = MTLVertexFormatFloat3;  // 3 floats (12 bytes)
    vertexDescriptor.attributes[1].bufferIndex = 0;                 // From buffer slot 0
    vertexDescriptor.attributes[1].offset = sizeof(float) * 3;      // Starts at byte 12
    
    // === VERTEX ATTRIBUTE 2: TEXTURE COORDINATES ===
    // 2D texture coordinates (u, v) as 32-bit floats
    // Defines how textures are mapped onto the 3D surface
    // Maps to [[attribute(2)]] in the vertex shader  
    vertexDescriptor.attributes[2].format = MTLVertexFormatFloat2;  // 2 floats (8 bytes)
    vertexDescriptor.attributes[2].bufferIndex = 0;                 // From buffer slot 0
    vertexDescriptor.attributes[2].offset = sizeof(float) * 6;      // Starts at byte 24
    
    // === VERTEX BUFFER LAYOUT ===
    // Define how vertices are packed in memory
    // Each vertex contains: position (12) + normal (12) + texCoords (8) = 32 bytes total
    vertexDescriptor.layouts[0].stride = sizeof(float) * 8;  // 32 bytes between vertices
    
    // Memory layout of each vertex:
    // Bytes 0-11:   Position (float3)
    // Bytes 12-23:  Normal (float3)  
    // Bytes 24-31:  TexCoords (float2)
    // Then pattern repeats for next vertex...
    
    return vertexDescriptor;
}

TexturedMesh::TexturedMesh() = default;

TexturedMesh::TexturedMesh(MDLMesh *mdlMesh, NSString *imageName, id<MTLDevice> device) {
    NSError *error = nil;

    _texture = CreateTextureFromImage(imageName, device, &error);

    MDLVertexDescriptor *mdlVertexDescriptor = [MDLVertexDescriptor new];
    mdlVertexDescriptor.attributes[0].name = MDLVertexAttributePosition;
    mdlVertexDescriptor.attributes[0].format = MDLVertexFormatFloat3;
    mdlVertexDescriptor.attributes[0].bufferIndex = 0;
    mdlVertexDescriptor.attributes[0].offset = 0;
    mdlVertexDescriptor.attributes[1].name = MDLVertexAttributeNormal;
    mdlVertexDescriptor.attributes[1].format = MDLVertexFormatFloat3;
    mdlVertexDescriptor.attributes[1].bufferIndex = 0;
    mdlVertexDescriptor.attributes[1].offset = sizeof(float) * 3;
    mdlVertexDescriptor.attributes[2].name = MDLVertexAttributeTextureCoordinate;
    mdlVertexDescriptor.attributes[2].format = MDLVertexFormatFloat2;
    mdlVertexDescriptor.attributes[2].bufferIndex = 0;
    mdlVertexDescriptor.attributes[2].offset = sizeof(float) * 6;
    mdlVertexDescriptor.layouts[0].stride = sizeof(float) * 8;
    
    mdlMesh.vertexDescriptor = mdlVertexDescriptor;
    
    // Convert ModelIO mesh to Metal-compatible format and upload to GPU
    _mesh = [[MTKMesh alloc] initWithMesh:mdlMesh device:device error:&error];
}

void TexturedMesh::draw(id<MTLRenderCommandEncoder> renderCommandEncoder, PoseConstants *poseConstants, size_t poseCount) {
    // === INSTANCE DATA PREPARATION ===
    // Create per-object data that will be sent to the vertex shader
    InstanceConstants instanceConstants;
    instanceConstants.modelMatrix = modelMatrix();  // Current transformation matrix for this object

    // === MESH GEOMETRY BINDING ===
    // Get the first (and typically only) submesh from our mesh
    // A submesh contains a set of triangles that share the same material
    MTKSubmesh *submesh = _mesh.submeshes.firstObject;
    
    // Get the vertex buffer containing position, normal, and texture coordinate data
    MTKMeshBuffer *vertexBuffer = _mesh.vertexBuffers.firstObject;
    
    // === VERTEX SHADER RESOURCE BINDING ===
    // Bind vertex data to buffer slot 0 (matches [[buffer(0)]] in shader)
    // This contains interleaved vertex attributes: position, normal, texCoords
    [renderCommandEncoder setVertexBuffer:vertexBuffer.buffer 
                                   offset:vertexBuffer.offset 
                                  atIndex:0];
    
    // Bind camera/view data to buffer slot 1 (matches [[buffer(1)]] in shader)
    // Contains projection and view matrices for each eye
    [renderCommandEncoder setVertexBytes:poseConstants 
                                  length:sizeof(PoseConstants) * poseCount 
                                 atIndex:1];
    
    // Bind per-object data to buffer slot 2 (matches [[buffer(2)]] in shader)  
    // Contains the model transformation matrix for this specific object
    [renderCommandEncoder setVertexBytes:&instanceConstants 
                                  length:sizeof(instanceConstants) 
                                 atIndex:2];

    // === FRAGMENT SHADER RESOURCE BINDING ===
    // Bind the diffuse texture to slot 0 (matches [[texture(0)]] in shader)
    // This is the color/albedo texture that gets mapped onto the 3D surface
    [renderCommandEncoder setFragmentTexture:_texture atIndex:0];

    // === GPU DRAW COMMAND ===
    // Issue the actual draw call that renders the triangles
    // This is where the GPU processes all vertices and generates pixels
    [renderCommandEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle  // Render triangular faces
                                     indexCount:submesh.indexCount         // Number of vertex indices to process
                                      indexType:submesh.indexType          // Size of each index (16-bit or 32-bit)
                                    indexBuffer:submesh.indexBuffer.buffer // Buffer containing vertex indices
                              indexBufferOffset:submesh.indexBuffer.offset];
    
    // The index buffer contains integers that reference vertices in the vertex buffer
    // This allows vertices to be shared between multiple triangles (more memory efficient)
    // GPU processes 3 indices at a time to form each triangle
}

SpatialEnvironmentMesh::SpatialEnvironmentMesh(NSString *imageName, CGFloat radius, id<MTLDevice> device) :
    TexturedMesh()
{
    NSError *error = nil;
    _texture = CreateTextureFromImage(imageName, device, &error);

    _environmentRotation = matrix_identity_float4x4;

    MTKMeshBufferAllocator *bufferAllocator = [[MTKMeshBufferAllocator alloc] initWithDevice:device];
    MDLMesh *mdlMesh = [MDLMesh newEllipsoidWithRadii:simd_make_float3(radius, radius, radius)
                                       radialSegments:24
                                     verticalSegments:24
                                         geometryType:MDLGeometryTypeTriangles
                                        inwardNormals:YES  // Makes sphere viewable from inside (skybox effect)
                                           hemisphere:NO
                                            allocator:bufferAllocator];


    MDLVertexDescriptor *mdlVertexDescriptor = [MDLVertexDescriptor new];
    mdlVertexDescriptor.attributes[0].name = MDLVertexAttributePosition;
    mdlVertexDescriptor.attributes[0].format = MDLVertexFormatFloat3;
    mdlVertexDescriptor.attributes[0].bufferIndex = 0;
    mdlVertexDescriptor.attributes[0].offset = 0;
    mdlVertexDescriptor.attributes[1].name = MDLVertexAttributeNormal;
    mdlVertexDescriptor.attributes[1].format = MDLVertexFormatFloat3;
    mdlVertexDescriptor.attributes[1].bufferIndex = 0;
    mdlVertexDescriptor.attributes[1].offset = sizeof(float) * 3;
    mdlVertexDescriptor.attributes[2].name = MDLVertexAttributeTextureCoordinate;
    mdlVertexDescriptor.attributes[2].format = MDLVertexFormatFloat2;
    mdlVertexDescriptor.attributes[2].bufferIndex = 0;
    mdlVertexDescriptor.attributes[2].offset = sizeof(float) * 6;
    mdlVertexDescriptor.layouts[0].stride = sizeof(float) * 8;

    mdlMesh.vertexDescriptor = mdlVertexDescriptor;

    _mesh = [[MTKMesh alloc] initWithMesh:mdlMesh device:device error:&error];
}

float SpatialEnvironmentMesh::cutoffAngle() const {
    return _cutoffAngle;
}

void SpatialEnvironmentMesh::setCutoffAngle(float cutoffAngle) {
    _cutoffAngle = cutoffAngle;
}

// === ENVIRONMENT MESH DRAW METHOD ===
// Renders the 360° background sphere with mixed reality portal support
void SpatialEnvironmentMesh::draw(id<MTLRenderCommandEncoder> renderCommandEncoder, PoseConstants *poseConstants, size_t poseCount) {
    // Calculate portal cutoff angles for mixed reality blending
    float cutoffAngleMin = cos(simd_clamp(_cutoffAngle - _cutoffEdgeWidth, 0.0f, 180.0f) * (M_PI / 180.0f));
    float cutoffAngleMax = cos(simd_clamp(_cutoffAngle + _cutoffEdgeWidth, 0.0f, 180.0f) * (M_PI / 180.0f));

    EnvironmentConstants environmentConstants;
    environmentConstants.modelMatrix = modelMatrix();
    environmentConstants.environmentRotation = matrix_identity_float4x4;
    environmentConstants.portalCutoffAngles = simd_make_float2(cutoffAngleMin, cutoffAngleMax);

    MTKSubmesh *submesh = _mesh.submeshes.firstObject;
    MTKMeshBuffer *vertexBuffer = _mesh.vertexBuffers.firstObject;
    [renderCommandEncoder setVertexBuffer:vertexBuffer.buffer offset:vertexBuffer.offset atIndex:0];
    [renderCommandEncoder setVertexBytes:poseConstants length:sizeof(PoseConstants) * poseCount atIndex:1];
    [renderCommandEncoder setVertexBytes:&environmentConstants length:sizeof(environmentConstants) atIndex:2];
    [renderCommandEncoder setFragmentBytes:&environmentConstants length:sizeof(environmentConstants) atIndex:0];
    [renderCommandEncoder setFragmentTexture:_texture atIndex:0];
    [renderCommandEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                     indexCount:submesh.indexCount
                                      indexType:submesh.indexType
                                    indexBuffer:submesh.indexBuffer.buffer
                              indexBufferOffset:submesh.indexBuffer.offset];
}
