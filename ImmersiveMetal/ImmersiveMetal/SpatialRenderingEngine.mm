#include "SpatialRenderingEngine.h"
#include "SpatialRenderer.h"
#include "ShaderTypes.h"
#include "Mesh.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <ARKit/ARKit.h>
#import <Spatial/Spatial.h>
#import "ImmersiveMetal-Swift.h" // Import Swift bridge

@implementation SRConfiguration

- (instancetype)initWithImmersionStyle:(SRImmersionStyle)immersionStyle {
    if (self = [super init]) {
        _immersionStyle = immersionStyle;
        _portalCutoffAngle = 180.0f;
    }
    return self;
}

- (instancetype)init {
    return [self initWithImmersionStyle:SRImmersionStyleFull];
}

@end

class SpatialRenderingEngine {
public:
    SpatialRenderingEngine(cp_layer_renderer_t layerRenderer, SRConfiguration *configuration) :
        _layerRenderer(layerRenderer),
        _isARSessionRunning(false),
        _arSessionStartTime(0.0)
    {
        _renderer = std::make_unique<SpatialRenderer>(layerRenderer, configuration);
        runWorldTrackingARSession();
        
        // Initialize Swift hand tracking manager - much simpler!
        _handTrackingManager = [[HandTrackingManager alloc] init];
    }

    ~SpatialRenderingEngine() {
        ar_session_stop(_arSession);
        [_handTrackingManager stopHandTracking];
    }

    void runLoop() {
        while (_running) {
            @autoreleasepool {
                switch (cp_layer_renderer_get_state(_layerRenderer)) {
                    case cp_layer_renderer_state_paused:
                        cp_layer_renderer_wait_until_running(_layerRenderer);
                        break;
                        
                    case cp_layer_renderer_state_running:
                        renderFrame();
                        break;
                        
                        
                    case cp_layer_renderer_state_invalidated:
                        _running = false;
                        break;
                }
            }
        }
    }

    void renderFrame() {
        cp_frame_t frame = cp_layer_renderer_query_next_frame(_layerRenderer);
        if (frame == nullptr) {
            return;
        }

        cp_frame_timing_t timing = cp_frame_predict_timing(frame);
        if (timing == nullptr) {
            return;
        }

        cp_frame_start_update(frame);
        
        //gather_inputs(engine, timing);
        //update_frame(engine, timing, input_state);
        
        // Simple hand tracking update using Swift bridge
        updateHandTracking();

        cp_frame_end_update(frame);
        
        cp_time_wait_until(cp_frame_timing_get_optimal_input_time(timing));
        
        cp_frame_start_submission(frame);
        cp_drawable_t drawable = cp_frame_query_drawable(frame);
        if (drawable == nullptr) {
            return;
        }

        cp_frame_timing_t actualTiming = cp_drawable_get_frame_timing(drawable);
        ar_device_anchor_t anchor = createPoseForTiming(actualTiming);
        cp_drawable_set_device_anchor(drawable, anchor);

        _renderer->drawAndPresent(frame, drawable);

        cp_frame_end_submission(frame);
    }

private:
    void runWorldTrackingARSession() {
        ar_world_tracking_configuration_t worldTrackingConfiguration = ar_world_tracking_configuration_create();
        _worldTrackingProvider = ar_world_tracking_provider_create(worldTrackingConfiguration);

        ar_data_providers_t dataProviders = ar_data_providers_create_with_data_providers(_worldTrackingProvider, nil);

        _arSession = ar_session_create();
        
        // Start the AR session
        ar_session_run(_arSession, dataProviders);
        
        // Record when we started the session
        _arSessionStartTime = CFAbsoluteTimeGetCurrent();
        
        NSLog(@"AR session started, waiting for world tracking to initialize...");
    }
    
    void updateHandTracking() {
        // Much simpler - just get positions from Swift manager!
        std::vector<simd_float3> handPositions;
        
        NSInteger handCount = [_handTrackingManager getHandCount];
        for (NSInteger i = 0; i < handCount; i++) {
            simd_float3 position = [_handTrackingManager getHandPositionAt:i];
            handPositions.push_back(position);
        }
        
        // Update the renderer with current hand positions
        _renderer->updateHandPositions(handPositions);
        
        if (handCount > 0) {
            NSLog(@"Updated renderer with %ld hand positions", (long)handCount);
        }
    }

    ar_device_anchor_t createPoseForTiming(cp_frame_timing_t timing) {
        ar_device_anchor_t outAnchor = ar_device_anchor_create();
        cp_time_t presentationTime = cp_frame_timing_get_presentation_time(timing);
        CFTimeInterval queryTime = cp_time_to_cf_time_interval(presentationTime);
        
        // Check if AR session has had time to initialize (wait at least 1 second)
        CFTimeInterval currentTime = CFAbsoluteTimeGetCurrent();
        if (!_isARSessionRunning && (currentTime - _arSessionStartTime) > 1.0) {
            _isARSessionRunning = true;
            NSLog(@"AR session should now be ready for tracking queries");
        }
        
        // Only query if we believe the session is running
        if (_isARSessionRunning) {
            ar_device_anchor_query_status_t status = ar_world_tracking_provider_query_device_anchor_at_timestamp(_worldTrackingProvider, queryTime, outAnchor);
            
            switch (status) {
                case ar_device_anchor_query_status_success:
                    // Success - no logging needed for normal operation
                    break;
                    
                case ar_device_anchor_query_status_failure:
                    // This is normal during startup - don't spam the log
                    static int notAvailableCount = 0;
                    if (notAvailableCount % 60 == 0) {  // Log every ~1 second at 60fps
                        NSLog(@"Device anchor not yet available (session still initializing)");
                    }
                    notAvailableCount++;
                    break;
                  
            }
        } else {
            // Still waiting for AR session to initialize
            static int waitingCount = 0;
            if (waitingCount % 60 == 0) {  // Log every ~1 second at 60fps
                NSLog(@"Waiting for AR session to initialize... (%0.1f seconds)", currentTime - _arSessionStartTime);
            }
            waitingCount++;
        }
        
        return outAnchor;
    }

    ar_session_t _arSession;
    ar_world_tracking_provider_t _worldTrackingProvider;
    bool _isARSessionRunning;
    CFTimeInterval _arSessionStartTime;
    
    // Swift hand tracking manager - much cleaner!
    HandTrackingManager *_handTrackingManager;
    
    cp_layer_renderer_t _layerRenderer;
    std::unique_ptr<SpatialRenderer> _renderer;
    bool _running = true;
};

@interface RenderThread : NSThread {
    cp_layer_renderer_t _layerRenderer;
    std::unique_ptr<SpatialRenderingEngine> _engine;
}

- (instancetype)initWithLayerRenderer:(cp_layer_renderer_t)layerRenderer
                        configuration:(SRConfiguration *)configuration;

@end

@implementation RenderThread

- (instancetype)initWithLayerRenderer:(cp_layer_renderer_t)layerRenderer
                        configuration:(SRConfiguration *)configuration
{
    if (self = [self init]) {
        _layerRenderer = layerRenderer;
        _engine = std::make_unique<SpatialRenderingEngine>(layerRenderer, configuration);
    }
    return self;
}

- (void)main {
    _engine->runLoop();
}

@end

void SpatialRenderer_InitAndRun(cp_layer_renderer_t layerRenderer, SRConfiguration *configuration) {
    RenderThread *renderThread = [[RenderThread alloc] initWithLayerRenderer:layerRenderer configuration:configuration];
    renderThread.name = @"Spatial Renderer Thread";
    [renderThread start];
}
