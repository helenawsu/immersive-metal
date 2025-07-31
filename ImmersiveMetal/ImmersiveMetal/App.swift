import SwiftUI
import CompositorServices

// === METAL LAYER CONFIGURATION ===
// Configures how Metal renders to Vision Pro displays
struct MetalLayerConfiguration: CompositorLayerConfiguration {
    func makeConfiguration(capabilities: LayerRenderer.Capabilities,
                           configuration: inout LayerRenderer.Configuration)
    {
        let supportsFoveation = capabilities.supportsFoveation
        let supportedLayouts = capabilities.supportedLayouts(options: supportsFoveation ? [.foveationEnabled] : [])
        
        // Choose optimal rendering strategy:
        // .layered = single-pass stereo (most efficient)
        // .dedicated = separate passes per eye (fallback)
        configuration.layout = supportedLayouts.contains(.layered) ? .layered : .dedicated
        configuration.isFoveationEnabled = supportsFoveation      // Higher res where user looks
        configuration.colorFormat = .rgba16Float                  // HDR color for realistic lighting
    }
}

@main
struct FullyImmersiveMetalApp: App {
    @State var immersionStyle: (any ImmersionStyle) = FullImmersionStyle.full
    @State var rendererConfig = SRConfiguration(immersionStyle: .full)

    var body: some Scene {
        // === 2D CONTROL WINDOW ===
        // Traditional SwiftUI interface that floats in 3D space
        WindowGroup {
            ContentView($immersionStyle, rendererConfig)
                .frame(minWidth: 480, maxWidth: 480, minHeight: 200, maxHeight: 320)
        }
        .windowResizability(.contentSize)

        // === 3D IMMERSIVE SPACE ===
        // Full 360° 3D environment rendered using Metal
        ImmersiveSpace(id: "ImmersiveSpace") {
            CompositorLayer(configuration: MetalLayerConfiguration()) { layerRenderer in
                // Bridge from SwiftUI to Metal 3D rendering pipeline
                SpatialRenderer_InitAndRun(layerRenderer, rendererConfig)
            }
        }
        .immersionStyle(selection: $immersionStyle, in: .mixed, .full) // Mixed reality or full immersion
    }
}
