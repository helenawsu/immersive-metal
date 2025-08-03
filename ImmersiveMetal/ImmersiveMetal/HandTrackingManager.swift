import Foundation
import ARKit
import simd

@objc public class HandTrackingManager: NSObject {
    private let arkitSession = ARKitSession()
    private let handTrackingProvider = HandTrackingProvider()
    private var handPositions: [HandAnchor.Chirality: simd_float3] = [:]
    private var handTrackingTask: Task<Void, Never>?
    private var isInitialized = false
    
    @objc public override init() {
        super.init()
        startHandTracking()
    }
    
    deinit {
        stopHandTracking()
    }
    
    private func startHandTracking() {
        guard HandTrackingProvider.isSupported else { return }
        
        handTrackingTask = Task {
            do {
                try await arkitSession.run([handTrackingProvider])
                isInitialized = true
                
                for await update in handTrackingProvider.anchorUpdates {
                    await handleHandUpdate(update)
                }
            } catch {
                isInitialized = false
            }
        }
    }
    
    @objc public func stopHandTracking() {
        handTrackingTask?.cancel()
        isInitialized = false
    }
    
    @MainActor
    private func handleHandUpdate(_ update: AnchorUpdate<HandAnchor>) {
        switch update.event {
        case .added, .updated:
            handPositions[update.anchor.chirality] = simd_float3(
                update.anchor.originFromAnchorTransform.columns.3.x,
                update.anchor.originFromAnchorTransform.columns.3.y,
                update.anchor.originFromAnchorTransform.columns.3.z
            )
        case .removed:
            handPositions.removeValue(forKey: update.anchor.chirality)
        }
    }
    
    // MARK: - C++ Bridge Methods (Required)
    
    @objc public func getHandCount() -> Int {
        handPositions.count
    }
    
    @objc public func getHandPositionAt(_ index: Int) -> simd_float3 {
        let positions = Array(handPositions.values)
        return index < positions.count ? positions[index] : simd_float3(0, 0, 0)
    }
    
    @objc public func isHandTrackingInitialized() -> Bool {
        isInitialized
    }
}

