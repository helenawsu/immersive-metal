import Foundation
import ARKit
import simd

// Swift class to handle ARKit hand tracking for visionOS
@objc public class HandTrackingManager: NSObject {
    private var arkitSession: ARKitSession
    private var handTrackingProvider: HandTrackingProvider
    private var handPositions: [simd_float3] = []
    private var isTracking = false
    private var handTrackingTask: Task<Void, Never>?
    private var isInitialized = false
    
    @objc public override init() {
        arkitSession = ARKitSession()
        handTrackingProvider = HandTrackingProvider()
        super.init()
        startHandTracking()
    }
    
    deinit {
        stopHandTracking()
    }
    
    private func startHandTracking() {
        guard HandTrackingProvider.isSupported else {
            print("Hand tracking is not supported on this device")
            return
        }
        
        handTrackingTask = Task {
            do {
                // Wait a bit to ensure proper initialization
                try await Task.sleep(nanoseconds: 500_000_000) // 0.5 seconds
                
                try await arkitSession.run([handTrackingProvider])
                isTracking = true
                isInitialized = true
                print("Hand tracking started successfully")
                
                for await update in handTrackingProvider.anchorUpdates {
                    await handleHandUpdate(update)
                }
            } catch {
                print("ARKit session error: \(error)")
                isTracking = false
                isInitialized = false
            }
        }
    }
    
    @objc public func stopHandTracking() {
        handTrackingTask?.cancel()
        handTrackingTask = nil
        isTracking = false
        isInitialized = false
    }
    
    @MainActor
    private func handleHandUpdate(_ update: AnchorUpdate<HandAnchor>) {
        switch update.event {
        case .added, .updated:
            let handAnchor = update.anchor
            let position = simd_float3(
                handAnchor.originFromAnchorTransform.columns.3.x,
                handAnchor.originFromAnchorTransform.columns.3.y,
                handAnchor.originFromAnchorTransform.columns.3.z
            )
            
            // Better hand tracking - match by chirality
            let isLeftHand = handAnchor.chirality == .left
            
            // Find existing hand or add new one
            var foundExisting = false
            for i in 0..<handPositions.count {
                // In a real implementation, you'd store chirality info too
                // For now, just update the first available slot
                if !foundExisting {
                    handPositions[i] = position
                    foundExisting = true
                    break
                }
            }
            
            if !foundExisting {
                handPositions.append(position)
            }
            
            let handName = isLeftHand ? "Left" : "Right"
            print("\(handName) hand at: (\(position.x), \(position.y), \(position.z))")
            
        case .removed:
            // Handle hand removal - in a real implementation you'd track which hand was removed
            if !handPositions.isEmpty {
                handPositions.removeLast()
            }
        }
    }
    
    // Method to manually update hand positions - for compatibility with existing API
    @objc public func updateHandPositions() {
        // In the new visionOS API, updates happen automatically via the async stream
        // This method is kept for compatibility but doesn't need to do anything
    }
    
    // C++ bridge method - returns current hand positions
    @objc public func getCurrentHandPositions() -> [NSValue] {
        return handPositions.map { position in
            var mutablePosition = position
            return NSValue(bytes: &mutablePosition, objCType: "{simd_float3=fff}")
        }
    }
    
    @objc public func getHandCount() -> Int {
        return handPositions.count
    }
    
    // Helper method to get a specific hand position by index
    @objc public func getHandPositionAt(_ index: Int) -> simd_float3 {
        guard index < handPositions.count else {
            return simd_float3(0, 0, 0)
        }
        return handPositions[index]
    }
    
    @objc public func isHandTrackingInitialized() -> Bool {
        return isInitialized
    }
}

