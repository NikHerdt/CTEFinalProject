import SwiftUI
import ARKit
import RealityKit
import Combine

struct ARScannerView: View {
    @StateObject private var viewModel = ARScannerViewModel()
    
    var body: some View {
        ZStack {
            ARViewContainer(viewModel: viewModel)
                .edgesIgnoringSafeArea(.all)
            
            VStack {
                Spacer()
                
                HStack(spacing: 20) {
                    Button(action: {
                        viewModel.toggleMeshVisualization()
                    }) {
                        Text(viewModel.showMeshVisualization ? "Hide Mesh" : "Show Mesh")
                            .font(.headline)
                            .foregroundColor(.white)
                            .padding()
                            .background(viewModel.showMeshVisualization ? Color.purple : Color.green)
                            .cornerRadius(10)
                    }
                    
                    Button(action: {
                        if viewModel.savedPoints.count < 2 {
                            viewModel.castRayAndSavePoint()
                        } else {
                            viewModel.deletePoints()
                        }
                    }) {
                        Text(viewModel.savedPoints.count == 0 ? "Save First Point" : 
                             viewModel.savedPoints.count == 1 ? "Save Second Point" : "Delete Points")
                            .font(.headline)
                            .foregroundColor(.white)
                            .padding()
                            .background(viewModel.savedPoints.count == 0 ? Color.blue : 
                                      viewModel.savedPoints.count == 1 ? Color.orange : Color.red)
                            .cornerRadius(10)
                    }
                }
                .padding(.bottom, 30)
            }
            
            if viewModel.showingProgress {
                ProgressView("Processing...")
                    .padding()
                    .background(Color.secondary.opacity(0.3))
                    .cornerRadius(10)
            }
        }
        .navigationTitle("AR Scanner")
        .navigationBarTitleDisplayMode(.inline)
        .alert(isPresented: $viewModel.showingAlert) {
            Alert(
                title: Text(viewModel.alertTitle),
                message: Text(viewModel.alertMessage),
                dismissButton: .default(Text("OK"))
            )
        }
    }
}

struct ARViewContainer: UIViewRepresentable {
    var viewModel: ARScannerViewModel
    
    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        viewModel.arView = arView
        viewModel.setupARSession()
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {}
}

class ARScannerViewModel: ObservableObject {
    @Published var isScanning = true // Start scanning automatically
    @Published var showMeshVisualization = true // Show mesh by default
    @Published var hasMesh = false
    @Published var showingProgress = false
    @Published var showingAlert = false
    @Published var alertTitle = ""
    @Published var alertMessage = ""
    
    var arView: ARView?
    var sceneReconstruction: ARWorldTrackingConfiguration.SceneReconstruction = .mesh
    private var cancellables = Set<AnyCancellable>()
    private var meshAnchors: [UUID: ARMeshAnchor] = [:]
    private(set) var savedPoints: [SIMD3<Float>] = []
    private var pointAnchors: [UInt64: AnchorEntity] = [:]
    private var pathAnchor: AnchorEntity?
    
    func setupARSession() {
        guard let arView = arView else { return }
        
        // Check if device supports scene reconstruction
        guard ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) else {
            showAlert("Device Not Supported", "Your device does not support scene reconstruction.")
            return
        }
        
        let config = ARWorldTrackingConfiguration()
        config.sceneReconstruction = .mesh
        config.environmentTexturing = .automatic
        
        arView.session.run(config)
        
        // Subscribe to AR session events
        arView.scene.subscribe(to: SceneEvents.Update.self) { [weak self] _ in
            guard let self = self else { return }
            self.updateMeshAnchors()
        }
        .store(in: &cancellables)
        
        // Set debug options to visualize the mesh
        updateMeshVisualization()
    }
    
    private func updateMeshAnchors() {
        guard let session = arView?.session, isScanning else { return }
        
        for anchor in session.currentFrame?.anchors ?? [] where anchor is ARMeshAnchor {
            guard let meshAnchor = anchor as? ARMeshAnchor else { continue }
            meshAnchors[meshAnchor.identifier] = meshAnchor
        }
        
        if !meshAnchors.isEmpty {
            hasMesh = true
        }
    }
    
    func toggleMeshVisualization() {
        showMeshVisualization.toggle()
        updateMeshVisualization()
    }
    
    private func updateMeshVisualization() {
        if showMeshVisualization {
            arView?.debugOptions.insert(.showSceneUnderstanding)
        } else {
            arView?.debugOptions.remove(.showSceneUnderstanding)
        }
    }
    
    func castRayAndSavePoint() {
        guard let arView = arView else { return }
        
        // Get the center of the screen
        let center = CGPoint(x: arView.bounds.midX, y: arView.bounds.midY)
        
        // Cast a ray from the center of the screen
        let results = arView.raycast(from: center, allowing: .estimatedPlane, alignment: .any)
        
        if let firstResult = results.first {
            // Convert the hit position to world coordinates
            let hitPosition = firstResult.worldTransform.columns.3
            let point = SIMD3<Float>(hitPosition.x, hitPosition.y, hitPosition.z)
            
            // Save the point
            savedPoints.append(point)
            
            // Create a visual indicator for the saved point
            let sphere = MeshResource.generateSphere(radius: 0.01)
            let material = SimpleMaterial(color: savedPoints.count == 1 ? .red : .blue, isMetallic: false)
            let entity = ModelEntity(mesh: sphere, materials: [material])
            
            let anchor = AnchorEntity(world: point)
            anchor.addChild(entity)
            arView.scene.addAnchor(anchor)
            pointAnchors[anchor.id] = anchor
            
            // If we have two points, create a path between them
            if savedPoints.count == 2 {
                createPathBetweenPoints()
            }
        } else {
            showAlert("No Surface Found", "Could not find a surface to place the point.")
        }
    }
    
    private func createPathBetweenPoints() {
        guard let arView = arView, savedPoints.count == 2 else { return }
        
        // Remove existing path if any
        if let existingPath = pathAnchor {
            arView.scene.removeAnchor(existingPath)
        }
        
        let startPoint = savedPoints[0]
        let endPoint = savedPoints[1]
        
        // Calculate the distance between points
        let distance = simd_distance(startPoint, endPoint)
        
        // Create a cylinder for the path
        let cylinder = MeshResource.generateCylinder(height: distance, radius: 0.005)
        let material = SimpleMaterial(color: .green, isMetallic: false)
        let pathEntity = ModelEntity(mesh: cylinder, materials: [material])
        
        // Position the cylinder between the points
        let midPoint = (startPoint + endPoint) / 2
        let direction = normalize(endPoint - startPoint)
        
        // Create a transform that aligns the cylinder with the direction vector
        let transform = Transform(
            scale: .one,
            rotation: simd_quaternion(SIMD3<Float>(0, 1, 0), direction),
            translation: midPoint
        )
        
        pathEntity.transform = transform
        
        // Create and add the path anchor
        let pathAnchor = AnchorEntity(world: .zero)
        pathAnchor.addChild(pathEntity)
        arView.scene.addAnchor(pathAnchor)
        self.pathAnchor = pathAnchor
    }
    
    func deletePoints() {
        guard let arView = arView else { return }
        
        // Remove all point anchors
        for (_, anchor) in pointAnchors {
            arView.scene.removeAnchor(anchor)
        }
        pointAnchors.removeAll()
        
        // Remove path anchor if it exists
        if let pathAnchor = pathAnchor {
            arView.scene.removeAnchor(pathAnchor)
            self.pathAnchor = nil
        }
        
        // Clear saved points
        savedPoints.removeAll()
    }
    
    private func showAlert(_ title: String, _ message: String) {
        alertTitle = title
        alertMessage = message
        showingAlert = true
    }
}

#Preview {
    ARScannerView()
} 
