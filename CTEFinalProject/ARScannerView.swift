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
    
    func setupARSession() {
        guard let arView = arView else { return }
        
        let session = arView.session
        
        // Check if device supports scene reconstruction
        guard ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) else {
            showAlert("Device Not Supported", "Your device does not support scene reconstruction.")
            return
        }
        
        let config = ARWorldTrackingConfiguration()
        config.sceneReconstruction = .mesh
        config.environmentTexturing = .automatic
        
        session.run(config)
        
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
    
    private func showAlert(_ title: String, _ message: String) {
        alertTitle = title
        alertMessage = message
        showingAlert = true
    }
}

#Preview {
    ARScannerView()
} 