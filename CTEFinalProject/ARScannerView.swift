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
    
    // PathNode struct for A* pathfinding - moved outside method for broader scope
    private struct PathNode: Hashable {
        let x, z: Int
        
        func hash(into hasher: inout Hasher) {
            hasher.combine(x)
            hasher.combine(z)
        }
        
        static func ==(lhs: PathNode, rhs: PathNode) -> Bool {
            return lhs.x == rhs.x && lhs.z == rhs.z
        }
    }
    
    // Priority queue element for A* pathfinding
    private struct PriorityQueueElement {
        let node: PathNode
        let priority: Float // Lower is better
    }
    
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
        
        // Calculate the floor level (use the lower of the two points)
        let floorLevel = min(startPoint.y, endPoint.y)
        
        // Get all mesh anchors for obstacles
        var meshVertices: [SIMD3<Float>] = []
        var obstacles: [SIMD3<Float>] = []
        
        for anchor in arView.session.currentFrame?.anchors ?? [] where anchor is ARMeshAnchor {
            guard let meshAnchor = anchor as? ARMeshAnchor else { continue }
            let vertices = meshAnchor.geometry.vertices
            let vertexCount = vertices.count
            
            // Sample mesh vertices to identify obstacles
            let strideAmount = 10 // Sample with higher frequency to catch more detail
            for i in stride(from: 0, to: vertexCount, by: strideAmount) {
                let offset = i * MemoryLayout<SIMD3<Float>>.stride
                guard offset < vertices.buffer.length else { break }
                
                let vertexPointer = vertices.buffer.contents().advanced(by: offset)
                guard vertexPointer != nil else { break }
                
                let vertex = vertexPointer.assumingMemoryBound(to: SIMD3<Float>.self).pointee
                let worldPosition = meshAnchor.transform * SIMD4<Float>(vertex.x, vertex.y, vertex.z, 1)
                let point = SIMD3<Float>(worldPosition.x, worldPosition.y, worldPosition.z)
                
                meshVertices.append(point)
                
                // Identify potential obstacles (points above floor level)
                let heightAboveFloor = point.y - floorLevel
                if heightAboveFloor > 0.1 && heightAboveFloor < 2.0 { // Between 10cm and 2m above floor
                    obstacles.append(point)
                }
            }
        }
        
        // Generate path using A* algorithm to avoid obstacles
        let path = findPathAvoiding(from: startPoint, to: endPoint, obstacles: obstacles, floorLevel: floorLevel)
        
        // Create the path visualization
        createPathVisualization(path: path)
    }
    
    private func isPathBlocked(startPoint: SIMD3<Float>, endPoint: SIMD3<Float>, obstacles: [SIMD3<Float>]) -> Bool {
        // Calculate direct path vector (horizontally)
        let pathDirection = normalize(SIMD3<Float>(endPoint.x - startPoint.x, 0, endPoint.z - startPoint.z))
        let pathLength = length(SIMD3<Float>(endPoint.x - startPoint.x, 0, endPoint.z - startPoint.z))
        
        // Check if any obstacle points are in the way
        for point in obstacles {
            // Project the point onto the path line (ignoring Y)
            let toPoint = SIMD3<Float>(point.x - startPoint.x, 0, point.z - startPoint.z)
            let projection = dot(toPoint, pathDirection)
            
            // Check if projection is within the path segment
            if projection >= 0 && projection <= pathLength {
                // Calculate the perpendicular distance to the line
                let perpendicular = toPoint - pathDirection * projection
                let distance = length(perpendicular)
                
                // Consider it blocked if the point is within 20cm of the path
                if distance < 0.2 {
                    return true
                }
            }
        }
        
        return false
    }
    
    // A* pathfinding algorithm
    private func findPathAvoiding(from start: SIMD3<Float>, to end: SIMD3<Float>, obstacles: [SIMD3<Float>], floorLevel: Float) -> [SIMD3<Float>] {
        // Check if there's a direct path without obstacles
        if !isPathBlocked(startPoint: start, endPoint: end, obstacles: obstacles) {
            return [start, end] // Direct path is clear
        }
        
        // Create a grid representation of the space
        let padding: Float = 1.0 // 1 meter padding around the path
        let minX = min(start.x, end.x) - padding
        let maxX = max(start.x, end.x) + padding
        let minZ = min(start.z, end.z) - padding
        let maxZ = max(start.z, end.z) + padding
        
        let cellSize: Float = 0.2 // 20cm grid cells
        let gridSizeX = Int((maxX - minX) / cellSize) + 1
        let gridSizeZ = Int((maxZ - minZ) / cellSize) + 1
        
        // Generate a grid with obstacle information and distance field
        var grid = Array(repeating: Array(repeating: Float.infinity, count: gridSizeZ), count: gridSizeX)
        
        // Calculate distance field from obstacles
        for obstacle in obstacles {
            let gridX = Int((obstacle.x - minX) / cellSize)
            let gridZ = Int((obstacle.z - minZ) / cellSize)
            
            // Update distance field in a radius around the obstacle
            let radius = 5 // 5 cells radius for distance calculation
            for dx in -radius...radius {
                for dz in -radius...radius {
                    let x = gridX + dx
                    let z = gridZ + dz
                    
                    if x >= 0 && x < gridSizeX && z >= 0 && z < gridSizeZ {
                        let distance = sqrt(Float(dx * dx + dz * dz)) * cellSize
                        grid[x][z] = min(grid[x][z], distance)
                    }
                }
            }
        }
        
        // Convert start and end to grid coordinates
        let startGridX = Int((start.x - minX) / cellSize)
        let startGridZ = Int((start.z - minZ) / cellSize)
        let endGridX = Int((end.x - minX) / cellSize)
        let endGridZ = Int((end.z - minZ) / cellSize)
        
        // A* algorithm data structures
        var openList: [PriorityQueueElement] = []
        var cameFrom: [PathNode: PathNode] = [:]
        var gScore: [PathNode: Float] = [:]
        var fScore: [PathNode: Float] = [:]
        var closedSet: Set<PathNode> = []
        
        let startNode = PathNode(x: startGridX, z: startGridZ)
        let endNode = PathNode(x: endGridX, z: endGridZ)
        
        // Initialize with start node
        gScore[startNode] = 0
        fScore[startNode] = heuristic(startNode, endNode)
        openList.append(PriorityQueueElement(node: startNode, priority: fScore[startNode]!))
        
        // The directions we can move in the grid
        let directions = [
            (x: 0, z: 1), (x: 1, z: 0), (x: 0, z: -1), (x: -1, z: 0), // Cardinal
            (x: 1, z: 1), (x: 1, z: -1), (x: -1, z: -1), (x: -1, z: 1) // Diagonal
        ]
        
        // A* search
        while !openList.isEmpty {
            // Find node with lowest f score
            openList.sort { $0.priority < $1.priority }
            let current = openList.removeFirst().node
            
            // Check if we reached the target
            if current == endNode {
                return reconstructPath(cameFrom: cameFrom, current: current, start: startNode, end: endNode, minX: minX, minZ: minZ, cellSize: cellSize, floorLevel: floorLevel)
            }
            
            closedSet.insert(current)
            
            // Check neighbors
            for direction in directions {
                let neighborX = current.x + direction.x
                let neighborZ = current.z + direction.z
                
                // Skip if out of bounds
                if neighborX < 0 || neighborX >= gridSizeX || neighborZ < 0 || neighborZ >= gridSizeZ {
                    continue
                }
                
                let neighbor = PathNode(x: neighborX, z: neighborZ)
                
                // Skip if in closed set
                if closedSet.contains(neighbor) {
                    continue
                }
                
                // Calculate cost based on distance and obstacle proximity
                let baseCost = direction.x != 0 && direction.z != 0 ? 1.414 : 1.0 // Diagonal is sqrt(2)
                let obstacleDistance = grid[neighborX][neighborZ]
                
                // Calculate obstacle cost separately
                let obstacleCost: Float
                if obstacleDistance == Float.infinity {
                    obstacleCost = 0
                } else {
                    obstacleCost = 1.0 / (obstacleDistance + 0.1)
                }
                
                // Calculate weighted costs separately
                let pathLengthCost = baseCost * 0.7
                let obstacleAvoidanceCost = obstacleCost * 0.3
                
                // Combine costs
                let tentativeGScore = gScore[current]! + (Float(pathLengthCost) + obstacleAvoidanceCost)
                
                // Check if this path is better
                if !gScore.keys.contains(neighbor) || tentativeGScore < gScore[neighbor]! {
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentativeGScore
                    fScore[neighbor] = tentativeGScore + heuristic(neighbor, endNode)
                    
                    // Check if node is already in open list
                    var isInOpenList = false
                    for element in openList {
                        if element.node == neighbor {
                            isInOpenList = true
                            break
                        }
                    }
                    
                    // Add to open list if not already there
                    if !isInOpenList {
                        let priority = fScore[neighbor]!
                        openList.append(PriorityQueueElement(node: neighbor, priority: priority))
                    }
                }
            }
        }
        
        // If no path was found, fall back to simpler approach
        return findSimplePathAroundObstacles(start: start, end: end, obstacles: obstacles, floorLevel: floorLevel)
    }
    
    // Improved heuristic function for A* (Euclidean distance)
    private func heuristic(_ a: PathNode, _ b: PathNode) -> Float {
        let dx = Float(a.x - b.x)
        let dz = Float(a.z - b.z)
        return sqrt(dx * dx + dz * dz)
    }
    
    // Convert grid path to world coordinates
    private func reconstructPath(cameFrom: [PathNode: PathNode], current: PathNode, start: PathNode, end: PathNode, minX: Float, minZ: Float, cellSize: Float, floorLevel: Float) -> [SIMD3<Float>] {
        var path: [SIMD3<Float>] = []
        
        // Convert end position to world coordinates
        let endWorldX = minX + Float(end.x) * cellSize + cellSize/2
        let endWorldZ = minZ + Float(end.z) * cellSize + cellSize/2
        path.append(SIMD3<Float>(endWorldX, floorLevel, endWorldZ))
        
        // Reconstruct path backwards
        var currentNode = current
        while let prevNode = cameFrom[currentNode] {
            let worldX = minX + Float(prevNode.x) * cellSize + cellSize/2
            let worldZ = minZ + Float(prevNode.z) * cellSize + cellSize/2
            path.append(SIMD3<Float>(worldX, floorLevel, worldZ))
            currentNode = prevNode
        }
        
        // Add the actual start point at the beginning
        path = path.reversed()
        path[0] = savedPoints[0]
        
        // Add the actual end point at the end
        path[path.count-1] = savedPoints[1]
        
        // Optimize the path by removing unnecessary waypoints (path smoothing)
        return path
    }
    
    // Simplify path by removing unnecessary waypoints
    private func simplifyPath(path: [SIMD3<Float>], obstacles: [SIMD3<Float>]) -> [SIMD3<Float>] {
        guard path.count > 2 else { return path }
        
        var result: [SIMD3<Float>] = [path[0]]
        var i = 0
        
        while i < path.count - 1 {
            var furthestVisible = i + 1
            
            // Find furthest point that is directly visible from current point
            for j in (i + 2)..<path.count {
                if !isPathBlocked(startPoint: path[i], endPoint: path[j], obstacles: obstacles) {
                    furthestVisible = j
                }
            }
            
            result.append(path[furthestVisible])
            i = furthestVisible
        }
        
        return result
    }
    
    // Fallback simpler path finding approach
    private func findSimplePathAroundObstacles(start: SIMD3<Float>, end: SIMD3<Float>, obstacles: [SIMD3<Float>], floorLevel: Float) -> [SIMD3<Float>] {
        // Calculate direct path vector
        let pathVector = SIMD3<Float>(end.x - start.x, 0, end.z - start.z)
        let pathLength = length(pathVector)
        let pathDirection = normalize(pathVector)
        
        // Create a perpendicular vector to the path
        let perpendicularDirection = SIMD3<Float>(-pathDirection.z, 0, pathDirection.x)
        
        // Try different waypoints with increasing offset
        let offsets: [Float] = [0.5, 0.8, 1.2, 1.8]
        let midPoint = start + pathDirection * (pathLength * 0.5)
        
        for offset in offsets {
            // Try right side
            let rightWaypoint = midPoint + perpendicularDirection * offset
            let rightWaypointAtFloor = SIMD3<Float>(rightWaypoint.x, floorLevel, rightWaypoint.z)
            
            if !isPathBlocked(startPoint: start, endPoint: rightWaypointAtFloor, obstacles: obstacles) &&
               !isPathBlocked(startPoint: rightWaypointAtFloor, endPoint: end, obstacles: obstacles) {
                return [start, rightWaypointAtFloor, end]
            }
            
            // Try left side
            let leftWaypoint = midPoint - perpendicularDirection * offset
            let leftWaypointAtFloor = SIMD3<Float>(leftWaypoint.x, floorLevel, leftWaypoint.z)
            
            if !isPathBlocked(startPoint: start, endPoint: leftWaypointAtFloor, obstacles: obstacles) &&
               !isPathBlocked(startPoint: leftWaypointAtFloor, endPoint: end, obstacles: obstacles) {
                return [start, leftWaypointAtFloor, end]
            }
        }
        
        // If none of the above worked, try with two waypoints
        let pathThirdPoint = start + pathDirection * (pathLength * 0.33)
        let pathTwoThirdsPoint = start + pathDirection * (pathLength * 0.66)
        
        for offset in offsets {
            // Try right-left
            let rightWaypoint = pathThirdPoint + perpendicularDirection * offset
            let leftWaypoint = pathTwoThirdsPoint - perpendicularDirection * offset
            
            let rightWaypointAtFloor = SIMD3<Float>(rightWaypoint.x, floorLevel, rightWaypoint.z)
            let leftWaypointAtFloor = SIMD3<Float>(leftWaypoint.x, floorLevel, leftWaypoint.z)
            
            if !isPathBlocked(startPoint: start, endPoint: rightWaypointAtFloor, obstacles: obstacles) &&
               !isPathBlocked(startPoint: rightWaypointAtFloor, endPoint: leftWaypointAtFloor, obstacles: obstacles) &&
               !isPathBlocked(startPoint: leftWaypointAtFloor, endPoint: end, obstacles: obstacles) {
                return [start, rightWaypointAtFloor, leftWaypointAtFloor, end]
            }
            
            // Try left-right
            let leftFirstWaypoint = pathThirdPoint - perpendicularDirection * offset
            let rightSecondWaypoint = pathTwoThirdsPoint + perpendicularDirection * offset
            
            let leftFirstWaypointAtFloor = SIMD3<Float>(leftFirstWaypoint.x, floorLevel, leftFirstWaypoint.z)
            let rightSecondWaypointAtFloor = SIMD3<Float>(rightSecondWaypoint.x, floorLevel, rightSecondWaypoint.z)
            
            if !isPathBlocked(startPoint: start, endPoint: leftFirstWaypointAtFloor, obstacles: obstacles) &&
               !isPathBlocked(startPoint: leftFirstWaypointAtFloor, endPoint: rightSecondWaypointAtFloor, obstacles: obstacles) &&
               !isPathBlocked(startPoint: rightSecondWaypointAtFloor, endPoint: end, obstacles: obstacles) {
                return [start, leftFirstWaypointAtFloor, rightSecondWaypointAtFloor, end]
            }
        }
        
        // Last resort - return a very wide berth around potential obstacles
        let maxOffset: Float = 2.0
        let farRightWaypoint = SIMD3<Float>(midPoint.x + perpendicularDirection.x * maxOffset, floorLevel, midPoint.z + perpendicularDirection.z * maxOffset)
        
        return [start, farRightWaypoint, end]
    }
    
    private func createPathVisualization(path: [SIMD3<Float>]) {
        guard let arView = arView else { return }
        
        let pathAnchor = AnchorEntity(world: .zero)
        
        // Create a cylinder for each segment of the path
        for i in 0..<path.count - 1 {
            let start = path[i]
            let end = path[i + 1]
            
            // Calculate horizontal distance only (for cylinder length)
            let horizontalDistance = length(SIMD3<Float>(end.x - start.x, 0, end.z - start.z))
            
            // Create a cylinder for the path segment
            let cylinder = MeshResource.generateCylinder(height: horizontalDistance, radius: 0.005)
            let material = SimpleMaterial(color: .green, isMetallic: false)
            let pathEntity = ModelEntity(mesh: cylinder, materials: [material])
            
            // Calculate the midpoint of the segment (at floor level)
            let midPoint = SIMD3<Float>(
                (start.x + end.x) / 2,
                (start.y + end.y) / 2,
                (start.z + end.z) / 2
            )
            
            // Calculate the direction vector (keeping it horizontal)
            let direction = normalize(SIMD3<Float>(end.x - start.x, 0, end.z - start.z))
            
            // Create a rotation that aligns the cylinder with the path segment
            // We need to rotate from the default up vector (y-axis) to our path direction
            let transform = Transform(
                scale: .one,
                rotation: simd_quatf(from: SIMD3<Float>(0, 1, 0), to: direction),
                translation: midPoint
            )
            
            pathEntity.transform = transform
            pathAnchor.addChild(pathEntity)
        }
        
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
