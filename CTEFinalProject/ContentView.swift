//
//  ContentView.swift
//  CTEFinalProject
//
//  Created by Nikhil Herdt on 4/8/25.
//

import SwiftUI
import ARKit
import RealityKit

struct ContentView: View {
    var body: some View {
        NavigationView {
            VStack {
                Text("AR Mesh Scanner")
                    .font(.largeTitle)
                    .fontWeight(.bold)
                    .padding()
                
                Image(systemName: "cube.transparent")
                    .resizable()
                    .scaledToFit()
                    .frame(width: 100, height: 100)
                    .foregroundColor(.blue)
                    .padding()
                
                Text("Continuously scan and visualize real-world objects as 3D meshes")
                    .multilineTextAlignment(.center)
                    .padding()
                
                NavigationLink(destination: ARScannerView()) {
                    Text("Start Scanning")
                        .font(.headline)
                        .foregroundColor(.white)
                        .padding()
                        .background(Color.blue)
                        .cornerRadius(10)
                }
                .padding()
            }
            .navigationTitle("AR Mesh Scanner")
        }
    }
}

#Preview {
    ContentView()
}
