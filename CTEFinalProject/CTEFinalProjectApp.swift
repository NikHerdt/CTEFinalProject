//
//  CTEFinalProjectApp.swift
//  CTEFinalProject
//
//  Created by Nikhil Herdt on 4/8/25.
//

import SwiftUI
import ARKit

class AppDelegate: NSObject, UIApplicationDelegate {
    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey : Any]? = nil) -> Bool {
        // Check AR capabilities
        checkARCapabilities()
        return true
    }
    
    private func checkARCapabilities() {
        guard ARWorldTrackingConfiguration.isSupported else {
            print("AR World Tracking is not supported on this device")
            return
        }
        
        // Check for scene reconstruction capabilities
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            print("Scene reconstruction is supported")
        } else {
            print("Scene reconstruction is not supported on this device")
        }
    }
}

@main
struct CTEFinalProjectApp: App {
    @UIApplicationDelegateAdaptor(AppDelegate.self) var appDelegate
    let persistenceController = PersistenceController.shared

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(\.managedObjectContext, persistenceController.container.viewContext)
        }
    }
}
