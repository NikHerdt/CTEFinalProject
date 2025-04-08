//
//  CTEFinalProjectApp.swift
//  CTEFinalProject
//
//  Created by Nikhil Herdt on 4/8/25.
//

import SwiftUI

@main
struct CTEFinalProjectApp: App {
    let persistenceController = PersistenceController.shared

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(\.managedObjectContext, persistenceController.container.viewContext)
        }
    }
}
