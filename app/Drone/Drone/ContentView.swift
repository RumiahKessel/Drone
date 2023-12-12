//
//  ContentView.swift
//  Drone
//
//  Created by Sadhika Akula on 12/11/23.
//

import SwiftUI
import MapKit
import CoreGraphics

struct ContentView: View {
    
    @State private var isLandscape = UIDevice.current.orientation.isLandscape

//    // Directions:
    @State private var throttle: Double = 0.0
    @State private var yaw: Double = 0.0
    @State private var pitch: Double = 0.0
    @State private var roll: Double = 0.0
    
    @State private var leftxDist: Double = 0.0
    @State private var rightxDist: Double = 0.0
    @State private var leftyDist: Double = 0.0
    @State private var rightyDist: Double = 0.0
    
    // Joystick Controls:
    @State private var leftLoc: CGPoint = CGPoint(x: 180, y: UIScreen.main.bounds.height / 2)
    @State private var leftFinalLoc: CGPoint = CGPoint(x: 47, y: UIScreen.main.bounds.height / 2)
    @State private var leftInnerLoc: CGPoint = CGPoint(x: 47, y: UIScreen.main.bounds.height / 2)
    
    @State private var rightLoc: CGPoint = CGPoint(x: 180, y: UIScreen.main.bounds.height / 2)
    @State private var rightFinalLoc: CGPoint = CGPoint(x: 47, y: UIScreen.main.bounds.height / 2)
    @State private var rightInnerLoc: CGPoint = CGPoint(x: 47, y: UIScreen.main.bounds.height / 2)
    
    @GestureState private var leftFingerLoc: CGPoint = CGPoint(x: 48, y: UIScreen.main.bounds.height / 2)
    @GestureState private var rightFingerLoc: CGPoint = CGPoint(x: 48, y: UIScreen.main.bounds.height / 2)
    
    private var bigCircleRadius: CGFloat = 100 // Adjust the radius of the blue circle
    
    var body: some View {
        ZStack() {
                    
            MapView(initialCoordinate: CLLocationCoordinate2D(latitude: 37.875, longitude: -122.2578), zoomLevel: 0.001)
                .edgesIgnoringSafeArea(.all)
        
            HStack() {
                // Larger circle
                Circle()
                    .foregroundColor(.gray.opacity(0.4))
                .frame(width: bigCircleRadius * 2, height: bigCircleRadius * 2)
                .position(leftLoc)

                // Smaller circle
                Circle()
                .foregroundColor(.white)
                .frame(width: 50, height: 50)
                .position(leftInnerLoc)
                .gesture(leftFingerDrag)
                        
                // Angle text
                Text(leftAngleText)
                    .font(.headline)
                    .foregroundColor(.white)
                    .padding()
                    .background(Color.purple)
                    .cornerRadius(10)
                    .position(x: -90, y: 50)
                
                
                
                
                // Larger circle
                Circle()
                .foregroundColor(.gray.opacity(0.4))
                .frame(width: bigCircleRadius * 2, height: bigCircleRadius * 2)
                .position(rightLoc)

                // Smaller circle
                Circle()
                .foregroundColor(.white)
                .frame(width: 50, height: 50)
                .position(rightInnerLoc)
                .gesture(rightFingerDrag)
                        
                // Angle text
                Text(rightAngleText)
                    .font(.headline)
                    .foregroundColor(.white)
                    .padding()
                    .background(Color.purple)
                    .cornerRadius(10)
                    .position(x: -80, y: 50)
            }
        }
    }

        
        var leftFingerDrag: some Gesture {
            DragGesture()
                .onChanged { value in
                    // Calculate the distance between the finger location and the center of the blue circle
                    let distance = sqrt(pow(value.location.x - leftLoc.x, 2) + pow(value.location.y - leftLoc.y, 2))
                    
                    // Calculate the angle between the center of the blue circle and the finger location
                    let angle = atan2(value.location.y - leftLoc.y, value.location.x - leftLoc.x)
                    
                    // Clamp the distance within the blue circle
                    let clampedDistance = min(distance, bigCircleRadius)
                                        
                    // Calculate the new leftLoc at the edge of the blue circle: -130, -20
                    let newX = (leftLoc.x - 130) + cos(angle) * clampedDistance
                    let newY = (leftLoc.y - 3) + sin(angle) * clampedDistance
                    
                    leftInnerLoc = CGPoint(x: newX, y: newY)
                }
                .updating($leftFingerLoc) { (value, leftFingerLoc, transaction) in
                    leftFingerLoc = value.location
                }
                .onEnded { value in
                    // Snap the smaller circle to regular position
                    leftInnerLoc = leftFinalLoc
                }
        }


        var rightFingerDrag: some Gesture {
            DragGesture()
                .onChanged { value in
                    // Calculate the distance between the finger location and the center of the blue circle
                    let distance = sqrt(pow(value.location.x - rightLoc.x, 2) + pow(value.location.y - rightLoc.y, 2))
                    
                    // Calculate the angle between the center of the blue circle and the finger rightLoc
                    
                    let angle = atan2(value.location.y - rightLoc.y, value.location.x - rightLoc.x)
                    
                    // Calculate the maximum allowable distance within the blue circle
                    let maxDistance = bigCircleRadius
                    
                    // Clamp the distance within the blue circle
                    let clampedDistance = min(distance, maxDistance)
                    
                    // Calculate the new location at the edge of the blue circle
                    let newX = (rightLoc.x - 130) + cos(angle) * clampedDistance
                    let newY = (rightLoc.y - 3) + sin(angle) * clampedDistance
                    
                    rightInnerLoc = CGPoint(x: newX, y: newY)
                }
                .updating($rightFingerLoc) { (value, rightFingerLoc, transaction) in
                    rightFingerLoc = value.location
                }
                .onEnded { value in
                    // Snap the smaller circle to the center of the larger circle
                    rightInnerLoc = rightFinalLoc
                }
        }

        var leftAngleText: String {
            let formattedX = String(format: "%.0f", (leftFingerLoc.x - leftLoc.x) * 10)
            let formattedY = String(format: "%.0f", (-1 * leftFingerLoc.y + leftLoc.y) * 10)
            return "X: \(formattedX), Y: \(formattedY)"
        }
        var rightAngleText: String {
            let formattedX = String(format: "%.0f", (rightFingerLoc.x - rightLoc.x) * 10)
            let formattedY = String(format: "%.0f", -1 * (rightFingerLoc.y + rightLoc.y) * 10)
            return "X: \(formattedX), Y: \(formattedY)"
        }
}

#Preview {
    ContentView()
}

struct MapView: UIViewRepresentable {
    let initialCoordinate: CLLocationCoordinate2D
    let zoomLevel: CLLocationDegrees

   func makeUIView(context: Context) -> MKMapView {
       let mapView = MKMapView()
       let region = MKCoordinateRegion(center: initialCoordinate, span: MKCoordinateSpan(latitudeDelta: zoomLevel, longitudeDelta: zoomLevel))
       mapView.setRegion(region, animated: true)
       return mapView
   }
    func updateUIView(_ uiView: MKMapView, context: Context) {
        // Update the map view if needed
    }
}


struct MyApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
