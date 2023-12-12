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

   // Directions:
    @State private var leftxDist: Double = 0.0
    @State private var rightxDist: Double = 0.0
    @State private var leftyDist: Double = 0.0
    @State private var rightyDist: Double = 0.0
    
    // Joystick Controls:
    @State private var location: CGPoint = CGPoint(x: 180, y: UIScreen.main.bounds.height / 2)
    @State private var innerCircleLocation: CGPoint = CGPoint(x: 180, y: UIScreen.main.bounds.height / 2)
    @GestureState private var fingerLocation: CGPoint? = nil

    @State private var rightLocation: CGPoint = CGPoint(x: 180, y: UIScreen.main.bounds.height / 2)
    @State private var rightInnerCircleLocation: CGPoint = CGPoint(x: 180, y: UIScreen.main.bounds.height / 2)
    @GestureState private var rightFingerLocation: CGPoint? = nil

    private var bigCircleRadius: CGFloat = 100 // Adjust the radius of the blue circle
    
    var body: some View {
        ZStack() {
                    
            MapView(initialCoordinate: CLLocationCoordinate2D(latitude: 37.875, longitude: -122.2578), zoomLevel: 0.001)
                .edgesIgnoringSafeArea(.all)

            HStack(spacing: 100) {

                HStack() {  
                    ZStack() {
                        Circle()
                            .foregroundColor(.gray.opacity(0.4))
                            .frame(width: bigCircleRadius * 2, height: bigCircleRadius * 2)
                            .position(location)
                        
                        // Smaller circle (green circle)
                        Circle()
                            .foregroundColor(.white)
                            .frame(width: 50, height: 50)
                            .position(innerCircleLocation)
                            .gesture(fingerDrag)
                    }
                    Text(angleText)
                       .font(.headline)
                       .foregroundColor(.white)
                       .padding()
                       .background(Color.purple)
                       .cornerRadius(10)
                       .position(x: -90, y: 50)
                }
                
                HStack() {
                    ZStack() {
                        Circle()
                            .foregroundColor(.gray.opacity(0.4))
                            .frame(width: bigCircleRadius * 2, height: bigCircleRadius * 2)
                            .position(rightLocation)
                        
                        // Smaller circle (green circle)
                        Circle()
                            .foregroundColor(.white)
                            .frame(width: 50, height: 50)
                            .position(rightInnerCircleLocation)
                            .gesture(rightFingerDrag)
                    }
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
        }

    var fingerDrag: some Gesture {
        DragGesture()
            .onChanged { value in
                let distance = sqrt(pow(value.location.x - location.x, 2) + pow(value.location.y - location.y, 2))
                let angle = atan2(value.location.y - location.y, value.location.x - location.x)
                let maxDistance = bigCircleRadius
                let clampedDistance = min(distance, maxDistance)
                let newX = location.x + cos(angle) * clampedDistance
                let newY = location.y + sin(angle) * clampedDistance
                innerCircleLocation = CGPoint(x: newX, y: newY)
            }
            .updating($fingerLocation) { (value, fingerLocation, transaction) in
                fingerLocation = value.location
            }
            .onEnded { value in
                let center = location
                innerCircleLocation = center
            }
    }

    var rightFingerDrag: some Gesture {
        DragGesture()
            .onChanged { value in
                let distance = sqrt(pow(value.location.x - rightLocation.x, 2) + pow(value.location.y - rightLocation.y, 2))
                let angle = atan2(value.location.y - rightLocation.y, value.location.x - rightLocation.x)
                let maxDistance = bigCircleRadius
                let clampedDistance = min(distance, maxDistance)
                let newX = rightLocation.x + cos(angle) * clampedDistance
                let newY = rightLocation.y + sin(angle) * clampedDistance
                rightInnerCircleLocation = CGPoint(x: newX, y: newY)
            }
            .updating($rightFingerLocation) { (value, rightFingerLocation, transaction) in
                rightFingerLocation = value.location
            }
            .onEnded { value in
                let center = rightLocation
                rightInnerCircleLocation = center
            }
    }


        var angleText: String {
            let formattedX = String(format: "%.0f", (innerCircleLocation.x))
            let formattedY = String(format: "%.0f", (innerCircleLocation.y))
            return "locX: \(formattedX), locY: \(formattedY)"
        }

        var rightAngleText: String {
            let formattedX = String(format: "%.0f", (rightInnerCircleLocation.x))
            let formattedY = String(format: "%.0f", (rightInnerCircleLocation.y))
            return "locX: \(formattedX), locY: \(formattedY)"
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
    
