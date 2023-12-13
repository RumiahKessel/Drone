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
    @State private var leftFinal: CGPoint = .zero
    @State private var rightFinal: CGPoint = .zero
    @State private var leftFinalText: String = ""
    @State private var rightFinalText: String = ""
    
    // Joystick Controls:
    @State private var location: CGPoint = CGPoint(x: 50, y: UIScreen.main.bounds.height / 2)
    @State private var innerCircleLocation: CGPoint = CGPoint(x: 50, y: UIScreen.main.bounds.height / 2)
    @GestureState private var fingerLocation: CGPoint? = nil

    @State private var rightLocation: CGPoint = CGPoint(x: 50, y: UIScreen.main.bounds.height / 2)
    @State private var rightInnerCircleLocation: CGPoint = CGPoint(x: 50, y: UIScreen.main.bounds.height / 2)
    @GestureState private var rightFingerLocation: CGPoint? = nil

    // Slider:
    @State private var sliderValue: Double = 2.0
    
    // Bluetooth
    //@StateObject var service = BluetoothService()
    @ObservedObject private var bluetoothViewModel = BluetoothService()
    
    // Timer
    @State private var timer: Timer?
    private let updateInterval: TimeInterval = 0.1

    private var bigCircleRadius: CGFloat = 100 // Adjust the radius of the blue circle
    
    var body: some View {
        ZStack() {
                    
            MapView(initialCoordinate: CLLocationCoordinate2D(latitude: 37.875, longitude: -122.2578), zoomLevel: 0.001)
                .edgesIgnoringSafeArea(.all)

            HStack() {
                Text(bluetoothViewModel.peripheralConnection)
                    .background(.white)
                    .font(.headline)
                    .position(x: 30, y: 400)

                HStack() {  
                    ZStack() {
                        Circle()
                            .foregroundColor(.gray.opacity(0.5))
                            .frame(width: bigCircleRadius * 2, height: bigCircleRadius * 2)
                            .position(location)
                        
                        // Smaller circle (green circle)
                        Circle()
                            .foregroundColor(.white)
                            .frame(width: 50, height: 50)
                            .position(innerCircleLocation)
                            .gesture(fingerDrag)
                    }
                    Text(leftFinalText)
                       .font(.headline)
                       .foregroundColor(.white)
                       .padding()
                       .frame(minWidth: 200)
                       .background(Color.blue)
                       .cornerRadius(10)
                       .position(x: -50, y: 50)
                }.onChange(of: innerCircleLocation) {
                    leftFinal = CGPoint(x: (innerCircleLocation.x - 50) * 10, y: (innerCircleLocation.y - UIScreen.main.bounds.height / 2) * -10)
                    updateText(final: leftFinal, textBinding: $leftFinalText)
                }
                
                VStack() {
                    Slider(value: $sliderValue, in: 0...10, step: 0.5, onEditingChanged: { editing in
                            if !editing {
                                bluetoothViewModel.updateSliderCharacteristic(with: Int(sliderValue))

                            }
                        })
                    Text("\(String(format: "%.1f", sliderValue))")
                }.frame(width: 200, height: 100)
                    .background(Color.gray)
                    .cornerRadius(10)
                    .foregroundColor(.white)
                    .padding()
                    .position(x: 25, y: 120)
                
                
                HStack() {
                    ZStack() {
                        Circle()
                            .foregroundColor(.gray.opacity(0.5))
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
                    Text(rightFinalText)
                        .font(.headline)
                        .foregroundColor(.white)
                        .padding()
                        .frame(minWidth: 200)
                        .background(Color.blue)
                        .cornerRadius(10)
                        .position(x: -85, y: 50)
                }.onChange(of: rightInnerCircleLocation) {
                    rightFinal = CGPoint(x: (rightInnerCircleLocation.x - 50) * 10, y: (rightInnerCircleLocation.y - UIScreen.main.bounds.height / 2) * -10)
                    updateText(final: rightFinal, textBinding: $rightFinalText)
                    
                }
            
               
              }
            }
            .onAppear {
            // Start the timer when the view appears
                startUpdateTimer()
            }
            .onDisappear {
                // Stop the timer when the view disappears
                stopUpdateTimer()
            }
        
        }

        // Function to start the update timer
    private func startUpdateTimer() {
        timer = Timer.scheduledTimer(withTimeInterval: updateInterval, repeats: true) { _ in
            bluetoothViewModel.updatePitchCharacteristic(with: Int(rightFinal.y))
            bluetoothViewModel.updateThrottleCharacteristic(with: Int(leftFinal.y))
            bluetoothViewModel.updateYawCharacteristic(with: Int(leftFinal.x))
            bluetoothViewModel.updateRollCharacteristic(with: Int(rightFinal.x))
        }
    }

    // Function to stop the update timer
    private func stopUpdateTimer() {
        timer?.invalidate()
        timer = nil
    }

    private func updateBluetoothData() {
            bluetoothViewModel.updateThrottleCharacteristic(with: Int(leftFinal.y))
            bluetoothViewModel.updatePitchCharacteristic(with: Int(rightFinal.y))
            bluetoothViewModel.updateYawCharacteristic(with: Int(leftFinal.x))
            bluetoothViewModel.updateRollCharacteristic(with: Int(rightFinal.x))
            bluetoothViewModel.updateSliderCharacteristic(with: Int(sliderValue))
        }
    
     func updateText(final: CGPoint, textBinding: Binding<String>) {
           let formattedX = String(format: "%.0f", final.x)
           let formattedY = String(format: "%.0f", final.y)
           textBinding.wrappedValue = "X: \(formattedX), Y: \(formattedY)"
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
    
