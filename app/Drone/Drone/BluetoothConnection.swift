//
//  BluetoothConnection.swift
//  Drone
//
//  Created by Sadhika Akula on 12/12/23.
//

import Foundation
import CoreBluetooth

// F4833386-80AA-B418-47A1-1A9C6A72FD25
// 6D678691-3DCF-7BF3-2D6C-C3100C932AF3
let droneService: CBUUID = CBUUID(string:"4fafc201-1fb5-459e-8fcc-c5c9c331914b")
let service: CBUUID = CBUUID(string:"4fafc201-1fb5-459e-8fcc-c5c9c331914b")
let pitchCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e1-4688-b7f5-ea07361b26a8")
let rollCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e2-4688-b7f5-ea07361b26a8")
let yawCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e3-4688-b7f5-ea07361b26a8")
let throttleCharacteristic: CBUUID = CBUUID(string:"beb5483e-36e4-4688-b7f5-ea07361b26a8")
let sliderCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e5-4688-b7f5-ea07361b26a8")

class BluetoothService: NSObject, ObservableObject {
    private var centralManager: CBCentralManager?
    var sensorPeripheral: CBPeripheral?
    private var peripherals: [CBPeripheral] = []
    @Published var peripheralConnection : String = ""
    
    override init() {
        super.init()
        self.centralManager = CBCentralManager(delegate: self, queue: .main)
    }
    
    // func scanForPeripherals() {
    //     peripheralStatus = .scanning
    //     self.centralManager?.scanForPeripherals(withServices: .nil)
    // }
}

extension BluetoothService: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            print("Scanning for peripherals")
            self.centralManager?.scanForPeripherals(withServices: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        if peripheral.name == "Ultra Drone" {
            print("Discovered \(peripheral.name!)")
            if !peripherals.contains(peripheral) {
                sensorPeripheral = peripheral
                self.centralManager?.connect(peripheral)
                self.peripheralConnection = peripheral.name ?? "not found"
                print("Successfully discovered!")
            }
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Successfully connected!")
        peripheral.delegate = self
        peripheral.discoverServices([service])
        self.centralManager?.stopScan()
    }
    
   func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
       print("Disconnecting peripheral")
   }
   
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("Failed to connect!")
        print(error?.localizedDescription ?? "no error")
    }
    
}

extension BluetoothService: CBPeripheralDelegate {
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        for service in peripheral.services ?? [] {
            if service.uuid == droneService {
                print("Finding characteristics")
                peripheral.discoverCharacteristics([pitchCharacteristic], for: service)
                peripheral.discoverCharacteristics([throttleCharacteristic], for: service)
                peripheral.discoverCharacteristics([rollCharacteristic], for: service)
                peripheral.discoverCharacteristics([yawCharacteristic], for: service)
                peripheral.discoverCharacteristics([sliderCharacteristic], for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for characteristic in service.characteristics ?? [] {
            print("Found characteristic: ", characteristic.uuid)
        }
    }
    
    private func writeValue(to characteristic: CBUUID, with data: Data) {
       guard let peripheral = sensorPeripheral else {
           print("Peripheral not found")
           return
       }

       guard let service = peripheral.services?.first(where: { $0.uuid == service }) else {
           print("Service not found")
           return
       }

       guard let targetCharacteristic = service.characteristics?.first(where: { $0.uuid == characteristic }) else {
           print("Characteristic not found")
           return
       }
        
       peripheral.writeValue(data, for: targetCharacteristic, type: .withResponse)
        print("Wrote ", data, "to characteristic: ", targetCharacteristic.uuid)
   }
    
    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        if (error != nil) {
            print("Errored")
            return
        }
        print("Successfully wrote back characteristic")
    }
    
    
    // Example function to update pitch characteristic
        func updatePitchCharacteristic(with value: Int) {
            var mutableValue = value
            let stringValue = String(mutableValue)
            if let stringData = stringValue.data(using: .utf8) {
                writeValue(to: pitchCharacteristic, with:  stringData)
            } else {
                print("Failed to convert string to data")
            }
        }

        func updateThrottleCharacteristic(with value: Int) {
            var mutableValue = value
            let stringValue = String(mutableValue)
            if let stringData = stringValue.data(using: .utf8) {
                writeValue(to: throttleCharacteristic, with:  stringData)
            } else {
                print("Failed to convert string to data")
            }
        }

    func updateYawCharacteristic(with value: Int) {
            var mutableValue = value
            let stringValue = String(mutableValue)
            if let stringData = stringValue.data(using: .utf8) {
                writeValue(to: yawCharacteristic, with:  stringData)
            } else {
                print("Failed to convert string to data")
            }
        }

        func updateRollCharacteristic(with value: Int) {
            var mutableValue = value
            let stringValue = String(mutableValue)
            if let stringData = stringValue.data(using: .utf8) {
                writeValue(to: rollCharacteristic, with:  stringData)
            } else {
                print("Failed to convert string to data")
            }
        }

        func updateSliderCharacteristic(with value: Int) {
            var mutableValue = value
            let stringValue = String(mutableValue)
            if let stringData = stringValue.data(using: .utf8) {
                writeValue(to: sliderCharacteristic, with:  stringData)
            } else {
                print("Failed to convert string to data")
            }
        }
    
}
