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
    @Published var pitchValue: Int = 0
    @Published var throttleValue: Int = 0
    
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
            self.centralManager?.scanForPeripherals(withServices: nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        if peripheral.name == "Super Drone" {
            print(peripheral.identifier)
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
        print(peripheral.delegate)
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
            print(sensorPeripheral)
            print(service)
            if service.uuid == sensorPeripheral {
                print("Finding characteristics")
                peripheral.discoverCharacteristics([pitchCharacteristic], for: service)
                peripheral.discoverCharacteristics([throttleCharacteristic], for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for charecteristic in service.characteristics ?? [] {
            peripheral.setNotifyValue(true, for: charecteristic)
            print("Found characteristic, waiting on values.")
        }
    }
    
//    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
//        if characteristic.uuid == pitchCharacteristic {
//            guard let data = characteristic.value else {
//                print("No data received for \(characteristic.uuid.uuidString)")
//                return
//            }
//            let sensorData: Int = data.withUnsafeBytes { $0.pointee }
//            pitchValue = sensorData
//        }
//        if characteristic.uuid == throttleCharacteristic {
//            guard let data = characteristic.value else {
//                print("No data received for \(characteristic.uuid.uuidString)")
//                return
//            }
//            let sensorData: Int = data.withUnsafeBytes { $0.pointee }
//            throttleValue = sensorData
//        }
//    }
    
}
