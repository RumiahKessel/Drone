//
//  BluetoothConnection.swift
//  Drone
//
//  Created by Sadhika Akula on 12/12/23.
//

import Foundation
import CoreBluetooth

enum ConnectionStatus {
    case connected
    case disconnected
    case scanning
    case connecting
    case error
}


let service: CBUUID = CBUUID(string:"4fafc201-1fb5-459e-8fcc-c5c9c331914b")
let pitchCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e1-4688-b7f5-ea07361b26a8")
let rollCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e2-4688-b7f5-ea07361b26a8")
let yawCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e3-4688-b7f5-ea07361b26a8")
let throttleCharacteristic: CBUUID = CBUUID(string:"beb5483e-36e4-4688-b7f5-ea07361b26a8")
let sliderCharacteristic: CBUUID = CBUUID(string: "beb5483e-36e5-4688-b7f5-ea07361b26a8")

class BluetoothService: NSObject, ObservableObject {
    private var centralManager: CBCentralManager!
    
    var sensorPeripheral: CBPeripheral?
    @Published var peripheralStatus: ConnectionStatus = .disconnected
    @Published var pitchValue: Int = 0
    @Published var throttleValue: Int = 0
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }
    
    func scanForPeripherals() {
        peripheralStatus = .scanning
        centralManager.scanForPeripherals(withServices: [service])
    }
}

extension BluetoothService: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            scanForPeripherals()
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi RSSI: NSNumber) {
        print("Discovered \(peripheral.name!)")
        sensorPeripheral = peripheral
        centralManager.connect(peripheral)
        peripheralStatus = .connecting
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheralStatus = .connected
        
        peripheral.delegate = self
        peripheral.discoverServices([service])
        centralManager.stopScan()
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        peripheralStatus = .disconnected
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        peripheralStatus = .error
        print(error?.localizedDescription ?? "no error")
    }
    
}

extension BluetoothService: CBPeripheralDelegate {
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        for service in peripheral.services ?? [] {
            if service.uuid == sensorPeripheral {
                peripheral.discoverCharacteristics([pitchCharacteristic], for: service)
                peripheral.discoverCharacteristics([throttleCharacteristic], for: service)
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for charecteristic in service.characteristics ?? [] {
            peripheral.setNotifyValue(true, for: charecteristic)
            print("found characteristic, waiting on values.")
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if characteristic.uuid == pitchCharacteristic {
            guard let data = characteristic.value else {
                print("No data received for \(characteristic.uuid.uuidString)")
                return
            }
            let sensorData: Int = data.withUnsafeBytes { $0.pointee }
            pitchValue = sensorData
        }
        if characteristic.uuid == throttleCharacteristic {
            guard let data = characteristic.value else {
                print("No data received for \(characteristic.uuid.uuidString)")
                return
            }
            let sensorData: Int = data.withUnsafeBytes { $0.pointee }
            pitchValue = sensorData
        }
    }
}
