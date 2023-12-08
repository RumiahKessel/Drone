"""

RECORD ACCELEROMETER MEASUREMENTS FOR ACCELEROMETER CALIBRATION 
VIA MAGNETO

Code By: Michael Wrona
Created: 17 Aug 2021

This code reads acceleroemter measurements in G's over a serial connection 
and logs them to a tab-delimited text file. It also pauses between each 
reading and averages a few measurements.

"""

import os
import math
import pandas
import serial


# global variables
MAX_MEAS = 200  # max number of readings in the session, so that we don't create an infinite loop
AVG_MEAS = 25  # for each reading, take this many measurements and average them
SER_PORT = '/dev/cu.usbserial-14210'  # serial port the device is connected to
SER_BAUD = 115200  # serial port baud rate
ACC_FILENAME = os.path.join(os.getcwd(), 'acc_data_1.txt')  # output file
MAG_FILENAME = os.path.join(os.getcwd(), 'mag_data_1.txt')  # output file
# # Function to find the serial port
# def find_serial_port():
#     ports = serial.tools.list_ports.comports()
#     for port, _, _ in ports:
#         if "usbserial" in port.lower():
#             return port
#     return None

# # Find the serial port dynamically
# SER_PORT = find_serial_port()


class SerialPort:
    """Create and read data from a serial port.

    Attributes:
        read(**kwargs): Read and decode data string from serial port.
    """

    def __init__(self, port, baud=9600):
        """Create and read serial data.

        Args:
            port (str): Serial port name. Example: 'COM4'
            baud (int): Serial baud rate, default 9600.
        """
        if isinstance(port, str) == False:
            raise TypeError('port must be a string.')

        if isinstance(baud, int) == False:
            raise TypeError('Baud rate must be an integer.')

        self.port = port
        self.baud = baud

        # Initialize serial connection
        self.ser = serial.Serial(
            self.port, self.baud, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
        self.ser.flushInput()
        self.ser.flushOutput()

    def Read(self, clean_end=True) -> str:
        """
        Read and decode data string from serial port.

        Args:
            clean_end (bool): Strip '\\r' and '\\n' characters from string. Common if used Serial.println() Arduino function. Default true

        Returns:
            (str): utf-8 decoded message.
        """
        self.ser.flushInput()
        bytesToRead = self.ser.readline()
        decodedMsg = bytesToRead.decode('utf-8')

        if clean_end == True:
            decodedMsg = decodedMsg.rstrip()  # Strip extra chars at the end

        return decodedMsg

    def Close(self) -> None:
        """Close serial connection."""
        self.ser.close()


def RecordDataPt(ser: SerialPort) -> tuple:
    """Record data from serial port and return averaged result."""
    # do a few readings and average the result
    ax = ay = az = 0.0
    mx = my = mz = 0.0

    # read data
    measurements_count = 0

    try:
        flag = False
        while True:
            data = ser.Read().split(',')
            if (len(data) == 8 and data[0] == 'Start' and data[7] == 'End'):
                acc_data = data[1:4]
                ax_now = float(acc_data[0])
                ay_now = float(acc_data[1])
                az_now = float(acc_data[2])

                ax += ax_now
                ay += ay_now
                az += az_now

                mag_data = data[4:7]
                mx_now = float(mag_data[0])
                my_now = float(mag_data[1])
                mz_now = float(mag_data[2])

                mx += mx_now
                my += my_now
                mz += mz_now

                measurements_count += 1

            if measurements_count >= AVG_MEAS:
                break

    except Exception as e:
        print("Exception:", e)
        ser.Close()
        raise SystemExit("[ERROR]: Error reading serial connection.")

    return (ax / AVG_MEAS, ay / AVG_MEAS, az / AVG_MEAS, mx/AVG_MEAS, my/AVG_MEAS, mz/AVG_MEAS)


def List2DelimFile(mylist: list, filename: str, delimiter: str = ',', f_mode='a') -> None:
    """Convert list to Pandas dataframe, then save as a text file."""
    df = pandas.DataFrame(mylist)
    df.to_csv(
        filename,  # path and filename
        sep=delimiter,
        mode=f_mode,
        header=False,  # no col. labels
        index=False  # no row numbers
    )


def main():
    ser = SerialPort(SER_PORT, baud=SER_BAUD)
    accel_data = []  # data list
    mag_data =[]

    print('[INFO]: Place sensor level and stationary on desk.')
    input('[INPUT]: Press any key to continue...')
    # take measurements
    for _ in range(MAX_MEAS):
        user = input(
            '[INPUT]: Ready for measurement? Type anything to measure or \'q\' to save and quit: ').lower()
        if user == 'q':
            # save, then quit
            print('[INFO]: Saving data and exiting...')
            List2DelimFile(accel_data, ACC_FILENAME, delimiter='\t')
            List2DelimFile(mag_data, MAG_FILENAME, delimiter='\t')
            ser.Close()
            print('[INFO]: Done!')
            return
        else:
            # record data to list
            ax, ay, az, mx, my, mz = RecordDataPt(ser)
            accel = math.sqrt(ax**2 + ay**2 + az**2)
            magn = math.sqrt(mx**2 + my**2 + mz**2)
            print('[INFO]: ACCEL Avgd Readings: {:.4f}, {:.4f}, {:.4f} Magnitude: {:.4f}'.format(
                ax, ay, az, accel))
            print('[INFO]: MAG Avgd Readings: {:.4f}, {:.4f}, {:.4f} Magnitude: {:.4f}'.format(
                mx, my, mz, magn))
            accel_data.append([ax, ay, az])
            mag_data.append([mx, my, mz])

    # save once max is reached
    print('[WARNING]: Reached max. number of datapoints, saving file...')
    List2DelimFile(accel_data, ACC_FILENAME, delimiter='\t')
    List2DelimFile(mag_data, MAG_FILENAME, delimiter='\t')
    ser.Close()
    print('[INFO]: Done!')


if __name__ == '__main__':
    main()