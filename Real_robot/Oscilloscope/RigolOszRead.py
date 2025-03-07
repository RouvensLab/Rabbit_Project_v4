import pyvisa
import time
import numpy as np
import threading

class OscilloscopeReader:
    def __init__(self, address="TCPIP0::192.168.1.30::inst0::INSTR"):
        self.address = address
        self.rm = pyvisa.ResourceManager()
        self.scope = None
        self.channel1_data = None
        self.channel2_data = None
        self._is_running = False
        self._lock = threading.Lock()

    def connect(self):
        try:
            self.scope = self.rm.open_resource(self.address, timeout=12000)
            print(f"Successfully connected to {self.address}")
            return True
        except pyvisa.VisaIOError:
            print("Connection failed!")
            return False

    def disconnect(self):
        if self.scope:
            self.scope.close()
            self.rm.close()
            self.scope = None
            print("Disconnected.")

    def start_reading(self):
        self._is_running = True
        self.thread = threading.Thread(target=self._update_data)
        self.thread.start()

    def stop_reading(self):
        self._is_running = False
        if self.thread.is_alive():
            self.thread.join()

    def _update_data(self):
        while self._is_running:
            self._read_channel_data()
            time.sleep(0.01)

    def _read_channel_data(self):
        channels = ['CHAN1', 'CHAN2']
        for channel in channels:
            try:
                self.scope.write(f':WAV:FORM BYTE')
                self.scope.write(f':WAV:SOUR {channel}')
                self.scope.write(f':WAV:MODE NORM')
                all_data = self.scope.query_binary_values(":WAV:DATA?", datatype='B', is_big_endian=True)
                x_increment = self.scope.query_ascii_values(':WAV:XINC?', converter='f')[0]
                x_origin = self.scope.query_ascii_values(':WAV:XOR?', converter='f')[0]
                y_increment = self.scope.query_ascii_values(':WAV:YINC?', converter='f')[0]
                y_origin = self.scope.query_ascii_values(':WAV:YOR?', converter='f')[0]
                voffset = self.scope.query_ascii_values(f':{channel}:OFFSet?', converter='f')[0]
                vscale = self.scope.query_ascii_values(f':{channel}:SCALe?', converter='f')[0]

                time_array = np.arange(len(all_data)) * x_increment + x_origin
                voltage_array = (((np.array(all_data) - 128) * 10 * vscale / 256) - voffset)

                with self._lock:
                    if channel == 'CHAN1':
                        self.channel1_data = (time_array, voltage_array)
                    elif channel == 'CHAN2':
                        self.channel2_data = (time_array, voltage_array)
            except:
                #print(f"Error while reading the total voltage. {channel}")
                pass
                
    def get_channel1_data(self):
        with self._lock:
            return self.channel1_data

    def get_channel2_data(self):
        with self._lock:
            return self.channel2_data
        
    def get_channel1_value(self):
        data = self.get_channel1_data()
        if data:
            time_array, voltage_array = data
            return self.filter_disturbance(voltage_array)
        else:
            return 0
        
    def get_channel2_value(self):
        data = self.get_channel2_data()
        if data:
            time_array, voltage_array = data
            return self.filter_disturbance(voltage_array)
        else:
            return 0
    
    def filter_disturbance(self, voltage_array):
        """
        Filter out disturbances from the oscilloscope data.
        
        Args:
            time_array (ndarray): Time array.
            voltage_array (ndarray): Voltage array.
        
        Returns:
            averaged voltage
        """
        average_voltage = np.mean(voltage_array)
        return average_voltage
    
    def calc_current(self, voltage, resistance=0.1568):
        return voltage / resistance+0.045
    


if __name__ == "__main__":
    oscilloscope = OscilloscopeReader("TCPIP0::192.168.1.30::inst0::INSTR")

    if oscilloscope.connect():
        oscilloscope.start_reading()
        try:
            while True:
                ch1_data = oscilloscope.get_channel1_value()
                ch2_data = oscilloscope.get_channel2_value()
                print(f"Channel 1: {ch1_data:.5f}V, Channel 2: {ch2_data:.5f}V, Current: {oscilloscope.calc_current(ch1_data):.5f}")
                time.sleep(0.01)
        except KeyboardInterrupt:
            oscilloscope.stop_reading()
            oscilloscope.disconnect()
