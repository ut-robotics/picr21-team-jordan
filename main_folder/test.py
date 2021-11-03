class SerialPortNotFound(Exception):
    def __init__(self):
        super().__init__("Serial port not found")

raise SerialPortNotFound