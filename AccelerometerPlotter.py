import serial
import sys
import random
from time import time_ns
from pyqtgraph import Qt, QtWidgets, QtCore, GraphicsLayoutWidget, setConfigOptions, mkPen
from collections import deque
from numpy import fromstring, int16

FS = 26666.7
DELTA_T = 1./FS
NUM_OF_LAST_DATAPOINTS = 10000

x_data = deque([random.randrange(-100, 100) for i in range(NUM_OF_LAST_DATAPOINTS)], maxlen=NUM_OF_LAST_DATAPOINTS)
y_data = deque([random.randrange(-100, 100) for i in range(NUM_OF_LAST_DATAPOINTS)], maxlen=NUM_OF_LAST_DATAPOINTS)
z_data = deque([random.randrange(-100, 100) for i in range(NUM_OF_LAST_DATAPOINTS)], maxlen=NUM_OF_LAST_DATAPOINTS)
t_data = deque(reversed([-i*DELTA_T for i in range(NUM_OF_LAST_DATAPOINTS)])) # shifted timeline
mps = 0
bpr = 0

class Chart(Qt.QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(Chart, self).__init__(parent)
        # Probably speeds up graphing

        self.FPS_TARGET = 100

        self.dataReciveThread = Worker()

        # Create GUI elements
        self.mainbox = Qt.QtWidgets.QWidget()
        self.setCentralWidget(self.mainbox)

        self.mainbox.setLayout(Qt.QtWidgets.QVBoxLayout())

        self.canvas = GraphicsLayoutWidget()
        self.canvas.setBackground('#19232D')

        # Speed up from stackoverflow https://stackoverflow.com/a/66704685/14973436
        self.canvas.setViewportUpdateMode(Qt.QtWidgets.QGraphicsView.BoundingRectViewportUpdate)
        self.canvas.setAntialiasing(False)
        self.canvas.setOptimizationFlag(Qt.QtWidgets.QGraphicsView.DontAdjustForAntialiasing)
        self.canvas.scene().setItemIndexMethod(Qt.QtWidgets.QGraphicsScene.NoIndex)

        #  X line
        self.plot1 = self.canvas.addPlot(row=0, col=0)
        self.plot1.setXRange(t_data[0], 0) # create custom timeline
        self.plot1.setYRange(-100, 100)
        self.plot1.showGrid(x=True, y=True)
        self.plot1.setLabel('bottom', 'Time', units='s')
        self.plot1.setLabel('left', 'X acceleration', units='mg')
        self.h1 = self.plot1.plot(pen=mkPen('r', width=1))

        #  Y line
        self.plot2 = self.canvas.addPlot(row=1, col=0)
        self.plot2.setXRange(t_data[0], 0) # create custom timeline
        self.plot2.setYRange(-100, 100)
        self.plot2.showGrid(x=True, y=True)
        self.plot2.setLabel('bottom', 'Time', units='s')
        self.plot2.setLabel('left', 'Y acceleration', units='mg')
        self.h2 = self.plot2.plot(pen=mkPen('g', width=1))

        #  Z line
        self.plot3 = self.canvas.addPlot(row=2, col=0)
        self.plot3.setXRange(t_data[0], 0) # create custom timeline
        self.plot3.setYRange(-100, 100)
        self.plot3.showGrid(x=True, y=True)
        self.plot3.setLabel('bottom', 'Time', units='s')
        self.plot3.setLabel('left', 'Z acceleration', units='mg')
        self.h3 = self.plot3.plot(pen=mkPen('y', width=1))

        self.plot1.setDownsampling(ds=5, auto=False, mode='peak') #If set to `True`, and NaN values exist, unpredictable behavior will occur.
        self.plot2.setDownsampling(ds=5, auto=False, mode='peak')
        self.plot3.setDownsampling(ds=5, auto=False, mode='peak')

        self.h1.setSkipFiniteCheck(True)
        self.h2.setSkipFiniteCheck(True)
        self.h3.setSkipFiniteCheck(True)

        self.h1.setAlpha(alpha=1, auto=False)
        self.h2.setAlpha(alpha=1, auto=False)
        self.h3.setAlpha(alpha=1, auto=False)

        self.mainbox.layout().addWidget(self.canvas)

        self.label1 = Qt.QtWidgets.QLabel()
        self.label2 = Qt.QtWidgets.QLabel()
        self.label3 = Qt.QtWidgets.QLabel()

        self.mainbox.layout().addWidget(self.label1)
        self.mainbox.layout().addWidget(self.label2)
        self.mainbox.layout().addWidget(self.label3)

        # FPS counter
        self.fps = 0.
        self.frame_lastupdate = time_ns()
        self.lastNonZero_dt = 1

        self.sps = 0.

        # self.ser = serial.Serial('COM8', 7000000, timeout=0.0001)
        # self.buffer = b''
        self.sample_lastupdate = time_ns()
        self.CHUNK_SIZE = NUM_OF_LAST_DATAPOINTS//5
        self.t = Qt.QtCore.QTime()
        self.t.start()

        # Plot refresher timer
        self.tmr = Qt.QtCore.QTimer()
        self.tmr.timeout.connect(self.refreshPlot)
        self.tmr.setInterval(1000//self.FPS_TARGET)
        self.tmr.start()

        self.startDataFeed()

    def startDataFeed(self):
        self.dataReciveThread.start()

    def refreshPlot(self):
        global x_data, y_data, z_data, t_data, mps
        # if bufferLock.tryLock(timeout=5):
        if  len(x_data) == NUM_OF_LAST_DATAPOINTS:
        # # _callSync='off' because we do not want to wait for a return value
            self.h1.setData(x=t_data, y=x_data, _callSync='off')
            self.h2.setData(x=t_data, y=y_data, _callSync='off')
            self.h3.setData(x=t_data, y=z_data, _callSync='off')
        # bufferLock.unlock()
        # app.processEvents()  ## brute force complete redraw for every plot <- could mess with event loop

        now = time_ns()
        if now <= self.frame_lastupdate:
            dt = self.lastNonZero_dt
        else:
            dt = (now-self.frame_lastupdate)
            self.lastNonZero_dt = dt
        self.frame_lastupdate = now
        self.fps = 1.0 / (dt / (10 ** 9))
        self.label1.setText(f'Frame Rate:  {self.fps:0.3f} FPS')
        self.label2.setText(f'Redaout per sec:  {mps:0.0f} mps')
        self.label3.setText(f'Bytes read:  {bpr:0.0f} bytes')
        # else:
        #     return

class Worker(QtCore.QThread):
    def __init__(self, parent=None):
        super(Worker, self).__init__(parent)
        self.exiting = False
        self.RAW_VAL_TO_MG = 0.061

    def __del__(self):
        self.exiting = True

    def run(self):
        # pass
        # import time
        # while(1):
            # lol=[random.randrange(-100, 100) for i in range(NUM_OF_LAST_DATAPOINTS)]
            # x_data.extend(lol)
            # y_data.extend(lol)
            # z_data.extend(lol)
            # time.sleep(0.01)
        sample_lastupdate = time_ns()
        lastNonZero_dt = 1
        buffer = b''
        RX_BUFF_SIZE = 128000000
        RX_CHUNK_SIZE_LIMIT = 1000
        TRIM_BUFFER_TSH = 5000
        EXPECTED_VALUES = 4
        EXPECTED_SEPARATORS = EXPECTED_VALUES - 1
        with serial.Serial('COM8', 7000000, timeout=0.005, parity=serial.PARITY_EVEN) as ser:
            ser.set_buffer_size(rx_size = RX_BUFF_SIZE)
            ser.flushInput()
            ser.flushOutput()
            global mps, bpr
            while True:
                readBytes = ser.inWaiting()
                if readBytes > RX_CHUNK_SIZE_LIMIT: # Handle big buffer state
                    if readBytes > TRIM_BUFFER_TSH: # Handle massive buffer state
                        ser.flushInput()
                    bpr = readBytes = RX_CHUNK_SIZE_LIMIT
                buffer += ser.read(readBytes)
                bpr = len(buffer)
                if b'\n' in buffer:
                    lines = buffer.splitlines(True)
                    line_cnt = 0
                    for line in lines[:-1]:
                        line_cnt+=1                          
                        if line != b'':
                            try:
                                vals_u = [(int(str_val, base=16)) for str_val in line[:-1].split(b';')]
                                vals_s = [-( (val ^ 0xffff) + 1) if (val & 0x8000) == 0x8000 else val for val in vals_u]
                            except:
                                pass
                                print(f'BAD {line} {len(lines)} {line_cnt}')
                            else:
                                if len(vals_s)>0:
                                    if len(vals_s) == 4:
                                        # bufferLock.lock()
                                        x_data.append(vals_s[1]*self.RAW_VAL_TO_MG)
                                        y_data.append(vals_s[2]*self.RAW_VAL_TO_MG)
                                        z_data.append(vals_s[3]*self.RAW_VAL_TO_MG)
                                        # bufferLock.unlock()
                                    else:
                                        print(f'BAD LEN {vals_s} {line} {len(lines)} {line_cnt}')
                                        # print(f'{lines}')
                                        pass
                                else:
                                    print(f'BAD VALS NO {vals_s} {line} {len(lines)} {line_cnt}')
                                    pass
                            now = time_ns()
                            if now <= sample_lastupdate:
                                dt = lastNonZero_dt
                            else:
                                dt = (now-sample_lastupdate)
                                lastNonZero_dt = dt
                            sample_lastupdate = now
                            mps = 1.0 / (dt / (10 ** 9))
                    line_cnt = 0
                    if lines[-1][0] == 48:  # indexing ascii '0'
                        buffer = lines[-1]
                    else:
                        buffer = b''

if __name__ == "__main__":
    bufferLock = QtCore.QMutex()

    setConfigOptions(antialias=False, useOpenGL=True)

    QtWidgets.QApplication.setHighDpiScaleFactorRoundingPolicy(QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)

    app = QtWidgets.QApplication([])
    app.setStyleSheet("""
    QWidget {
        background-color: #19232D;
        border: 0px solid #32414B;
        padding: 0px;
        color: #F0F0F0;
        selection-background-color: #1464A0;
        selection-color: #F0F0F0;
    }""")
    app.setStyle(QtWidgets.QStyleFactory.create('Cleanlooks'))

    widget = Chart()
    widget.setWindowTitle('Accelerometer plot')
    widget.show()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        sys.exit(app.instance().exec_())

