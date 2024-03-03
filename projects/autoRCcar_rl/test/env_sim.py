import sys
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout
import numpy as np

app = QApplication([])

# 데이터 준비
x = np.linspace(0, 10, 100)
y1 = np.sin(x)
y2 = np.cos(x)
y3 = np.tan(x)

# 그래프 위젯 생성
widget = QWidget()
layout = QGridLayout()  # 그리드 레이아웃으로 변경
widget.setLayout(layout)

# 그래프 생성 및 설정
graphWidget1 = pg.PlotWidget(title="Trajectory")
graphWidget2 = pg.PlotWidget(title="Heading")
graphWidget3 = pg.PlotWidget(title="Speed")

# X, Y 축 라벨 설정
graphWidget1.setLabel('left', 'Y')
graphWidget1.setLabel('bottom', 'X')
graphWidget2.setLabel('left', 'Y')
graphWidget2.setLabel('bottom', 'X')
graphWidget3.setLabel('left', 'Y')
graphWidget3.setLabel('bottom', 'X')

# 데이터 플롯
graphWidget1.plot(x, y1, pen=pg.mkPen('r'))  # Red color for sin
graphWidget2.plot(x, y2, pen=pg.mkPen('g'))  # Green color for cos
graphWidget3.plot(x, y3, pen=pg.mkPen('b'))  # Blue color for tan

# 위젯 추가
layout.addWidget(graphWidget1, 0, 0)  # 첫 번째 행, 첫 번째 열
layout.addWidget(graphWidget2, 0, 1)  # 첫 번째 행, 두 번째 열
layout.addWidget(graphWidget3, 0, 2)  # 첫 번째 행, 세 번째 열

widget.show()
sys.exit(app.exec_())