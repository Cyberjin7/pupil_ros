from rqt_bag_plugins.image_view import ImageView
from rqt_bag.plugins.raw_view import MessageTree
from rqt_bag import TopicMessageView

# from PIL import Image
# from PIL.ImageQt import ImageQt

from python_qt_binding.QtGui import QPen, QBrush, QPixmap
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox, QVBoxLayout, QHBoxLayout, QApplication, QGraphicsPixmapItem, QComboBox, QLabel
from python_qt_binding.QtCore import QRectF, Qt

# for code completion. Comment out when running code
# from PyQt5.QtGui import QPixmap
# from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QPen, QBrush, QPixmap

from PyQt5.QtWidgets import QApplication, QGraphicsPixmapItem, QComboBox, QLabel

import numpy as np

class GazeView(ImageView):
    name = 'Gaze'

    def __init__(self, timeline, parent, topic):
        # super(GazeView, self).__init__(timeline, parent, topic)
        TopicMessageView.__init__(self, timeline, parent, topic)

        self.message_tree = MessageTree(parent)
        parent.layout().addWidget(self.message_tree)

        self.label_Layout = QHBoxLayout()
        self.label_Layout.addWidget(QLabel("Marker:"))
        self.labels_box = QComboBox()
        self.labels_box.addItems(['Target Start',
                                  'Target End',
                                  'Goal Start',
                                  'Goal End',
                                  'Experiment Start',
                                  'Experiment End'])
        #self.labels_box.setPlaceholderText('None')
        # self.labels_box.currentTextChanged.connect(self.test2)
        # self.labels_box.activated.connect(self.test1)

        self.label_Layout.addWidget(self.labels_box)
        parent.layout().addLayout(self.label_Layout)

        self.remove_button = QPushButton('Remove')
        self.remove_button.setCheckable(True)
        self.remove_button.setChecked(True)
        self.remove_button.setEnabled(False)
        self.add_button = QPushButton('Add')
        self.add_button.clicked.connect(self.add_marker)

        self.button_Layout = QHBoxLayout()
        self.button_Layout.addWidget(self.remove_button)
        self.button_Layout.addWidget(self.add_button)
        parent.layout().addLayout(self.button_Layout)

        self.labels_box.setCurrentText('Experiment Start')
        self.labels_box.setCurrentIndex(-1)

        self.pen = QPen()
        self.brush = QBrush(Qt.green, Qt.SolidPattern)

        self.popups = self.timeline.popups
        self.popup_keys = list(self.popups.keys())
        # self._image_view = self.popups[self.popup_keys[0]].findChild(QGraphicsView)
        self._image_view = self.popups['pupil_world__World_View'].findChild(QGraphicsView)
        self._scene = self._image_view.scene()

        self.gaze_size = 30

        self.markers = []  # TODO: Use numpy array instead. Or maybe stick to list for markers since string?
        # self.timestamps = []  # TODO: Use numpy array instead
        self.timestamps = np.array([], dtype=np.float64)
        self.msg_timestamp = 0.0

        # print(self.popup_keys)
        # print(self._image_view)
        # print(self._scene)

    def add_marker(self):
        marker = self.labels_box.currentText()
        # print(marker)

        if marker != "":
            index_array = np.nonzero(self.timestamps >= self.msg_timestamp)
            if not index_array[0].any():
                self.timestamps = np.append(self.timestamps, self.msg_timestamp)
                self.markers.append(marker)
            else:
                pass  # TODO: 

        if self.msg_timestamp not in self.timestamps and marker != "":
            index = np.nonzero(self.timestamps > self.msg_timestamp)
            print(index)
            # self.timestamps = np.insert(self.timestamps, index[0][0])
            # self.markers.insert(index[0][0], marker)
            print(self.timestamps)
            print(self.markers)
            # self.timestamps = np.append(self.timestamps, self.msg_timestamp)  # placeholder code
        elif self.msg_timestamp in self.timestamps:
            pass

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        self.msg_timestamp = msg.timestamp  # alternatively: self.msg_timestamp = t.to_sec()

        # Currently assumes only one pixmap for image drawn
        scene_items = self._scene.items(Qt.DescendingOrder)
        for item in scene_items:
            if isinstance(item, QGraphicsPixmapItem):
                width = item.pixmap().width()
                height = item.pixmap().height()
                image = item.pixmap()
                self._scene.clear()
                self._scene.addPixmap(image)
                self._scene.addEllipse(QRectF(width*msg.norm_pos.x - self.gaze_size/2,
                                              height*(1-msg.norm_pos.y) - self.gaze_size/2,
                                              self.gaze_size,
                                              self.gaze_size),
                                       self.pen,
                                       self.brush)

        if t is None:
            TopicMessageView.message_cleared(self)
            self.message_tree.set_message(None)
        else:
            self.message_tree.set_message(msg)
            if self.msg_timestamp not in self.timestamps:
                self.labels_box.setCurrentIndex(-1)
            else:
                index = np.nonzero(self.timestamps == self.msg_timestamp)[0][0]
                self.labels_box.setCurrentText(self.markers[index])

