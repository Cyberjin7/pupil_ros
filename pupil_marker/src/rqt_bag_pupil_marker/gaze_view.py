from rqt_bag_plugins.image_view import ImageView
from rqt_bag import TopicMessageView

from PIL import Image
from PIL.ImageQt import ImageQt

from python_qt_binding.QtGui import QPen, QBrush, QPixmap
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox, QVBoxLayout, QHBoxLayout, QApplication
from python_qt_binding.QtCore import QRectF, Qt

# for code completion. Comment out when running code
# from PyQt5.QtGui import QPixmap
# from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QPen, QBrush, QPixmap
from PyQt5.QtWidgets import QApplication, QGraphicsPixmapItem



class GazeView(ImageView):
    name = 'Gaze'

    def __init__(self, timeline, parent, topic):
        # super(GazeView, self).__init__(timeline, parent, topic)
        TopicMessageView.__init__(self, timeline, parent, topic)

        self.pen = QPen()
        self.brush = QBrush(Qt.green, Qt.SolidPattern)

        self.popups = self.timeline.popups
        self.popup_keys = list(self.popups.keys())
        # self._image_view = self.popups[self.popup_keys[0]].findChild(QGraphicsView)
        self._image_view = self.popups['pupil_world__Marker'].findChild(QGraphicsView)
        self._scene = self._image_view.scene()

        self.gaze_size = 30

        # print(self.popup_keys)
        # print(self._image_view)
        # print(self._scene)

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]

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

