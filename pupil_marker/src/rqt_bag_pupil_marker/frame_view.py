from rqt_bag_plugins.image_view import ImageView
from rqt_bag import TopicMessageView
from rqt_bag.plugins.raw_view import MessageTree

from PIL import Image
from PIL.ImageQt import ImageQt

import sys

from python_qt_binding import QT_BINDING_MODULES
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox, QVBoxLayout, QHBoxLayout

# for code completion. Comment out when running code
from PyQt5.QtGui import QPixmap
# from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox


class FrameView(ImageView):
    name = 'Marker'

    def __init__(self, timeline, parent, topic):
        # super(FrameView, self).__init__(timeline, parent, topic)
        TopicMessageView.__init__(self, timeline, parent, topic)
        self._image = None
        self._image_topic = None
        self._image_stamp = None
        self.quality = Image.NEAREST

        self.world_frame = None

        self.message_tree = MessageTree(parent)

        self._overlay_font_size = 14.0
        self._overlay_indent = (4, 4)
        self._overlay_color = (0.2, 0.2, 1.0)

        self.make_box('1')

        self._image_view = QGraphicsView(parent)
        self._image_view.resizeEvent = self._resizeEvent
        self._scene = QGraphicsScene()
        self._image_view.setScene(self._scene)
        parent.layout().addWidget(self._image_view)
        parent.layout().addWidget(self.message_tree)
        parent.layout().addWidget(QPushButton('Export Markers'))
        parent.layout().addWidget(self.box1)
        print(type(parent))

    def make_box(self, name):
        self.box1 = QGroupBox(name)

        layout = QVBoxLayout()
        layout.addWidget(QPushButton("1"))
        layout.addWidget(QPushButton("2"))
        self.box1.setLayout(layout)

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        for frame in msg.frames:
            if frame.topic == 'frame.world':
                self.world_frame = frame.image

        if not msg:
            self.set_image(None, topic, 'no message')
        else:
            self.set_image(self.world_frame, topic, t)  # temporary time until I implement synced time
        if t is None:
            self.message_cleared()
        else:
            self.message_tree.set_message(msg)

    def message_cleared(self):
        super(ImageView, self).message_cleared()




