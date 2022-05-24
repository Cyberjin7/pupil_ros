from rqt_bag_plugins.image_view import ImageView
from rqt_bag import TopicMessageView
from rqt_bag.plugins.raw_view import MessageTree

from PIL import Image
from PIL.ImageQt import ImageQt

import sys

from python_qt_binding import QT_BINDING_MODULES
from python_qt_binding.QtGui import QPen, QBrush
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox, QVBoxLayout, QHBoxLayout
from python_qt_binding.QtCore import QRectF

# for code completion. Comment out when running code
# from PyQt5.QtGui import QPixmap
# from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QPushButton, QGroupBox
from PyQt5.QtCore import QRectF
from PyQt5.QtGui import QPen, QBrush


class FrameView(ImageView):
    name = 'Marker'

    def __init__(self, timeline, parent, topic):
        # super(FrameView, self).__init__(timeline, parent, topic)
        # TopicMessageView.__init__(self, timeline, parent, topic)
        self._image = None
        self._image_topic = None
        self._image_stamp = None
        self.quality = Image.NEAREST

        self.world_frame = None

        self.message_tree = MessageTree(parent)

        self._overlay_font_size = 14.0
        self._overlay_indent = (4, 4)
        self._overlay_color = (0.2, 0.2, 1.0)

        # self.make_box('1')

        self._image_view = QGraphicsView(parent)
        self._image_view.resizeEvent = self._resizeEvent
        self._scene = QGraphicsScene()
        self._image_view.setScene(self._scene)
        parent.layout().addWidget(self._image_view)
        parent.layout().addWidget(self.message_tree)
        TopicMessageView.__init__(self, timeline, parent, topic)
        parent.layout().addWidget(QPushButton('Export Markers'))
        # parent.layout().addWidget(self.box1)
        # print(type(parent))
        self.topics = timeline._get_topics()  # once I know which topics are needed, will filter list accordingly

    def make_box(self, name):
        self.box1 = QGroupBox(name)

        layout = QHBoxLayout()
        layout.addWidget(QPushButton("1"))
        layout.addWidget(QPushButton("2"))
        self.box1.setLayout(layout)

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        topic, msg, t = msg_details[:3]
        # for frame in msg.frames:
        #     if frame.topic == 'frame.world':
        #         self.world_frame = frame.image

        if not msg:
            self.set_image(None, topic, 'no message')
        else:
            if topic == 'pupil_world':
                self.set_image(msg.image, topic, t)  # temporary time until I implement synced time
            elif topic == 'pupil_gaze':
                self._scene.addEllipse(QRectF(50, 50, 10, 10), QPen(), QBrush())

        if t is None:
            self.message_cleared()
        else:
            self.message_tree.set_message(msg)

    def message_cleared(self):
        super(ImageView, self).message_cleared()

    def navigate_next(self):
        for bag, entry in self.timeline.get_entries_with_bags(self.topics, self.timeline._timeline_frame.playhead, self.timeline._timeline_frame.end_stamp):
            if entry.time > self.timeline._timeline_frame.playhead:
                self.message_viewed(bag, self.timeline.read_message(bag, entry.position))
                self.timeline._timeline_frame.playhead = entry.time
                break

    # TODO: skips entries. fix
    def navigate_previous(self):
        last_entry = None
        last_bag = None
        for bag, entry in self.timeline.get_entries_with_bags(self.topics, self.timeline._timeline_frame.start_stamp,
                                               self.timeline._timeline_frame.playhead):
            if entry.time < self.timeline._timeline_frame.playhead:
                last_entry = entry
                last_bag = bag

        if last_entry:
            self.message_viewed(last_bag, self.timeline.read_message(last_bag, last_entry.position))
            self.timeline._timeline_frame.playhead = last_entry.time

    def navigate_first(self):
        for bag, entry in self.timeline.get_entries_with_bags(self.topics, *self.timeline._timeline_frame.play_region):
            self.message_viewed(bag, self.timeline.read_message(bag, entry.position))
            self.timeline._timeline_frame.playhead = entry.time
            break

    def navigate_last(self):
        last_entry = None
        last_bag = None
        for bag, entry in self.timeline.get_entries_with_bags(self.topics, *self.timeline._timeline_frame.play_region):
            last_entry = entry
            last_bag = bag

        if last_entry:
            self.message_viewed(last_bag, self.timeline.read_message(last_bag, last_entry.position))
            self.timeline._timeline_frame.playhead = last_entry.time