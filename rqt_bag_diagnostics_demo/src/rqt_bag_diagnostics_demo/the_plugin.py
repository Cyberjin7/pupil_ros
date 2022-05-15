from rqt_bag import TopicMessageView
from rqt_bag.plugins.plugin import Plugin
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QBrush
from diagnostic_msgs.msg import DiagnosticStatus


def get_color(diagnostic):
    if diagnostic.level == DiagnosticStatus.OK:
        return Qt.green
    elif diagnostic.level == DiagnosticStatus.WARN:
        return Qt.yellow
    else:
        return Qt.red


class DiagnosticPanel(TopicMessageView):
    name = 'Awesome Diagnostic'

    def __init__(self, timeline, parent, topic):
        super(DiagnosticPanel, self).__init__(timeline, parent, topic)
        self.widget = QWidget()
        parent.layout().addWidget(self.widget)
        self.msg = None
        self.widget.paintEvent = self.paintEvent

    def message_viewed(self, bag, msg_details):
        super(DiagnosticPanel, self).message_viewed(bag, msg_details)
        _, self.msg, _ = msg_details  # t, msg, topic = msg_details
        self.widget.update()

    def paintEvent(self, event):
        self.qp = QPainter()
        self.qp.begin(self.widget)

        rect = event.rect()

        if self.msg is None:
            self.qp.fillRect(0, 0, rect.width(), rect.height(), Qt.white)
        else:
            color = get_color(self.msg)
            self.qp.setBrush(QBrush(color))
            self.qp.drawEllipse(0, 0, rect.width(), rect.height())


class DiagnosticBagPlugin(Plugin):
    def __int__(self):
        pass

    def get_view_class(self):
        return DiagnosticPanel

    def get_renderer_class(self):
        return None

    def get_message_types(self):
        return ['diagnostic_msgs/DiagnosticStatus']
