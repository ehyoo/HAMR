from matplotlib.widgets import Button
from matplotlib.widgets import AxesWidget

class MyButton(Button):
    def __init__(self, ax, label, image=None,
             color='0.85', hovercolor='0.95'):
        AxesWidget.__init__(self, ax)

        if image is not None:
            ax.imshow(image)
        self.label = ax.text(0.5, 0.5, label,
                             verticalalignment='center',
                             horizontalalignment='center',
                             transform=ax.transAxes)

        self.cnt = 0
        self.observers = {}

        self.connect_event('button_press_event', self._click)
        self.connect_event('button_release_event', self._release)
        self.connect_event('motion_notify_event', self._motion)
        ax.set_navigate(False)
        ax.set_axis_bgcolor(color)
        ax.set_xticks([])
        ax.set_yticks([])
        # self.ax.spines['top'].set_visible(False)
        # self.ax.spines['right'].set_visible(False)
        # self.ax.spines['bottom'].set_visible(False)
        # self.ax.spines['left'].set_visible(False)
        self.color = color
        self.hovercolor = hovercolor

        self._lastcolor = color



    def _release(self, event):
        super(MyButton, self)._release(event)
        self.redraw_button()

    def _motion(self, event):
        if self.ignore(event):
            return
        if event.inaxes == self.ax:
            c = self.hovercolor
        else:
            c = self.color
        if c != self._lastcolor:
            self.ax.set_axis_bgcolor(c)
            self._lastcolor = c
            if self.drawon:
                self.redraw_button()

    def redraw_button(self):
        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.ax.spines['top'])
        self.ax.draw_artist(self.ax.spines['left'])
        self.ax.draw_artist(self.label)
        self.ax.figure.canvas.update()
        self.ax.figure.canvas.flush_events()