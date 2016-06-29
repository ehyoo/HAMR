from matplotlib.widgets import AxesWidget
from matplotlib.widgets import Button
from matplotlib.widgets import CheckButtons
from matplotlib.widgets import Slider
import six


#####################
# MODIFIED BUTTON
#####################
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
        self.color = color
        self.hovercolor = hovercolor

        self._lastcolor = color


    def _release(self, event):
        super(MyButton, self)._release(event)
        self.redraw()

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
                self.redraw()

    def redraw(self):
        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.ax.spines['top'])
        self.ax.draw_artist(self.ax.spines['left'])
        self.ax.draw_artist(self.label)
        self.ax.figure.canvas.update()
        self.ax.figure.canvas.flush_events()


#####################
# MODIFIED SLIDER
#####################
class MySlider(Slider):
    def set_val(self, val, round = None):

        if round is not None: val = int(val / round * round)
        self.valbbox = self.valtext.get_window_extent().get_points()
        self.labelbbox = self.label.get_window_extent().get_points()

        xy = self.poly.xy
        xy[2] = val, 1
        xy[3] = val, 0
        self.poly.xy = xy
        self.valtext.set_text(self.valfmt % val)
        if self.drawon:
            self.redraw()
        self.val = val
        if not self.eventson:
            return
        for cid, func in six.iteritems(self.observers):
            func(val)

    def redraw(self):
        patch = self.ax.figure.patch
        wh = self.ax.figure.get_size_inches()*self.ax.figure.dpi

        x = 1.0 * self.valbbox[0][0] / wh[0]
        y = 1.0 * self.valbbox[0][1] / wh[1]
        width = 1.0 * self.valbbox[1][0] / wh[0]- x
        height = 1.0 * self.valbbox[1][1] / wh[1]- y

        patch.set_bounds(x, y, width, height)
        self.ax.draw_artist(patch)

        x = 1.0 * self.labelbbox[0][0] / wh[0]
        y = 1.0 * self.labelbbox[0][1] / wh[1]
        width = 1.0 * self.labelbbox[1][0] / wh[0]- x
        height = 1.0 * self.labelbbox[1][1] / wh[1]- y

        patch.set_bounds(x, y, width, height)
        self.ax.draw_artist(patch)


        patch.set_bounds(0, 0, 1, 1)

        # print slider_r_motor.valtext.get_window_extent()
        # print  fig.get_size_inches()*fig.dpi

        self.ax.draw_artist(self.ax.patch)
        self.ax.draw_artist(self.poly)
        self.ax.draw_artist(self.vline)
        self.ax.draw_artist(self.valtext)
        self.ax.draw_artist(self.label)
        self.ax.draw_artist(self.ax.spines['top'])
        self.ax.draw_artist(self.ax.spines['left'])


        self.ax.figure.canvas.update()
        self.ax.figure.canvas.flush_events()


#####################
# MODIFIED CHECKBOX
#####################
class MyCheckBox(CheckButtons):
    def _clicked(self, event):
        if self.ignore(event):
            return
        if event.button != 1:
            return
        if event.inaxes != self.ax:
            return

        for p, t, lines in zip(self.rectangles, self.labels, self.lines):
            if (t.get_window_extent().contains(event.x, event.y) or
                    p.get_window_extent().contains(event.x, event.y)):
                l1, l2 = lines
                l1.set_visible(not l1.get_visible())
                l2.set_visible(not l2.get_visible())
                thist = t
                break
        else:
            return

        if self.drawon:
            self.redraw()

        if not self.eventson:
            return
        for cid, func in six.iteritems(self.observers):
            func(thist.get_text())

    def redraw(self):
        self.ax.draw_artist(self.ax.patch)
        for x in self.rectangles: self.ax.draw_artist(x)
        for x in self.labels: self.ax.draw_artist(x)
        self.ax.draw_artist(self.ax.spines['top'])
        self.ax.draw_artist(self.ax.spines['left'])
        self.ax.draw_artist(self.ax.spines['bottom'])
        self.ax.draw_artist(self.ax.spines['right'])
        for x in self.lines:
            for y in x: self.ax.draw_artist(y)


        self.ax.figure.canvas.update()
        self.ax.figure.canvas.flush_events()