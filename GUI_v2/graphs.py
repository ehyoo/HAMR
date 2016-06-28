from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
import matplotlib.pyplot as plt
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvas

class PlotColumn(BoxLayout):
    def __init__(self, label_text, 
                width, height, 
                graphs_wanted=0, 
                *args, **kwargs):
        super(PlotColumn, self).__init__(*args, **kwargs)
        self.orientation = 'vertical'
        self.add_widget(Label(text=label_text, size_hint=(1, 0.05)))
        self.add_widget(self.figure(width, height, graphs_wanted))

    def figure(self, width, height, graphs_wanted):
        fig = plt.figure()
        counter = 1
        graph_count = 0
        if graphs_wanted == 0:
            graph_count = width * height
        else:
            graph_count = graphs_wanted

        for i in range(1, width + 1):
            for j in range(1, height + 1):
                if counter <= graph_count:
                    subplot = fig.add_subplot(height, width, counter)
                    subplot.axis([-10, 10, -1, 1])
                    fig.add_subplot()
                    counter += 1
        return FigureCanvas(fig)

class Layout(BoxLayout):
    def __init__(self, **kwargs):
        super(Layout, self).__init__(**kwargs)
        self.orientation='horizontal'
        self.add_widget(PlotColumn(
            'Right Motor Velocity (m/s)', 1, 2))
        self.add_widget(PlotColumn(
            'Left Motor Velocity (m/s)', 1, 2))
        self.add_widget(PlotColumn(
            'Setpoint Velocity (m/s)', 1, 2))
        self.add_widget(PlotColumn(
            'Angular Velocity (deg/s)', 2, 2, graphs_wanted=3))