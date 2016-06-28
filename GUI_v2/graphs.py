from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvas
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import numpy as np
from constants import *
pause_graph = False

class PlotColumn(BoxLayout):
    # A column consisting of a label and graphs.
    # label_text: text of label you want on head
    # width, height: how many graphs wide and tall you want the plot to be
    # graphs_wanted: number of graphs you want. default width*height
    # graph_lims: dictionary of x_lim and y_lim. 
    # template is {graph_#: [(xmin, xmax),(ymin, ymax)]}
    def __init__(self, label_text, 
                width, height, 
                graphs_wanted=0,
                graph_lims={}, 
                *args, **kwargs):
        super(PlotColumn, self).__init__(*args, **kwargs)
        self.orientation = 'vertical'
        self.add_widget(Label(text=label_text, size_hint=(1, 0.05)))
        self.add_widget(self.figure(width, height, graphs_wanted, graph_lims))

    def figure(self, width, height, graphs_wanted, graph_lims):
        # Generates a plot with the number of subplots delineated 
        # by graphs_wanted. If graphs_wanted is not defined, then
        # generates maximum number of graphs. 
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
                    for key in graph_lims:
                        if key == counter:
                            # recall: [(xmin, xmax), (ymin, ymax)]
                            subplot.set_xlim(graph_lims[key][0][0], 
                                graph_lims[key][0][0])
                            subplot.set_ylim(graph_lims[key][1][0], 
                                graph_lims[key][1][1])
                    counter += 1
        return FigureCanvas(fig)

    # def figure(self, width, height, graphs_wanted):
    #     fig = plt.figure(figsize=(21,8), facecolor='#E1E6E8')
    #     plt.subplots_adjust(left=.3)

    #     # create NUM_PLOTS subplots in 2 rows
    #     graph_columns = np.ceil(NUM_PLOTS/float(GRAPH_ROWS))
    #     m_axes = [plt.subplot(int(100 * GRAPH_ROWS + 10 * graph_columns + x + 1)) for x in range(NUM_PLOTS)]

    #     xdata = np.zeros(DATA_SIZE)
    #     ydata = [np.full(DATA_SIZE, None) for x in range(NUM_PLOTS)]

    #     # set axis limits
    #     lines = [m_axes[x].plot(ydata[x], '-')[0] for x in range(NUM_PLOTS)]
    #     for x in range(NUM_PLOTS):
    #         m_axes[x].set_ylim(-1.2,1.2)
    #         # m_axes[x].set_title('Plot ' + str(x))
    #         # m_axes[x].get_xaxis().set_visible(False)
    #     m_axes[0].set_title('desired_h_xdot (m/s)')
    #     m_axes[1].set_title('desired_h_ydot (m/s)')
    #     m_axes[2].set_title('desired_h_rdot (deg/s)')
    #     m_axes[2].set_ylim(-360,360) 

    #     m_axes[3].set_title('computed_xdot')
    #     m_axes[4].set_title('computed_ydot')
    #     m_axes[5].set_title('computed_tdot')
    #     m_axes[5].set_ylim(-360,360)
    #     return FigureCanvas(fig)

    # initial state for blitting
    def init_blit(self):
        for j in range(NUM_PLOTS):
            lines[j].set_ydata([])
            lines[j].set_xdata([])
            m_axes[j].set_xlim(-10, 0)

        return tuple([line for line in lines])


    def update_animation(self, data):
        global xdata 
        global ydata

        if pause_graph: return []
        for i in range(NUM_PLOTS):
            lines[i].set_xdata(xdata)
            lines[i].set_ydata(ydata[i])

            xmax = max(xdata)
            m_axes[i].set_xlim(xmax-10, xmax)
        return tuple([line for line in lines])

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