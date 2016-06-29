# Main logic for building the graphs
# Make custom graphs using the PlotColumn class
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvas
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import numpy as np
from constants import *

class PlotColumn(BoxLayout):
    # A column consisting of a label and graphs.
    # label_text: text of label you want on head
    # width, height: how many graphs wide and tall you want the plot to be
    # graphs_wanted: number of graphs you want. default width*height
    # graph_lims: dictionary of x_lim and y_lim. 
    # template is {graph_#: [xmin, xmax, ymin, ymax]}
    # subgraph_names: labels for the subgraphs, {graph#: name1}
    def __init__(self, label_text, 
                width, height, 
                graphs_wanted=0,
                graph_lims={},
                subgraph_names={}, 
                *args, **kwargs):
        super(PlotColumn, self).__init__(*args, **kwargs)
        self.orientation = 'vertical'
        self.add_widget(Label(text=label_text, size_hint=(1, 0.05)))
        self.add_widget(self.figure(
            width, height, graphs_wanted, graph_lims, subgraph_names))

# # TODO: PUT THIS SOMEWHERE
# xdata = np.zeros(DATA_SIZE) # numpy array of DATA_SIZE length
# ydata = [np.full(DATA_SIZE, None) for x in range(NUM_PLOTS)]
# # numpy matrix of DATA_SIZE length and NUM_PLOTS height

# # set axis limits
# lines = [m_axes[x].plot(ydata[x], '-')[0] for x in range(NUM_PLOTS)]

    def figure(self, width, height, graphs_wanted, graph_lims, subgraph_names):
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
                    subplot.axis([-10, 0, -1.2, 1.2]) #defaults
                    fig.add_subplot()
                    self.graph_setter(subplot, counter, graph_lims)
                    self.graph_setter(subplot, counter, subgraph_names)
                    counter += 1
        return FigureCanvas(fig)

    def graph_setter(self, subplot, counter, plot_dict):
        # helper function that implements certain attributes to a subplot 
        # that's passed in. currently only supporting x and y max/min 
        # and title setting
        for key in plot_dict:
            if key == counter:
                if type(plot_dict[key]) == list:
                    print 'hits' 
                    subplot.axis(plot_dict[key])
                elif type(plot_dict[key]) == str:
                    subplot.set_title(plot_dict[key])

    #initial state for blitting
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
            'xdot', 
            width=1, 
            height=2, 
            subgraph_names={1: 'Desired (m/s)', 2: 'Computed'}))

        self.add_widget(PlotColumn(
            'ydot', 
            width=1, 
            height=2, 
            subgraph_names={1: 'Desired (m/s)', 2: 'Computed'}))

        self.add_widget(PlotColumn(
            'hdot/rdot', 
            width=1, 
            height=2, 
            graph_lims={1: [-10, 0, -360, 360], 2: [-10, 0, -360, 360]}, 
            subgraph_names={1: 'Desired (m/s)', 2: 'Computed'}))
        