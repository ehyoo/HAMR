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
pause_graph = False

class PlotColumn(BoxLayout):
    # A column consisting of a label and graphs.
    # label_text: text of label you want on head
    # width, height: how many graphs wide and tall you want the plot to be
    # graphs_wanted: number of graphs you want. default width*height
    # graph_lims: dictionary of x_lim and y_lim. 
    # template is {graph_#: [(xmin, xmax),(ymin, ymax)]}
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
                    subplot.axis([-1.2, 1.2, -10, 0]) #defaults
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
                if type(plot_dict[key][0]) == tuple:
                    print 'hits'                
                    subplot.set_xlim(plot_dict[key][0][0], 
                                    plot_dict[key][0][0])
                    subplot.set_ylim(plot_dict[key][1][0], 
                                    plot_dict[key][1][1])
                elif type(plot_dict[key]) == str:
                    subplot.set_title(plot_dict[key])

    # def figure(self, width, height, graphs_wanted, graph_lims, blah):
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
            graph_lims={1: [(-10, 0),(-360, 360)], 2: [(-10, 0),(-360, 360)]}, 
            subgraph_names={1: 'Desired (m/s)', 2: 'Computed'}))
        