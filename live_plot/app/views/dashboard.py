# Details the bottom half of the screen:
# That is, the buttons and the sliders. 

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.slider import Slider
import sys

sys.path.append('../callbacks')
sys.path.append('../shared')
import dashboard_callbacks
import constants

class ButtonWidget(GridLayout):
    # Left column buttons
    def __init__(self, **kwargs):
        super(ButtonWidget, self).__init__(**kwargs)
        self.cols = 1
        button_list = [Button(text='Connect'), 
                    Button(text='Update Arduino'), 
                    Button(text='Reset Arduino'), 
                    Button(text='Log Data'), 
                    Button(text='Pause Graph')]
        callback_list = [dashboard_callbacks.connect, 
                        dashboard_callbacks.update_arduino, 
                        dashboard_callbacks.reset_arduino, 
                        dashboard_callbacks.log_data, 
                        dashboard_callbacks.pause_graph]

        for button in button_list:
            self.add_widget(button)

        for i in range(len(button_list)):
            button_list[i].bind(on_press=callback_list[i])

    def connect_callback(self, button_instance):
        if button_instance.text == 'Connect':
            button_instance.text = 'Disconnect'
        else:
            button_instance.text = 'Connect'
        print 'this is the connect callback'

    def update_callback(self, button_instance):
        print 'this is the update callback'

    def reset_callback(self, button_instance):
        print 'this is the reset callback'

    def log_callback(self, button_instance):
        print 'this is the log callback'

    def pause_callback(self, button_instance):
        print 'this is the pause callback'

class SliderCombo(BoxLayout):
    # Class that defines:
    # [label] [----slider----] [value] [whitespace]
    # For use with SlierSubWidget for easier abstraction of this unit
    def __init__(self, callback, label, minimum, maximum, **kwargs):
        # Need to pass in a function as a callback
        super(SliderCombo, self).__init__(**kwargs)
        name_label = Label(text=label, size_hint=(0.25, 1.0))
        slider = Slider(min=minimum, max=maximum, value=0)
        label = Label(text='0.00', size_hint=(0.05, 1.0))
        
        def label_change(self, combo_instance):
            # helper function that changes label
            label.text = str(round(slider.value, 3)) # TODO: ROUND VALUE. PLEASE.

        slider.bind(value=label_change) # changes label with change in value
        slider.bind(value=callback) # your callback here
        # Add the widgets in order
        self.add_widget(name_label)
        self.add_widget(slider)
        self.add_widget(label)
        self.add_widget(Label(text='', size_hint=(0.15, 1.0)))


class SliderColumn(BoxLayout):
    # A column of sliders.
    # name_list: list of names for the sliders
    # callback_list: list of callbacks that will be added to each slider
    # val_list: list of tuples for each slider: [(slider1min, slider1max), ...]
    def __init__(self, name_list, callback_list, val_list, **kwargs):
        super(SliderColumn, self).__init__(**kwargs)
        self.orientation = 'vertical'
        for i in range(0,len(callback_list)):
                self.add_widget(SliderCombo(
                    callback=callback_list[i],
                    label=name_list[i],
                    minimum=val_list[0],
                    maximum=val_list[1]))

class SliderWidget(BoxLayout):
    def __init__(self, **kwargs):
        super(SliderWidget, self).__init__(**kwargs)
        # callback lists:
        dot_callback_list = [
        self.x_callback,
        self.y_callback,
        self.theta_callback,
        ]
        # X, Y, Theta dot sliders
        self.add_widget(SliderColumn(
            name_list=['X Dot', 'Y Dot', 'Theta Dot'],
            callback_list=dot_callback_list,
            val_list=[(constants.MIN_R_MOTOR, constants.MAX_R_MOTOR),
            (constants.MIN_L_MOTOR , constants.MAX_L_MOTOR),
            (constants.MAX_T_MOTOR, constants.MAX_T_MOTOR)]))

        self.add_widget(SliderColumn(
            name_list=['P', 'I', 'D'],
            callback_list=dot_callback_list,
            val_list=[(constants.MIN_P, constants.MAX_P),
            (constants.MIN_I , constants.MAX_I),
            (constants.MIN_D, constants.MAX_D)]))

    def x_callback(self, instance, value):
        #dashboard_callbacks.
        print "regular callback"

    def y_callback(self, instance, value):
        print "regular callback"

    def theta_callback(self, instance, value):
        print "regular callback"


# 4 x 3 

class RadioButtonWidget(GridLayout):
    # TODO: Make the toggle thing a lot clearer
    # Maybe get a smaller class to make a switch
    # TODO: Fix the bug that 
    def __init__(self, **kwargs):
        super(RadioButtonWidget, self).__init__(**kwargs)
        self.cols = 1
        self.width = 150
        radio_button_list = [
        ToggleButton(text='Holonomic', group='log_buttons', state='down'),
        ToggleButton(text='Motor', group='log_buttons'),
        ToggleButton(text='Differential', group='log_buttons'),
        ToggleButton(text='Toggle Immediate\nUpdate', group='update')
        ]
        callback_list = [self.holonomic_callback, 
                        self.motor_callback, 
                        self.differential_callback,
                        self.update_callback]

        for button in radio_button_list:
            self.add_widget(button)

        for i in range(len(radio_button_list)):
            radio_button_list[i].bind(on_press=callback_list[i])

    ##### WE LEFT OFF HERE #####
    ## NOTE: SLIDER DOESN'T WORK FOR SOME REASON

    # TODO: abstract this later
    # Remember, separation of concern. we want no callback logic here.
    def holonomic_callback(self, button_instance):
        print 'this is the holonomic callback'

    def motor_callback(self, button_instance):
        print 'this is the motor callback'

    def differential_callback(self, button_instance):
        print 'this is the differential callback'

    def update_callback(self, button_instance):
        print 'this is the update callback'

class Layout(BoxLayout):
    # the integrated layout of all the buttons and sliders. 
    def __init__(self, **kwargs):
        super(Layout, self).__init__(**kwargs)
        self.add_widget(ButtonWidget(size_hint=(0.15, 1.0)))
        self.add_widget(SliderWidget(size_hint=(0.7, 1.0)))
        self.add_widget(RadioButtonWidget(size_hint=(0.15, 1.0)))

