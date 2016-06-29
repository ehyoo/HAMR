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
        callback_list = [self.connect_callback, 
                        self.update_callback, 
                        self.reset_callback, 
                        self.log_callback, 
                        self.pause_callback]

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
    # [----slider----] [value] [whitespace]
    # For use with SlierSubWidget for easier abstraction of this unit
    def __init__(self, callback, **kwargs):
        # Need to pass in a function as a callback
        super(SliderCombo, self).__init__(**kwargs)
        slider = Slider(min=-1, max=1, value=0)
        label = Label(text='0.00', size_hint=(0.05, 1.0))
        
        def label_change(self, combo_instance):
            # helper function that changes label
            label.text = str(round(slider.value, 3)) # TODO: ROUND VALUE. PLEASE.

        slider.bind(value=label_change) # changes label with change in value
        slider.bind(value=callback) # your callback here
        # Add the widgets in order
        self.add_widget(slider)
        self.add_widget(label)
        self.add_widget(Label(text='', size_hint=(0.15, 1.0)))

class SliderSubWidget(BoxLayout):
    # Abstracts away the rows of x dot, y dot, and theta dot.
    # Params are the name of the row (xdot, ydot, theta dot)
    # as well as a list of the callbacks for the three sliders. 
    # callback list must be: [whatever]dot, P, I, D.
    def __init__(self, subwidget_label, callback_list, **kwargs):
        super(SliderSubWidget, self).__init__(**kwargs)
        self.add_widget(Label(text=subwidget_label, size_hint=(0.3, 1.0)))
        for callback in callback_list:
            self.add_widget(SliderCombo(callback=callback))

class LabelRowSubWidget(BoxLayout):
    #  PID Labels. 
    def __init__(self, **kwargs):
        super(LabelRowSubWidget, self).__init__(**kwargs)
        self.add_widget(Label(text="", size_hint=(0.15, 1.0), width=60))
        self.add_widget(Label(text=""))
        self.add_widget(Label(text="P"))
        self.add_widget(Label(text="I"))
        self.add_widget(Label(text="D"))


class SliderWidget(BoxLayout):
    def __init__(self, **kwargs):
        super(SliderWidget, self).__init__(**kwargs)
        self.orientation = 'vertical'
        # callback lists:
        x_dot_callback_list = [
        self.x_callback,
        self.x_p_callback,
        self.x_i_callback,
        self.x_d_callback
        ]
        # labels
        self.add_widget(LabelRowSubWidget())
        # X dot row
        self.add_widget(SliderSubWidget('X Dot', x_dot_callback_list))
        # Y dot row
        self.add_widget(SliderSubWidget('Y Dot', x_dot_callback_list))
        # Z dot row
        self.add_widget(SliderSubWidget('Z Dot', x_dot_callback_list))

    def x_callback(self, instance, value):
        print "regular callback"

    def x_p_callback(self, instance, value):
        print "regular callback"

    def x_i_callback(self, instance, value):
        print "regular callback"

    def x_d_callback(self, instance, value):
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

