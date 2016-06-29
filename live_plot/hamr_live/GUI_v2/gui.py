from kivy.app import App
from kivy.lang import Builder
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
import dashboard as Dashboard
import graphs as Graphs

class MainLayout(BoxLayout):
    def __init__(self, **kwargs):
        super(MainLayout, self).__init__(**kwargs)
        self.orientation = 'vertical'
        self.add_widget(Graphs.Layout())
        self.add_widget(Dashboard.Layout(size_hint=(1.0, 0.3)))

class TestApp(App):
    def build(self):
        return MainLayout()


if __name__ == '__main__':
    TestApp().run()