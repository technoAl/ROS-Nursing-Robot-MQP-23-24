#!/usr/bin/env python3

from asciimatics.widgets import Frame, Layout, Divider, Button
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.exceptions import ResizeScreenError, NextScene, StopApplication
import sys
import os


class TabButtons(Layout):
    def __init__(self, frame, active_tab_idx):
        cols = [1, 1, 1, 1, 1]
        super().__init__(cols)
        self._frame = frame
        self._frame.background = "black"
        for i, _ in enumerate(cols):
            self.add_widget(Divider(), i)
        btns = [Button("Interfaces", self._on_click_1),
                Button("Start Trial", self._on_click_2),
                Button("Btn3", self._on_click_3),
                Button("Btn4", self._on_click_4),
                Button("Quit", self._on_click_Q)]
        for i, btn in enumerate(btns):
            self.add_widget(btn, i)
        btns[active_tab_idx].disabled = True

    def _on_click_1(self):
        raise NextScene("Interfaces")

    def _on_click_2(self):
        raise NextScene("Tab2")

    def _on_click_3(self):
        raise NextScene("Tab3")

    def _on_click_4(self):
        raise NextScene("Tab4")

    def _on_click_Q(self):
        raise StopApplication("Quit")


class RootPage(Frame):
    def __init__(self, screen):
        super().__init__(screen,
                         screen.height,
                         screen.width,
                         can_scroll=False,
                         title="Root Page")
        layout1 = Layout([1, 1], fill_frame=True)
        self.add_layout(layout1)
        # add your widgets here
        btns = [Button("Drag", self._intDrag),
                Button("Gaze", self._intGaze),
                Button("Point", self._intPoint),
                Button("Drag Select", self._intDragSel),
                Button("Gaze Select", self._intGazeSel),
                Button("Point Select", self._intPointSel)]
        for i, btn in enumerate(btns):
            layout1.add_widget(btn, 0)

        btns = [Button("Show Lables", self._intShowLables),
                Button("Hide Lables", self._intHideLables)]
        for i, btn in enumerate(btns):
            layout1.add_widget(btn, 1)

        layout2 = TabButtons(self, 0)
        self.add_layout(layout2)
        self.fix()

    def _intDrag(self):
        os.system("rostopic pub -1 /statemanager/command std_msgs/String \"data: selector_type drag\"> /dev/null&")

    def _intDragSel(self):
        os.system(
            "rostopic pub -1 /statemanager/command std_msgs/String \"data: selector_type drag_select\"> /dev/null&")

    def _intPointSel(self):
        os.system(
            "rostopic pub -1 /statemanager/command std_msgs/String \"data: selector_type point_select\"> /dev/null&")

    def _intPoint(self):
        os.system(
            "rostopic pub -1 /statemanager/command std_msgs/String \"data: selector_type point\"> /dev/null&")

    def _intGaze(self):
        os.system("rostopic pub -1 /statemanager/command std_msgs/String \"data: selector_type gaze\"> /dev/null&")

    def _intGazeSel(self):
        os.system("rostopic pub -1 /statemanager/command std_msgs/String \"data: selector_type gaze_select\"> /dev/null&")

    def _intShowLables(self):
        os.system("rostopic pub -1 /statemanager/command std_msgs/String \"data: show lables\"> /dev/null&")

    def _intHideLables(self):
        os.system("rostopic pub -1 /statemanager/command std_msgs/String \"data: hide lables\"> /dev/null&")


class AlphaPage(Frame):
    def __init__(self, screen):
        super().__init__(screen,
                         screen.height,
                         screen.width,
                         can_scroll=False,
                         title="Trials")
        layout1 = Layout([1], fill_frame=True)
        self.add_layout(layout1)
        # add your widgets here
        btns = [Button("Start Trial", self._starttrial)]
        for i, btn in enumerate(btns):
            layout1.add_widget(btn, i)

        layout2 = TabButtons(self, 1)
        self.add_layout(layout2)
        self.fix()

    def start(self):
        raise NextScene("RunTab")

    def _starttrial(self):
        os.system("rostopic pub -1 /statemanager/command std_msgs/String \"data: trial start\"> /dev/null&")


class BravoPage(Frame):
    def __init__(self, screen):
        super().__init__(screen,
                         screen.height,
                         screen.width,
                         can_scroll=False,
                         title="Bravo Page")
        layout1 = Layout([1], fill_frame=True)
        self.add_layout(layout1)
        # add your widgets here

        layout2 = TabButtons(self, 2)
        self.add_layout(layout2)
        self.fix()


class CharliePage(Frame):
    def __init__(self, screen):
        super().__init__(screen,
                         screen.height,
                         screen.width,
                         can_scroll=False,
                         title="Charlie Page")
        layout1 = Layout([1], fill_frame=True)
        self.add_layout(layout1)
        # add your widgets here

        layout2 = TabButtons(self, 3)
        self.add_layout(layout2)
        self.fix()


class TrialRun(Frame):

    def __init__(self, screen):
        super().__init__(screen,
                         screen.height,
                         screen.width,
                         can_scroll=False,
                         title="Trial Run")
        layout1 = Layout([1], fill_frame=True)
        self.add_layout(layout1)

        layout2 = TabButtons(self, 3)
        self.add_layout(layout2)
        self.fix()


def demo(screen, scene):
    scenes = [
        Scene([RootPage(screen)], -1, name="Interfaces"),
        Scene([AlphaPage(screen)], -1, name="Tab2"),
        Scene([BravoPage(screen)], -1, name="Tab3"),
        Scene([CharliePage(screen)], -1, name="Tab4"),
        Scene([TrialRun(screen)], -1, name="RunTab"),
    ]
    screen.play(scenes, stop_on_resize=True, start_scene=scene, allow_int=True)


last_scene = None
while True:
    try:
        Screen.wrapper(demo, catch_interrupt=True, arguments=[last_scene])
        sys.exit(0)
    except ResizeScreenError as e:
        last_scene = e.scene