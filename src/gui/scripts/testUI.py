#!/usr/bin/python
import pacmanGUI



class GUIController(pacmanGUI.MainWindowController):
        def getResourceFolder(self):
                return "/home/max/git/Pacman/pacman_ws/src/pacman_config"
        def toggleEStop(self, active):
                print(active)

if __name__ == '__main__':
        controller = GUIController()
        pacmanGUI.initGUI(controller)
        pacmanGUI.runGUI()
