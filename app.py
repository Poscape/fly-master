from PyQt6.QtWidgets import QApplication
import sys, os

from main_window import MainWindow

if __name__ == "__main__":
    os.system("python -m PyQt6.uic.pyuic master.ui -o ui.py")

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())