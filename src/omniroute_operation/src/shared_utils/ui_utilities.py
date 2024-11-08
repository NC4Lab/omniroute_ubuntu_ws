#!/usr/bin/env python
"""
Class for common UI operations.
"""

# PyQt and PySide Imports
from PyQt5.QtWidgets import QApplication, QWidget


class UIUtilities:
    
    @staticmethod
    def set_fixed_size(widget: QWidget):
        """
        Set the fixed size of the given widget based on its current dimensions.

        Arguments:
        - widget: The QWidget to set the size for.
        """
        # Set the fixed size of the main window based on the dimensions from the UI file
        main_window_width = widget.geometry().width()
        main_window_height = widget.geometry().height()
        widget.setFixedSize(main_window_width, main_window_height)

        # Set the size hint of the main window to match the size of the widget
        widget.window().setMinimumSize(main_window_width, main_window_height)
        widget.window().setMaximumSize(main_window_width, main_window_height)

    @staticmethod
    def move_ui_window(widget: QWidget, horizontal_alignment: str, vertical_alignment: str, offset: int = 100):
        """
        Move the given widget to the specified position on the given monitor, with an optional offset from the screen edges.

        Arguments:
        - widget: The widget or window to move.
        - horizontal_alignment: A string specifying the horizontal alignment. Valid values are 'left', 'center', and 'right'.
        - vertical_alignment: A string specifying the vertical alignment. Valid values are 'top', 'middle', and 'bottom'.
        - offset: An integer specifying the offset from the screen edges in pixels. Ignored for 'center' or 'middle' alignments.
        """
        # Get the geometry of the main monitor to determine available screen dimensions
        screen_geometry = QApplication.desktop().screenGeometry(0)

        # Determine the horizontal position based on the specified alignment
        if horizontal_alignment == "left":
            x = offset  # Align to the left edge with the given offset
        elif horizontal_alignment == "center":
            x = (screen_geometry.width() - widget.width()) / \
                2  # Center horizontally on the screen
        elif horizontal_alignment == "right":
            # Align to the right edge with the given offset
            x = screen_geometry.width() - widget.width() - offset
        else:
            raise ValueError(
                f"Invalid horizontal_alignment value: {horizontal_alignment}")

        # Determine the vertical position based on the specified alignment
        if vertical_alignment == "top":
            y = offset  # Align to the top edge with the given offset
        elif vertical_alignment == "middle":
            y = (screen_geometry.height() - widget.height()) / \
                2  # Center vertically on the screen
        elif vertical_alignment == "bottom":
            # Align to the bottom edge with the given offset
            y = screen_geometry.height() - widget.height() - offset
        else:
            raise ValueError(
                f"Invalid vertical_alignment value: {vertical_alignment}")

        # Move the window to the calculated position on the screen
        widget.window().move(x, y)
