class PlotFormat:
    def __init__(self, x_label, y_label, subplot=(1,1), title=None) -> None:
        self.subplot = subplot
        self.x_label = x_label
        self.y_label = y_label
        self.title = title