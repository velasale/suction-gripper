
# Sources
# https://stackoverflow.com/questions/46789053/python3-tkinter-analog-gauge
# https://stackoverflow.com/questions/74835321/update-python-gauge

import tkinter as tk
import tk_tools
import random
import tkinter as tk
from math import pi, cos, sin

# ---------- First Example
# root = tk.Tk()
#
# p = tk_tools.RotaryScale(root, max_value=100.0, unit='psi')
# p.grid()
#
# count = 30.0
# p.set_value(count)
#
# def update_gauge():
#     global p, count
#     count += 1.0
#     p.set_value(random.randrange(100))
#     root.after(50, update_gauge)
#
# root.after(50, update_gauge)
# root.mainloop()


# -------------- Second Example
class Meter(tk.Frame):
    def __init__(self, master=None, **kw):
        tk.Frame.__init__(self, master, **kw)

        self.var = tk.IntVar(self, 0)

        self.canvas = tk.Canvas(self, width=200, height=110,
                                borderwidth=2, relief='sunken',
                                bg='white')
        self.scale = tk.Scale(self, orient='horizontal', from_=0, to=100, variable=self.var)

        self.meter = self.canvas.create_line(100, 100, 10, 100,
                                             fill='black',
                                             width=3,
                                             arrow='last')
        self.angle = 0
        self.updateMeterLine(0.2)

        self.canvas.create_arc(10, 10, 190, 190, extent=108, start=36,
                               style='arc', outline='red')

        self.canvas.pack(fill='both')
        self.scale.pack()

        self.var.trace_add('write', self.updateMeter)  # if this line raises an error, change it to the old way of adding a trace: self.var.trace('w', self.updateMeter)

    def updateMeterLine(self, a):
        """Draw a meter line"""
        self.angle = a
        x = 100 - 90 * cos(a * pi)
        y = 100 - 90 * sin(a * pi)
        self.canvas.coords(self.meter, 100, 100, x, y)

    def updateMeter(self, name1, name2, op):
        """Convert variable to angle on trace"""
        mini = self.scale.cget('from')
        maxi = self.scale.cget('to')
        # self.var.set(7)
        # pos = (self.var.get() - mini) / (maxi - mini)
        # print(self.var.set(outside))
        pos = (self.var.get() - mini) / (maxi - mini)
        self.updateMeterLine(pos * 0.6 + 0.2)



if __name__ == '__main__':
    root = tk.Tk()
    meter = Meter(root)
    meter.pack()
    # root.after(50, meter.updateMeter(random.randrange(100)))
    root.mainloop()