import math
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = ["Times New Roman"]

# --- Plot Font Sizes ---
SMALL_SIZE = 12
MEDIUM_SIZE = 14
BIGGER_SIZE = 16

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

# --- Cam Mechanism Parameters ---
a = 12.13
b = 18.5
c = 17.5
e = 47.81

initial_stepper_distance = 50

# --- Lead Screw Transmission ---
# https://www.engineersedge.com/mechanics_machines/power_screws_design_13982.htm
# https://learning.hccs.edu/faculty/edward.osakue/dftg-2306-lecture-notes/Unit%209-Power%20Screws.pdf
# https://uni.edu/~rao/MD-18%20Power%20screws.pdf
T = 0.4                     # N.m
pitch = 2/1000              # [m]
starts = 4
diameter = 8/1000           # [m]
mu = 0.25                   # Nut material: Brass
beta = 0.968                # ACME Thread geometry parameter  = cos(14.5deg)
d_m = diameter - pitch/2    # mean diameter[m]
L = pitch * starts          # pitch [m]

efficiency = 0.125

F_nut = (2 * T / d_m) * (math.pi * d_m * beta - mu * L) / (math.pi * mu * d_m + L * beta) * efficiency
print(F_nut)

x = []
y = []
f_outs = []

for i in range(150):

    # --- Vary distance [mm] ---
    stepper_distance = initial_stepper_distance + i/10
    d = 90 - 7 - stepper_distance

    x.append(stepper_distance)

    # --- Angles ---
    alfa = math.atan(a / d)
    gamma = math.acos((c ** 2 + b ** 2 - a ** 2 - d ** 2) / (2 * b * c))
    theta = math.asin(c * math.sin(gamma) / math.sqrt(a ** 2 + d ** 2))

    # --- Force ratio ---
    ratio = (c/e) * math.sin(gamma) / math.cos(alfa + theta)

    F_out = F_nut * ratio
    f_outs.append(F_out)

    y.append(ratio)
    # print(d, alfa_deg, ratio)

fig = plt.figure()
plt.plot(x, y, c='r')
plt.xlabel('nut travel distance [mm]')
plt.ylabel('Force transmission ratio Fout/Fnut')
plt.grid()

fig = plt.figure()
plt.plot(x, f_outs, c='r')
plt.xlabel('nut travel distance [mm]')
plt.ylabel('Output Force [N]')
plt.grid()

plt.show()






