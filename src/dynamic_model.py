import math
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = ["Times New Roman"]

# --- Plot Font Sizes ---
SMALL_SIZE = 12
MEDIUM_SIZE = 14
BIGGER_SIZE = 16

# --- Image size ---
x_size = 3.5
y_size = 4

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

#### Power Screw ###
# --- Parameters ---
pitch = 2/1000              # [m]
starts = 4
diameter = 8/1000           # [m]
mu = 0.2                   # Brass (Nut material) and Steel (Screw material)
beta = 0.968                # ACME Thread geometry parameter  = cos(14.5deg)
d_m = diameter - pitch/2    # mean diameter[m]
l = pitch * starts          # pitch [m]
print('lead angle [deg]: ', math.degrees(math.atan(l/(math.pi*d_m))))

# --- Force to raise load ---
T = 0.4                     # N.m
efficiency = 1            # From friction, manufacturing tolerances,
factor = (l + math.pi * mu * d_m / beta) / (math.pi * d_m - mu * l / beta)
F_nut = (2 * T / d_m) * factor * efficiency
print('Given a torque motor of %.2f [Nm], the nut force is: %.2f F_nut' %(T, F_nut))
print('The power screw factor is: ', factor)

x = []
y = []
f_outs = []
f_outs_per_finger = []
T_motors = []
levers = []

for i in range(150):

    # --- Vary distance [mm] ---
    stepper_distance = initial_stepper_distance + i/10
    d = 90 - 7 - stepper_distance
    x.append(stepper_distance)

    # --- Angles ---
    alfa = math.atan(a / d)
    gamma = math.acos((c ** 2 + b ** 2 - a ** 2 - d ** 2) / (2 * b * c))
    theta = math.asin(c * math.sin(gamma) / math.sqrt(a ** 2 + d ** 2))

    lever = alfa + theta
    lever_in_deg = lever * 180 / math.pi
    levers.append(lever_in_deg)

    # --- Forces ratio ---
    ratio = (c/e) * math.sin(gamma) / math.cos(alfa + theta)
    y.append(ratio)

    # --- APPROACH 1: Given the Motor Torque, find the output force
    efficiency = 1.0
    F_out = F_nut * ratio * efficiency
    f_outs.append(F_out)
    f_outs_per_finger.append(F_out/3)

    # --- APPROACH 2: Given the Output Force, find the Motor Torque
    F_out2 = 30
    F_nut2 = F_out2 / ratio
    T = F_nut2 * d_m * factor / 2
    T_motors.append(T)

    # print(d, alfa_deg, ratio)

fig, ax = plt.subplots(figsize=(x_size, y_size))
ax.plot(x, y, c='r', label='Force ratio')
ax.set_xlabel('nut travel distance [mm]')
ax.set_ylabel('Force transmission ratio Fout/Fnut')
ax.legend(loc='lower right')
plt.grid()
ax2 = ax.twinx()
ax2.plot(x, levers, label=r"$\alpha$ + $\theta$", linestyle='dashed')
ax2.set_ylabel(r"$\alpha$ + $\theta$ [deg]")
ax2.legend(loc='upper left')
plt.tight_layout()

fig = plt.figure(figsize=(x_size, y_size))
# plt.plot(x, f_outs, c='r', label='total')
plt.plot(x, f_outs_per_finger, c='orange', label='per finger (total/3)')
plt.xlabel('nut travel distance [mm]')
plt.ylabel('push force [N]')
plt.tight_layout()

# Plot apple bruising threshold
thr_press = 0.29e6  # Pa (Li et al. 2016)
finger_width = 20  # mm
thr_force = thr_press * (10 ** 2) / 1e6
print(thr_force)
# plt.hlines(y=thr_force, xmin=1285, xmax=1425, linestyles='--', lw=2, label='Apple Bruising threshold')
plt.hlines(y=thr_force, xmin=min(x), xmax=max(x), linestyles='--', lw=2, label='apple bruising threshold')
plt.ylim([0, 35])
plt.xlim([52, 60])
plt.legend()
plt.grid()
plt.tight_layout()

fig = plt.figure(figsize=(x_size, y_size))
plt.plot(x, T_motors, c='r')
plt.xlabel('nut travel distance [mm]')
plt.ylabel('Motor Torque [N.m]')
plt.grid()
plt.tight_layout()

plt.show()






