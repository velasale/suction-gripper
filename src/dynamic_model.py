import math
import matplotlib.pyplot as plt


def cam_driven_finger_model():
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
    pitch = 2 / 1000  # [m]
    starts = 4
    diameter = 8 / 1000  # [m]
    mu = 0.2  # Brass (Nut material) and Steel (Screw material)
    beta = 0.968  # ACME Thread geometry parameter  = cos(14.5deg)
    d_m = diameter - pitch / 2  # mean diameter[m]
    l = pitch * starts  # pitch [m]
    print('lead angle [deg]: ', math.degrees(math.atan(l / (math.pi * d_m))))

    # --- Force to raise load ---
    T = 0.4  # N.m
    efficiency = 1  # From friction, manufacturing tolerances,
    factor = (l + math.pi * mu * d_m / beta) / (math.pi * d_m - mu * l / beta)
    F_nut = (2 * T / d_m) * factor * efficiency
    print('Given a torque motor of %.2f [Nm], the nut force is: %.2f F_nut' % (T, F_nut))
    print('The power screw factor is: ', factor)

    x = []
    y = []
    f_outs = []
    f_outs_per_finger = []
    T_motors = []
    levers = []

    displ_thr = 58

    for i in range(150):

        # --- Vary distance [mm] ---
        stepper_distance = initial_stepper_distance + i / 10
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
        ratio = (c / e) * math.sin(gamma) / math.cos(alfa + theta)
        y.append(ratio)

        # --- APPROACH 1: Given the Motor Torque, find the output force
        efficiency = 1.0
        if stepper_distance <= displ_thr:
            F_out = F_nut * ratio * efficiency
        else:
            pass
        f_outs.append(F_out)
        f_outs_per_finger.append(F_out / 3)

        if stepper_distance == 51.5:
            print('Force ratio at 51.5mm', ratio)

        # --- APPROACH 2: Given the Output Force, find the Motor Torque
        F_out2 = 30
        F_nut2 = F_out2 / ratio
        T = F_nut2 * d_m * factor / 2
        T_motors.append(T)

        # print(d, alfa_deg, ratio)

    return x, y, f_outs_per_finger, T_motors, levers


def main():
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman"]

    # --- Plot Font Sizes ---
    SMALL_SIZE = 22
    MEDIUM_SIZE = 22
    BIGGER_SIZE = 22

    # --- Image size ---
    x_size = 10.5
    y_size = 6.5

    plt.rc('font', size=SMALL_SIZE)  # controls default text sizes
    plt.rc('axes', titlesize=SMALL_SIZE)  # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

    x, y, f_outs_per_finger, T_motors, levers = cam_driven_finger_model()

    ### Figure 1: Force Tranmission Ratio vs Distance ###
    # Source: https://onelinerhub.com/python-matplotlib/how-to-add-third-y-axis
    fig, ax = plt.subplots(figsize=(x_size, y_size))
    twin1 = ax.twinx()
    twin2 = ax.twinx()

    twin2.spines.right.set_position(("axes", 1.15))

    p1, = ax.plot(x, y, c='k', label=r"$F_{out}$ / $F_{nut}$ ratio", linewidth=2)
    ax.set_xlabel('x [mm]')
    ax.set_ylabel(r"$F_{out}$/$F_{nut}$ ratio [-]")
    # ax.legend(loc='lower left')
    ax.grid()

    p2, = twin1.plot(x, levers, label=r"$\alpha$ + $\theta$", linestyle='dashed', c='g', linewidth=2)
    twin1.set_ylabel(r"$\alpha$ + $\theta$ [deg]", c='g')
    # twin1.legend(loc='upper right')
    travel_limit = 59.5
    # ax.vlines(x=travel_limit, ymin=min(y), ymax=max(y), linestyles='dotted', color='k', lw=2, label='apple bruising threshold')

    p3, = twin2.plot(x, T_motors, c='b', linestyle='dotted', label='Motor torque', linewidth=2)
    twin2.set_ylabel(r"$T_{motor}$ [N.m]", c='b')
    # twin2.legend(loc='upper left')

    ax.tick_params(axis='y', colors=p1.get_color())
    twin1.tick_params(axis='y', colors=p2.get_color())
    twin2.tick_params(axis='y', colors=p3.get_color())

    plt.tight_layout()

    ### Figure 2: Push Force vs Distance ###
    fig = plt.figure(figsize=(x_size, y_size))
    # plt.plot(x, f_outs, c='r', label='total')
    plt.plot(x, f_outs_per_finger, c='orange', label='per finger (total/3)')
    plt.xlabel('nut travel distance [mm]')
    plt.ylabel('push force [N]')
    plt.tight_layout()
    thr_press = 0.29e6  # Pa (Li et al. 2016)
    finger_width = 20  # mm
    thr_force = thr_press * (10 ** 2) / 1e6
    print(thr_force)
    # plt.hlines(y=thr_force, xmin=1285, xmax=1425, linestyles='--', lw=2, label='Apple Bruising threshold')
    plt.hlines(y=thr_force, xmin=min(x), xmax=max(x), linestyles='--', lw=2, label='apple bruising threshold')
    plt.ylim([0, 35])
    plt.xlim([52, 60])
    plt.legend()
    # plt.grid()
    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()



