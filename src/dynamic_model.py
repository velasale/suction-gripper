import math
import matplotlib.pyplot as plt
import numpy as np
from data_analysis_mark10 import *
from matplotlib.patches import Patch

def latex_figure_settings():
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman"]

    # --- Plot Font Sizes ---
    SMALL_SIZE = 19
    MEDIUM_SIZE = 24
    BIGGER_SIZE = 24

    # --- Image size ---


    plt.rc('font', size=SMALL_SIZE)  # controls default text sizes
    plt.rc('axes', titlesize=SMALL_SIZE)  # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title


class AcmeLeadScrew():
    def __init__(self):

        # ACME screw parameters
        self.pitch = 2/1000                                             # Pitch [m]
        self.n_starts = 4                                               # Number of starts [-]
        self.external_diameter = 8/1000                                 # Screw external diameter [m]
        self.mu = 0.2                                                   # Brass (Nut material) and Steel (Screw material)
        self.beta = 0.968148                                            # ACME Thread geometry parameter  = cos(14.5deg)
        self.mean_diameter = self.external_diameter - self.pitch / 2    # mean diameter[m]
        self.thread_length = self.pitch * self.n_starts                 # pitch [m]

        print('ACME lead angle [deg]: ', math.degrees(math.atan(self.thread_length / (math.pi * self.mean_diameter))))

        self.factors()

    def factors(self):

        num_factor = self.mean_diameter * (self.thread_length + math.pi * self.mean_diameter * self.mu / self.beta)
        den_factor = 2 * (math.pi * self.mean_diameter - self.mu * self.thread_length / self.beta)
        self.factor_force_into_torque = num_factor / den_factor      # Fnut = c * Torque
        self.factor_torque_into_force = den_factor / num_factor      # Torque = c * Fnut

        print('Power screw factor: ', self.factor_torque_into_force, self.factor_force_into_torque)


class CamDrivenFinger():

    def __init__(self):

        # Mechanism lengths and distances
        self.d1 = 47.81             # vertical distance to point of contact with apple
        self.d2 = 7.5               # horizontal distance to point of contact with apple
        self.d5 = 18.5              # length of connecting bar
        self.d6 = 17.5              # length of knuckle
        self.d7 = 12.13             # horizontal distance between pivot and nut

        # Friction Coefficients
        self.mu = 0.9               # TBD Friction coefficient between apple and fingers
        self.num_fingers = 3

        # Bar-Linkage angles
        self.alfa = 0.0
        self.gamma = 0.0
        self.theta = 0.0

        # Experiment parameters
        self.apple_radius = 80      # in Millimeters
        self.clamping_force = 8     # in Newtons
        self.finger_offset = 0
        self.psi = math.tan(self.finger_offset / self.apple_radius)
        self.omega = 0              # angle between gripper and apple main axis
        self.beta = 0               # angle between pull force and apple main axis

        # Experiment forces
        self.forces_array = []

    def ratio(self, distance):

        # Four-bar mechanism angles
        self.alfa = math.atan(self.d7 / distance)       # RAL25 paper equation No. 3

        num_gamma = self.d6 ** 2 + self.d5 ** 2 - self.d7 ** 2 - distance ** 2
        den_gamma = 2 * self.d5 * self.d6
        self.gamma = math.acos(num_gamma / den_gamma)   # RAL25 paper equation No. 2

        num_theta = self.d6 * math.sin(self.gamma)
        den_theta = math.sqrt(self.d7 ** 2 + distance ** 2)
        self.theta = math.asin(num_theta / den_theta)   # RAL25 paper equation No. 4

        lever_angle = self.alfa + self.theta
        lever_angle_deg = lever_angle * 180 / math.pi

        # Four-bar mechanism force ratio
        num_ratio = self.d6 * math.sin(self.gamma)
        den_ratio = self.num_fingers * (self.d1 - self.mu * self.d2) * math.cos(lever_angle)
        ratio = num_ratio / den_ratio

        return lever_angle_deg, ratio

    def forces(self):
        self.psi = math.asin(self.finger_offset / self.apple_radius)

        # APPROACH: System of Eqns with normal force limit
        A = np.array([[1, 0, 0, 0, 0, 0, 0, 0],   # Superposition Theorem Finger 1
                      [0, 0, 0, 1, math.tan(self.psi), 0, 0, -math.sin(self.omega + self.beta) / (2 * 0.5 * math.cos(self.psi))],   # Superposition Theorem
                      [-math.sin(self.psi), math.cos(self.psi), 0, -2 * math.sin(self.psi), 2 * math.cos(self.psi), 0, 0, -math.cos(self.omega + self.beta)],   # Balance of Forces y
                      [0, -1, 0, 0, 2 * 0.5, 0, 0, math.sin(self.beta)],                        # Balance of Moments in apple
                      [-self.d1, self.d2, self.d6 * math.sin(self.gamma), 0, 0, 0, 0, 0],       # Balance of Moments in finger 1
                      [0, 0, 0, -self.d1, self.d2, self.d6 * math.sin(self.gamma), 0, 0],       # Balance of Moments in finger 2
                      [0, 0, math.cos(self.alfa + self.theta), 0, 0, 2 * math.cos(self.alfa + self.theta), -1, 0],  # Balance of forces in nut
                      [self.mu, -1, 0, 0, 0, 0, 0, 0]])     # Friction and Normal force relationship for finger 1

        B = ([[self.clamping_force],
              [self.clamping_force],
              [0],
              [0],
              [0],
              [0],
              [0],
              [0]])

        # Forces array: [Normal-1  Friction-1  Flink-1  Normal-2  Friction-2   Flink-2  Fnut  Fpull]
        self.forces_array = np.dot(np.linalg.pinv(A), B)
        print(f'Cam Driven finger forces=,\n {self.forces_array}')

        return self.forces_array



def cam_driven_finger_model():

    n_fingers = 3
    pick_force = 15                 # Force required to pick an apple [N]
    apple_finger_friction = 0.5
    friction_force_per_finger = pick_force / n_fingers
    normal_force_per_finger = friction_force_per_finger / apple_finger_friction

    camfinger = CamDrivenFinger()

    # --- Lead Screw Transmission ---
    # https://www.engineersedge.com/mechanics_machines/power_screws_design_13982.htm
    # https://learning.hccs.edu/faculty/edward.osakue/dftg-2306-lecture-notes/Unit%209-Power%20Screws.pdf
    # https://uni.edu/~rao/MD-18%20Power%20screws.pdf

    screw = AcmeLeadScrew()

    # --- Given a torqueForce to raise load ---
    motor_torque = 0.4              # N.m
    efficiency = 1                  # From friction, manufacturing tolerances,
    F_nut = efficiency * motor_torque * screw.factor_torque_into_force
    print('Given a torque motor of %.2f [Nm], the nut force is: %.2f F_nut' % (motor_torque, F_nut))

    # Book-keeping
    x = []
    cam_ratios = []
    f_normals_per_finger = []
    f_pulls = []
    required_torques = []
    lever_angles = []

    displ_thr = 58.5
    initial_stepper_distance = 50
    for i in range(150):

        # --- Vary distance [mm] ---
        stepper_distance = initial_stepper_distance + i / 10
        d = 90 - 7 - stepper_distance
        x.append(stepper_distance)

        lever_angle, cam_ratio = camfinger.ratio(d)
        lever_angles.append(lever_angle)
        cam_ratios.append(cam_ratio)

        # --- APPROACH 1: Given the output normal, find the required Torque
        required_nut_force = normal_force_per_finger * n_fingers / cam_ratio
        required_torque = required_nut_force * screw.factor_force_into_torque
        required_torques.append(required_torque)

        # --- APPROACH 2: Given the torque, find the output force
        if stepper_distance <= displ_thr:
            f_normal = (F_nut / n_fingers) * cam_ratio * efficiency
            f_friction = f_normal * apple_finger_friction
            f_pull = 3 * f_friction
        else:
            pass
        f_normals_per_finger.append(f_normal)
        f_pulls.append(f_pull)

        # Print values
        print(f'Stepper distance: {stepper_distance:.2f}, Gamma: {camfinger.gamma:.2f}, Alpha: {camfinger.alfa:.2f}, Theta: {camfinger.theta:.2f}')


    return x, cam_ratios, f_normals_per_finger, required_torques, lever_angles, f_pulls


def main():

    x, cam_force_ratios, f_normals_per_finger, T_motors, levers, pull_forces = cam_driven_finger_model()

    ### Figure 1: Force Tranmission Ratio vs Distance ###
    # Source: https://onelinerhub.com/python-matplotlib/how-to-add-third-y-axis
    latex_figure_settings()
    x_size = 10.5
    y_size = 6.5
    fig, ax = plt.subplots(figsize=(x_size, y_size))
    twin1 = ax.twinx()
    twin2 = ax.twinx()

    twin2.spines.right.set_position(("axes", 1.15))

    p1, = ax.plot(x, cam_force_ratios, c='k', label=r"$F_{normal}$ / $F_{nut}$ ratio", linewidth=2)
    ax.set_xlabel('x [mm]')
    ax.set_ylabel(r"$F_{normal}$/$F_{nut}$ ratio [-]")
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
    plt.plot(x, f_normals_per_finger, c='orange', label='per finger (total/3)')
    plt.plot(x, pull_forces, c='red', label='total pull force')
    plt.xlabel('nut travel distance [mm]')
    plt.ylabel('push force [N]')
    plt.tight_layout()
    thr_press = 0.29e6  # Pa (Li et al. 2016)
    finger_width = 20  # mm
    thr_force = thr_press * (10 ** 2) / 1e6
    print(thr_force)
    # plt.hlines(y=thr_force, xmin=1285, xmax=1425, linestyles='--', lw=2, label='Apple Bruising threshold')
    plt.hlines(y=thr_force, xmin=min(x), xmax=max(x), linestyles='--', lw=2, label='apple bruising threshold')
    # plt.ylim([0, 35])
    plt.xlim([52, 60])
    plt.legend()
    # plt.grid()
    plt.tight_layout()


    # ------------------------------------------------------------------
    # First configure bar-linkage at the desired position
    camfinger = CamDrivenFinger()
    displacement = 58
    _, _ = camfinger.ratio(90 - 7 - displacement)
    x_size = 9
    y_size = 7.5

    # ------------------ Experiment 1: Finger offsets ----------------------
    boxwidth = 3*20/45
    xloc_delta = 1.1* boxwidth
    mus = [0.6, 0.8, 1.0]
    offsets = [0, 5, 10, 15, 20]

    for mu in mus:
        camfinger.mu = mu
        Fpulls = []

        for offset in offsets:
            camfinger.finger_offset = offset
            # print(camfinger.alfa, camfinger.theta, camfinger.gamma)
            x = camfinger.forces()
            Fpulls.append(float(x[7]))

        if mu == mus[0]:
            Fpulls_min = Fpulls
        elif mu == mus[1]:
            Fpulls_mid = Fpulls
        elif mu == mus[2]:
            Fpulls_max = Fpulls

    fig, ax = plt.subplots(figsize=(x_size, y_size))
    model_plot, = ax.plot(offsets, Fpulls_mid, '-o', color='green', label='fingers model')
    ax.fill_between(offsets, Fpulls_min, Fpulls_max, color='green', alpha=.2)
    ax.set_ylim([0, 60])
    # ax.grid()
    ax.set_ylabel('Force [N]')
    ax.set_xlabel('offset [mm]')

    Fpulls_fingers = [[25.35, 22.95, 19.0, 26.05, 22.1], [17.25, 19.25, 16.25, 19.6, 21.95],
                       [11.4, 14.7, 10.55, 13.5, 15.55],
                       [10.65, 11.55, 11.4, 11.4, 12.5], [7.8, 8.25, 8.0, 5.05, 9.35]]

    ax.boxplot(Fpulls_fingers, positions=offsets, widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='green', color='black'),
               medianprops=dict(color='red')
               )

    Fpulls_dual = np.array(Fpulls_fingers) + 12.12          # simply add the suction force
    Fpulls_dual = Fpulls_dual.tolist()
    ax.boxplot(Fpulls_dual, positions=[0+xloc_delta, 5+xloc_delta, 10+xloc_delta, 15+xloc_delta, 20+xloc_delta], widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='orange', color='black'),
               medianprops=dict(color='red')
               )

    Fpulls_suction = [[12.35, 11.9, 12.5, 11.9, 12.25, 11.95, 11.85, 12.3, 12.25, 11.95, 11.9],
                      [12.85, 13.0, 12.6, 11.75, 11.7, 13.6], [11.65, 13.0, 13.8], [13.35, 12.0, 13.7], [13.35, 12.0, 13.7]]
    ax.boxplot(Fpulls_suction, positions=[0 - xloc_delta, 5 - xloc_delta, 10 - xloc_delta, 15 - xloc_delta, 20 - xloc_delta],
               widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='skyblue', color='black'),
               medianprops=dict(color='red')
               )

    ref_fdf = ax.axhline(y=16, xmin=0, xmax=20, linestyle='--', lw=2, label='median FDF', color='k')

    # Add legend manually with proxy artists
    legend_handles = [
        ref_fdf,
        Patch(facecolor='orange', edgecolor='black', label='tandem meas.'),
        model_plot,
        Patch(facecolor='green', edgecolor='black', label='fingers meas.'),
        Patch(facecolor='skyblue', edgecolor='black', label='suction meas.')
    ]
    ax.legend(handles=legend_handles, loc='upper left')
    ax.set_xticks([])
    ax.set_xticks([0, 5, 10, 15, 20])
    ax.grid()
    plt.tight_layout()

    # ------------------ Experiment 2: Angled pull-back -----------------------------
    camfinger.finger_offset = 0
    omegas = [0, 15, 30, 45]
    for mu in mus:
        camfinger.mu = mu
        Fpulls = []

        for omega in omegas:
            camfinger.omega = math.radians(omega)
            x = camfinger.forces()
            Fpulls.append(float(x[7]))

        if mu == mus[0]:
            Fpulls_min = Fpulls
        elif mu == mus[1]:
            Fpulls_mid = Fpulls
        elif mu == mus[2]:
            Fpulls_max = Fpulls

    fig, ax = plt.subplots(figsize=(x_size, y_size))
    model_plot, = ax.plot(omegas, Fpulls_mid, '-o', color='green', label='fingers model')
    ax.fill_between(omegas, Fpulls_min, Fpulls_max, color='green', alpha=.2)

    # Measurement values
    Fpulls_fingers = [[25.35, 22.95, 19.0, 26.05, 22.1], [26.45, 27.8, 25.1, 32.6, 18.5, 12.6], [23.5, 19.6, 25.15, 28.7, 13.4], [16.6, 29.95, 29.25, 37.5, 31.25]]
    boxwidth = 3
    xloc_delta = 1.1* boxwidth
    ax.boxplot(Fpulls_fingers, positions=omegas, widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='green', color='black'),
               medianprops=dict(color='red')
               )

    Fpulls_dual = [[33.95, 33.8, 36.9, 34.65, 32.1], [36.6, 32.1, 49.5, 35.6, 36.65, 38.9, 30.9, 23.2, 27.95, 29.6, 32.1], [36.7, 36.1, 33.1, 38.7, 39.2, 38.3], [36.75, 31.7, 47.9, 39.85, 39.3]]
    ax.boxplot(Fpulls_dual, positions=[0+xloc_delta,15+xloc_delta,30+xloc_delta,45+xloc_delta], widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='orange', color='black'),
               medianprops=dict(color='red')
               )

    Fpulls_suction = [[12.35, 11.9, 12.5, 11.9, 12.25, 11.95, 11.85, 12.3, 12.25, 11.95, 11.9], [12.85, 13.0, 12.6, 11.75, 11.7, 13.6], [11.65, 13.0, 13.8], [13.35, 12.0, 13.7]]
    ax.boxplot(Fpulls_suction, positions=[0 - xloc_delta, 15 - xloc_delta, 30 - xloc_delta, 45 - xloc_delta], widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='skyblue', color='black'),
               medianprops=dict(color='red')
               )

    ref_fdf = ax.axhline(y=16, xmin=0, xmax=5, linestyle='--', lw=2, label='median FDF', color='k')

    # Add legend manually with proxy artists
    legend_handles = [
        ref_fdf,
        Patch(facecolor='orange', edgecolor='black', label='tandem meas.'),
        model_plot,
        Patch(facecolor='green', edgecolor='black', label='fingers meas.'),
        Patch(facecolor='skyblue', edgecolor='black', label='suction meas.')
    ]
    ax.legend(handles=legend_handles)
    ax.set_xticks([])
    ax.set_xticks([0, 15, 30, 45])
    ax.set_xticklabels(['0°', '15°', '30°', '45°'])
    ax.set_ylim([0, 60])
    ax.set_ylabel('Force [N]')
    ax.set_xlabel('$\omega$')
    ax.grid()
    plt.tight_layout()

    #------------------------------------ Experiment 3: Angled pull back with Beta ------------------------------------
    camfinger.finger_offset = 0
    camfinger.beta = math.radians(90)
    omegas = [0, 15, 30, 45]
    for mu in mus:
        camfinger.mu = mu
        Fpulls = []

        for omega in omegas:
            camfinger.omega = math.radians(omega)
            x = camfinger.forces()
            Fpulls.append(float(x[7]))

        if mu == mus[0]:
            Fpulls_min = Fpulls
        elif mu == mus[1]:
            Fpulls_mid = Fpulls
        elif mu == mus[2]:
            Fpulls_max = Fpulls

    fig, ax = plt.subplots(figsize=(x_size, y_size))
    model_plot, = ax.plot(omegas, Fpulls_mid, '-o', color='green', label='fingers model')
    ax.fill_between(omegas, Fpulls_min, Fpulls_max, color='green', alpha=.2)

    # Mark10 Measurements
    boxwidth = 3
    xloc_delta = 1.1* boxwidth

    Fpulls_dual = [22.2, 22.9, 23.24, 22.18, 21.18]
    Fpulls_suction = [5.3, 5.18, 5.64, 5.18, 4.96]
    Fpulls_fingers = [14.82, 12.04, 9.74, 16.3, 13.56, 17.52]
    ax.boxplot(Fpulls_suction, positions=[0-xloc_delta], widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='skyblue', color='black'),
               medianprops=dict(color='red')
               )

    ax.boxplot(Fpulls_fingers, positions=[0], widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='green', color='black'),
               medianprops=dict(color='red')
               )

    ax.boxplot(Fpulls_dual, positions=[0+xloc_delta], widths=boxwidth,
               patch_artist=True,
               boxprops=dict(facecolor='orange', color='black'),
               medianprops=dict(color='red')
               )

    ref_fdf = ax.axhline(y=16, xmin=0, xmax=45, linestyle='--', lw=2, label='median FDF', color='k')

    # Add legend manually with proxy artists
    legend_handles = [
        ref_fdf,
        Patch(facecolor='orange', edgecolor='black', label='tandem meas.'),
        model_plot,
        Patch(facecolor='green', edgecolor='black', label='fingers meas.'),
        Patch(facecolor='skyblue', edgecolor='black', label='suction meas.')
    ]
    ax.legend(handles=legend_handles, loc='upper left')
    ax.set_xticks([])
    ax.set_xticks([0, 15, 30, 45])
    ax.set_xticklabels(['0°', '15°', '30°', '45°'])
    ax.set_ylim([0, 60])
    ax.set_ylabel('Force [N]')
    ax.set_xlabel('$\omega$')
    ax.grid()
    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()



