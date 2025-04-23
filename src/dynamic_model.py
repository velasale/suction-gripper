import math
import matplotlib.pyplot as plt
import numpy as np

def latex_figure_settings():
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman"]

    # --- Plot Font Sizes ---
    SMALL_SIZE = 22
    MEDIUM_SIZE = 22
    BIGGER_SIZE = 22

    # --- Image size ---


    plt.rc('font', size=SMALL_SIZE)  # controls default text sizes
    plt.rc('axes', titlesize=SMALL_SIZE)  # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=SMALL_SIZE)  # fontsize of the tick labels
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
        self.psi = math.tan(self.finger_offset / self.apple_radius)

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

    # Experiment 1: Fruit offsets
    mus = [0.8, 0.9, 1.0]
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

    fig = plt.figure(figsize=(x_size, y_size))
    plt.plot(offsets, Fpulls_mid, '-o', color='green')
    plt.fill_between(offsets, Fpulls_min, Fpulls_max, color='green', alpha=.2)
    plt.ylim([0, 60])
    plt.grid()
    plt.ylabel('Force[N]')
    plt.xlabel('offset[mm]')

    fig = plt.figure(figsize=(x_size, y_size))
    plt.plot(offsets, Fpulls_mid, '--o', color='black')
    # plt.fill_between(omegas, Fpulls_min, Fpulls_max, color='green', alpha=.2)
    plt.ylim([0, 60])
    plt.grid()
    plt.ylabel('Force[N]')
    plt.xlabel('offset[deg]')

    # Experiment 2: Angled pull-back
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

    fig = plt.figure(figsize=(x_size, y_size))
    plt.plot(omegas, Fpulls_mid, '-o', color='green')
    plt.fill_between(omegas, Fpulls_min, Fpulls_max, color='green', alpha=.2)
    plt.ylim([0, 60])
    plt.grid()
    plt.ylabel('Force[N]')
    plt.xlabel('omega[deg]')

    plt.show()


if __name__ == '__main__':
    main()



