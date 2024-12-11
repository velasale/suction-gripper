import math
import matplotlib.pyplot as plt


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

    def factor(self):

        num_factor = self.thread_length + math.pi * self.mean_diameter * self.mu / self.beta
        den_factor = math.pi * self.mean_diameter - self.mu * self.thread_length / self.beta
        factor = num_factor / den_factor

        return factor


class CamDrivenFinger():

    def __init__(self):

        # Mechanism lengths and distances
        self.px = 12.13             # horizontal distance between pivot and nut
        self.lb = 18.5              # length of connecting bar
        self.lk = 17.5              # length of knuckle
        self.lf = 47.81             # vertical distance to point of contact with apple

        # Mechanism angles
        self.alfa = 0.0
        self.gamma = 0.0
        self.theta = 0.0

    def ratio(self, distance):

        # Four-bar mechanism angles
        self.alfa = math.atan(self.px / distance)

        num_gamma = self.lk ** 2 + self.lb ** 2 - self.px ** 2 - distance ** 2
        den_gamma = 2 * self.lb * self.lk
        self.gamma = math.acos(num_gamma / den_gamma)

        num_theta = self.lk * math.sin(self.gamma)
        den_theta = math.sqrt(self.px ** 2 + distance ** 2)
        self.theta = math.asin(num_theta / den_theta)

        lever_angle = self.alfa + self.theta
        lever_angle_deg = lever_angle * 180 / math.pi

        # Four-bar mechanism force ratio
        num_ratio = self.lk * math.sin(self.gamma)
        den_ratio = self.lf * math.cos(lever_angle)
        ratio = num_ratio / den_ratio

        return lever_angle_deg, ratio


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
    motor_torque = 0.4  # N.m
    efficiency = 1  # From friction, manufacturing tolerances,
    F_nut = 2 * efficiency * motor_torque / (screw.mean_diameter * screw.factor())
    # F_nut = 2 * efficiency * motor_torque * screw.factor() / screw.mean_diameter

    print('Given a torque motor of %.2f [Nm], the nut force is: %.2f F_nut' % (motor_torque, F_nut))
    print('The power screw factor is: ', screw.factor())

    # Book-keeping
    x = []
    cam_ratios = []
    f_normals_per_finger = []
    f_pulls = []
    required_torques = []
    lever_angles = []

    displ_thr = 58
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
        required_torque = required_nut_force * screw.mean_diameter * screw.factor() / 2
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

    return x, cam_ratios, f_normals_per_finger, required_torques, lever_angles, f_pulls


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

    x, cam_force_ratios, f_normals_per_finger, T_motors, levers, pull_forces = cam_driven_finger_model()

    ### Figure 1: Force Tranmission Ratio vs Distance ###
    # Source: https://onelinerhub.com/python-matplotlib/how-to-add-third-y-axis
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

    plt.show()


if __name__ == '__main__':
    main()



