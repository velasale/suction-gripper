import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
from matplotlib.animation import FFMpegWriter
import matplotlib.patches as patches
from PIL import Image
import matplotlib as mpl

# ----------------------- SETTINGS ------------------------------------#
environment = 'ubuntu'
base_paths = {
    'ubuntu': '/media/alejo/Elements/Alejo - Air Pressure Servoing/Joes Videos/',
    'windows': r'D:/Alejo - Air Pressure Servoing/Joes Videos/'
}
path = base_paths[environment]

trial = '20240911__stf_high__offset_3__seq_1__rep_todo/'

# --- Air pressure settings ---
pressure_threshold = 60.0  # kPa
colors = ['r', 'orange', 'b']  # color for each sensor
vector_angles_deg = [60, 180, 300]  # vector directions
unit_vectors = np.array([[np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a))] for a in vector_angles_deg])

# ------------------- FIGURE STYLE -------------------
mpl.rcParams.update({
    'text.usetex': False,         # disable system LaTeX
    'mathtext.fontset': 'cm',     # Computer Modern fonts
    'font.family': 'serif',
    'font.size': 14,
    'axes.titlesize': 16,
    'axes.labelsize': 14,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12
})


# ----------------------- DATA LOADING ------------------------------------#
def load_pressure_csvs(path, trial, sensors=3):
    dfs = [pd.read_csv(os.path.join(path, trial, f"gripper-pressure-sc{i+1}.csv")) for i in range(sensors)]
    t_min = min(df['Time'].min() for df in dfs)
    t_max = max(df['Time'].max() for df in dfs)
    dt = min(np.median(np.diff(df['Time'])) for df in dfs)
    t_uniform = np.arange(t_min, t_max, dt)
    magnitudes = np.vstack([np.interp(t_uniform, df['Time'], df['data']) for df in dfs]).T / 10
    return t_uniform, magnitudes

t_uniform, magnitudes = load_pressure_csvs(path, trial)
elapsed_time = t_uniform - t_uniform[0]
n_frames = len(elapsed_time)


# --- Load camera images ---
image_folder = os.path.join(path, trial, 'pngs_fixed_cam/')
image_files = sorted(glob.glob(os.path.join(image_folder, '*.png')),
                     key=lambda f: int(os.path.splitext(os.path.basename(f))[0]))
img_timestamps = np.array([int(os.path.splitext(os.path.basename(f))[0])/10 for f in image_files])


# ----------------------- FIGURE SETUP ------------------------------------#
fig = plt.figure(figsize=(18,5))
fig_width, fig_height = fig.get_size_inches()
h_axes, bottom, margin, min_w2 = 0.75, 0.1, 0.05, 0.25

# Load background image once
script_dir = os.path.dirname(os.path.abspath(__file__))
img_bg_path = os.path.normpath(os.path.join(script_dir, '..', 'media', 'gripper_topview.png'))
bg_img = plt.imread(img_bg_path)
h1, w1 = bg_img.shape[:2]
aspect1 = w1/h1

# Load sample camera image to get aspect
sample_img = Image.open(image_files[0])
h3, w3 = sample_img.size[1], sample_img.size[0]
aspect3 = w3/h3

# --- Compute normalized widths ---
w1_norm = aspect1 * h_axes / (fig_height / fig_width)
w3_norm = aspect3 * h_axes / (fig_height / fig_width)
total_w = w1_norm + w3_norm + 3*margin
if total_w > 1 - min_w2:
    scale = (1 - min_w2 - 3*margin) / (w1_norm + w3_norm)
    w1_norm *= scale
    w3_norm *= scale
w2_norm = 1 - w1_norm - w3_norm - 3*margin

left1 = margin
left2 = left1 + w1_norm + margin
left3 = left2 + w2_norm + margin

# --- Create axes ---
ax2 = fig.add_axes([left1, bottom, w2_norm, h_axes])
ax1 = fig.add_axes([left2, bottom, w1_norm, h_axes])
ax3 = fig.add_axes([left3, bottom, w3_norm, h_axes])

# Camera subplot
img_display = ax3.imshow(sample_img)
# ax3.set_aspect(aspect3,adjustable='box')
ax3.set_aspect(1.0)
ax3.axis('off')
ax3.set_title("Camera View")

# Vector subplot setup
max_val = np.max(magnitudes)
ax1.set_xlim(-max_val*1.2, max_val*1.2)
ax1.set_ylim(-max_val*1.2, max_val*1.2)
ax1.set_title("Air Pressure Vectors")
ax1.spines['left'].set_position('center')
ax1.spines['bottom'].set_position('center')
ax1.spines['right'].set_color('none')
ax1.spines['top'].set_color('none')
ax1.xaxis.set_ticks_position('bottom')
ax1.yaxis.set_ticks_position('left')
ax1.imshow(bg_img, extent=[-max_val*1.2, max_val*1.2, -max_val*1.2, max_val*1.2], zorder=0, alpha=0.5)
ax1.set_aspect(w1/h1, adjustable='box')

# Suction cups and lines
cup_coords = [(70*np.cos(np.deg2rad(a)), 70*np.sin(np.deg2rad(a))) for a in [60,180,-60]]
cup_colors = ['red', 'orange', 'blue']
circles = [ax1.add_patch(patches.Circle(c, radius=25, color=col, alpha=0.3, visible=False))
           for c, col in zip(cup_coords, cup_colors)]
cup_line = ax1.add_line(Line2D([0,0],[0,0], color='gray', linestyle='--', linewidth=1.5, visible=False))
projection_line = ax1.add_line(Line2D([0,0],[0,0], color='purple', linestyle='--', linewidth=1.5, visible=False))
engaged_label = ax1.text(0, -max_val*1.3, "All suction cups engaged!", ha='center', va='top', fontsize=18, color='green', weight='bold')
engaged_label.set_visible(False)

# Quivers
quiver = ax1.quiver([0]*3, [0]*3, [0]*3, [0]*3, angles='xy', scale_units='xy', scale=1, color=colors)
quiver_sum = ax1.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1, color='k', width=0.02)
quiver_perp = ax1.quiver([0], [0], [0], [0], angles='xy', scale_units='xy', scale=1, color='purple', width=0.02)
texts = [ax1.text(0,0,'', color=c, fontsize=12, weight='bold') for c in colors]
txt_sum = ax1.text(0,0,'', color='k', fontsize=12, weight='bold')
txt_perp = ax1.text(0,0,'', color='purple', fontsize=12, weight='bold')

# Time series subplot
ax2.plot(elapsed_time, magnitudes[:,0], color='r', label='scA')
ax2.plot(elapsed_time, magnitudes[:,1], color='orange', label='scB')
ax2.plot(elapsed_time, magnitudes[:,2], color='b', label='scC')
ax2.set_xlim(elapsed_time[0], elapsed_time[-1])
ax2.set_ylim(0, max_val*1.1)
ax2.set_title("Time Series")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Air Pressure [kPa]")
ax2.axhline(y=pressure_threshold, color='green', linestyle='--', linewidth=2, label='Engagement Threshold')
time_marker = ax2.axvline(elapsed_time[0], color='k', linestyle=(0,(5,5)))
ax2.legend()
ax2.grid(True)


# -------------------------- UPDATE FUNCTION ---------------------------------
def update(frame):
    mags = magnitudes[frame]
    U = mags * unit_vectors[:,0]
    V = mags * unit_vectors[:,1]
    quiver.set_UVC(U, V)

    for i, txt in enumerate(texts):
        txt.set_position((U[i], V[i]))
        txt.set_text(f"sc{['A','B','C'][i]}: {mags[i]:.2f} kPa")

    U_sum, V_sum = sum(U), sum(V)
    quiver_sum.set_UVC([U_sum], [V_sum])
    txt_sum.set_position((U_sum, V_sum))
    txt_sum.set_text(f"Sum: {np.linalg.norm([U_sum,V_sum]):.1f} kPa")

    active_idx = [i for i, m in enumerate(mags) if m < pressure_threshold]
    for i, c in enumerate(circles):
        c.set_visible(i in active_idx)
    engaged_label.set_visible(len(active_idx)==3)

    origin_x, origin_y = 0,0
    cup_line.set_visible(False)
    projection_line.set_visible(False)

    if len(active_idx)==1:
        origin_x, origin_y = cup_coords[active_idx[0]]
    elif len(active_idx)==2:
        idx1, idx2 = active_idx
        x1, y1 = cup_coords[idx1]
        x2, y2 = cup_coords[idx2]
        cup_line.set_data([x1,x2],[y1,y2])
        cup_line.set_visible(True)
        line_vec = np.array([x2-x1, y2-y1])
        line_start = np.array([x1, y1])
        sum_vec = np.array([U_sum, V_sum])
        try:
            s, t = np.linalg.lstsq(np.column_stack((line_vec,-sum_vec)), -line_start, rcond=None)[0]
        except np.linalg.LinAlgError:
            s = 0.5
        s = np.clip(s,0,1)
        origin_x, origin_y = line_start + s*line_vec
        projection_line.set_data([origin_x, origin_x+U_sum], [origin_y, origin_y+V_sum])
        projection_line.set_visible(True)

    U_perp, V_perp = -V_sum, U_sum
    quiver_perp.set_offsets([[origin_x, origin_y]])
    quiver_perp.set_UVC([U_perp], [V_perp])
    txt_perp.set_position((origin_x+U_perp+2, origin_y+V_perp+2))
    txt_perp.set_text("$\\omega$ (⊥ Sum)")

    time_marker.set_xdata(elapsed_time[frame])
    idx_img = np.argmin(np.abs(img_timestamps - elapsed_time[frame]))
    img_display.set_data(np.array(Image.open(image_files[idx_img])))

    return quiver, quiver_sum, quiver_perp, time_marker, *texts, txt_sum, txt_perp, img_display, cup_line, projection_line, engaged_label


# --- Animate ---
interval_ms = np.diff(elapsed_time, prepend=elapsed_time[0]).mean()*1000*4  # slower
ani = animation.FuncAnimation(fig, update, frames=n_frames, interval=interval_ms, blit=True)
writer = FFMpegWriter(fps=30/4, metadata=dict(artist="Me"), bitrate=1800)
ani.save("vectors_with_timeseries.mp4", writer=writer)

plt.show()
