import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def cups_engaged(pressure_data):
    """"""

    max_cnt = 0

    for i, j, k, l in zip(pressure_data['elapsed_time_ms'], pressure_data['data_0'], pressure_data['data_1'], pressure_data['data_2']):

        cnt = 0
        if j < 600:
            cnt += 1
        if k < 600:
            cnt += 1
        if l < 600:
            cnt += 1

        if cnt > max_cnt:
            max_cnt = cnt

        if max_cnt == 3:
            print('success')
            break

    print(f'Maximum_count = {max_cnt}\n')


def extract_timestamp_and_distance(data):
    """Extracts timestamp and distance from the respective columns."""
    # Extract timestamp directly
    data['timestamp'] = pd.to_datetime(data['time'], format='%Y/%m/%d %H:%M:%S.%f')

    # Sort by timestamp
    data = data.sort_values('timestamp').reset_index(drop=True)

    # Convert timestamp to elapsed time in milliseconds
    data['elapsed_time_ms'] = (data['timestamp'] - data['timestamp'].iloc[0]).dt.total_seconds() * 1000

    # Extract Distance in mm from the 'data' column using regex
    distance_regex = r"Distance:\s*(\d+)\s*mm"
    data['Distance in mm'] = data['data'].str.extract(distance_regex).astype(float)

    return data

def extract_pressure_signals(data):
    """Extracts timestamp and pressure signals from the respective columns."""
    # Extract timestamp directly
    data['timestamp'] = pd.to_datetime(data['time'], format='%Y/%m/%d %H:%M:%S.%f')

    # Sort by timestamp
    data = data.sort_values('timestamp').reset_index(drop=True)

    # Convert timestamp to elapsed time in milliseconds
    data['elapsed_time_ms'] = (data['timestamp'] - data['timestamp'].iloc[0]).dt.total_seconds() * 1000

    # Return the relevant columns for plotting
    return data[['elapsed_time_ms', 'data_0', 'data_1', 'data_2']]

def find_and_plot_distance_csv(folder_path, output_pdf):
    """Generate distance plots."""
    with PdfPages(output_pdf) as pdf:
        file_paths_with_batches = []

        # Walk through the folder and subfolders
        for root, dirs, files in os.walk(folder_path):
            dirs[:] = [d for d in dirs if not d.startswith('pick_controller_')]

            # Ensure the directory name contains batch number >= 40
            batch_number = get_batch_number_from_path(root)
            if batch_number is None or batch_number < 40:
                continue  # Skip directories with batch numbers below 40

            for file in files:
                if file == "distance.csv":
                    file_path = os.path.join(root, file)
                    file_paths_with_batches.append((file_path, batch_number))

        # Sort by batch number (ascending order)
        file_paths_with_batches.sort(key=lambda x: x[1])

        for file_path, _ in file_paths_with_batches:
            try:
                # Read the distance.csv file
                distance_data = pd.read_csv(file_path)
                if 'time' in distance_data.columns and 'data' in distance_data.columns:
                    distance_data = extract_timestamp_and_distance(distance_data)

                    # Create the plot
                    plt.figure()
                    plt.plot(distance_data['elapsed_time_ms'], distance_data['Distance in mm'], label=os.path.relpath(file_path, folder_path))
                    plt.xlabel('Elapsed Time (ms)')
                    plt.ylabel('Distance (mm)')
                    plt.title(f"Distance Plot: {os.path.relpath(file_path, folder_path)}")
                    plt.legend()
                    plt.grid(True)

                    pdf.savefig()
                    plt.close()
            except Exception as e:
                print(f"Error reading {file_path}: {e}")

def find_and_plot_pressure_csv(folder_path, output_pdf):
    """Generate pressure plots."""
    with PdfPages(output_pdf) as pdf:
        file_paths_with_batches = []

        # Walk through the folder and subfolders
        for root, dirs, files in os.walk(folder_path):
            dirs[:] = [d for d in dirs if not d.startswith('pick_controller_')]

            # Ensure the directory name contains batch number >= 40
            batch_number = get_batch_number_from_path(root)
            if batch_number is None or batch_number < 40:
                continue  # Skip directories with batch numbers below 40

            for file in files:
                if file == "pressure.csv":
                    file_path = os.path.join(root, file)
                    file_paths_with_batches.append((file_path, batch_number))

        # Sort by batch number (ascending order)
        file_paths_with_batches.sort(key=lambda x: x[1])

        for file_path, _ in file_paths_with_batches:
            try:
                # Read the pressure.csv file
                pressure_data = pd.read_csv(file_path)
                if 'time' in pressure_data.columns and 'data_0' in pressure_data.columns and 'data_1' in pressure_data.columns and 'data_2' in pressure_data.columns:
                    pressure_data = extract_pressure_signals(pressure_data)        # Convert hPa to KPa

                    # Scale air-pressure to kPa (The original data is given in hPa)
                    pressure_data['data_0'] = pressure_data['data_0'] / 10
                    pressure_data['data_1'] = pressure_data['data_1'] / 10
                    pressure_data['data_2'] = pressure_data['data_2'] / 10

                    # Scale time to seconds.
                    pressure_data['elapsed_time_s'] = pressure_data['elapsed_time_ms'] / 1000

                    print(file_path)
                    cups_engaged(pressure_data)

                    # Create the plot
                    plt.figure(figsize=(6, 5))
                    plt.plot(pressure_data['elapsed_time_s'], pressure_data['data_0'], label='suction cup a', linestyle='-', color='c')
                    plt.plot(pressure_data['elapsed_time_s'], pressure_data['data_1'], label='suction cup b', linestyle='--', color='m')
                    plt.plot(pressure_data['elapsed_time_s'], pressure_data['data_2'], label='suction cup c', linestyle=':', color='y')
                    plt.xlabel('Elapsed Time [sec]')
                    plt.ylabel('Pressure [kPa]')
                    plt.title(f"Pressure Signals: {os.path.relpath(file_path, folder_path)}")
                    plt.legend()
                    plt.ylim([0, 110])
                    plt.grid(True)
                    plt.tight_layout()

                    pdf.savefig()
                    plt.show()
                    plt.close()
            except Exception as e:
                print(f"Error reading {file_path}: {e}")

def apply_moving_average(data, window_size=5):
    """Applies a moving average filter to smooth the data."""
    return data.rolling(window=window_size, min_periods=1).mean()

def plot_distance_and_pressure(folder_path, output_pdf):
    """Generate combined pressure and distance plots with distance on secondary y-axis."""
    with PdfPages(output_pdf) as pdf:
        file_paths_with_batches = []

        # Walk through the folder and subfolders
        for root, dirs, files in os.walk(folder_path):
            dirs[:] = [d for d in dirs if not d.startswith('pick_controller_')]

            # Ensure the directory name contains batch number >= 40
            batch_number = get_batch_number_from_path(root)
            if batch_number is None or batch_number < 40:
                continue  # Skip directories with batch numbers below 40

            for file in files:
                if file == "distance.csv":
                    file_path = os.path.join(root, file)
                    batch_number = get_batch_number_from_path(root)
                    file_paths_with_batches.append((file_path, batch_number))

        # Sort by batch number (ascending order)
        file_paths_with_batches.sort(key=lambda x: x[1])

        for file_path, _ in file_paths_with_batches:
            try:
                # Read the distance.csv file
                distance_data = pd.read_csv(file_path)
                print(f"Loaded distance data for {file_path}: {distance_data.shape}")

                if 'time' in distance_data.columns and 'data' in distance_data.columns:
                    distance_data = extract_timestamp_and_distance(distance_data)

                    # Apply moving average filter to the distance data to reduce noise
                    distance_data['Distance in mm'] = apply_moving_average(distance_data['Distance in mm'], window_size=5)

                    # Round the elapsed_time_ms to avoid precision issues during merge
                    distance_data['elapsed_time_ms'] = distance_data['elapsed_time_ms'].round(3)

                    # Find the corresponding pressure.csv file in the same folder
                    pressure_file_path = file_path.replace('distance.csv', 'pressure.csv')

                    if os.path.exists(pressure_file_path):
                        pressure_data = pd.read_csv(pressure_file_path)
                        print(f"Loaded pressure data for {pressure_file_path}: {pressure_data.shape}")

                        if 'time' in pressure_data.columns and 'data_0' in pressure_data.columns and 'data_1' in pressure_data.columns and 'data_2' in pressure_data.columns:
                            pressure_data = extract_pressure_signals(pressure_data)

                            # Round the elapsed_time_ms in the pressure data to match the distance data
                            pressure_data['elapsed_time_ms'] = pressure_data['elapsed_time_ms'].round(3)

                            # Create the combined plot
                            fig, ax1 = plt.subplots(figsize=(10, 6))

                            # Plot pressure signals on primary y-axis (left)
                            ax1.plot(pressure_data['elapsed_time_ms'], pressure_data['data_0'], label='suction cup a',
                                     linestyle='-', color='c')
                            ax1.plot(pressure_data['elapsed_time_ms'], pressure_data['data_1'], label='suction cup b',
                                     linestyle='--', color='m')
                            ax1.plot(pressure_data['elapsed_time_ms'], pressure_data['data_2'], label='suction cup c',
                                     linestyle=':', color='y')
                            ax1.set_xlabel('Elapsed Time (ms)')
                            ax1.set_ylabel('Pressure (Pa)', color='b')
                            ax1.tick_params(axis='y', labelcolor='b')

                            # Create the secondary y-axis (right) for distance
                            ax2 = ax1.twinx()
                            ax2.plot(distance_data['elapsed_time_ms'], distance_data['Distance in mm'],
                                     label='Distance (mm)', color='purple', linestyle='-.')
                            ax2.set_ylabel('Distance (mm)', color='purple')
                            ax2.tick_params(axis='y', labelcolor='purple')

                            # Set the title and legend
                            plt.title(
                                f"Pressure and Distance Plot: {os.path.relpath(file_path, folder_path)}")
                            ax1.legend(loc='upper left')
                            ax2.legend(loc='upper right')
                            plt.grid(True)

                            pdf.savefig(fig)
                            plt.close()
            except Exception as e:
                print(f"Error reading {file_path}: {e}")



def get_batch_number_from_path(path):
    """Extract the batch number from the folder path."""
    try:
        batch_part = [part for part in path.split(os.sep) if part.startswith('batch_')]
        if batch_part:
            batch_number = int(batch_part[0].split('_')[1])
            return batch_number
    except Exception as e:
        print(f"Error extracting batch number from path: {path}. Error: {e}")
    return None

def extract_wrench_forces(data):
    """Extracts the forces and converts the timestamp into elapsed time."""
    # Convert timestamp columns to a single timestamp (seconds + fractional seconds)
    data['timestamp'] = data['header_stamp_secs'] + data['header_stamp_nsecs'] * 1e-9

    # Convert timestamp to datetime, assuming it's in Unix epoch time
    data['timestamp'] = pd.to_datetime(data['timestamp'], unit='s', origin='unix')

    # Sort by timestamp
    data = data.sort_values('timestamp').reset_index(drop=True)

    # Convert timestamp to elapsed time in milliseconds
    data['elapsed_time_ms'] = (data['timestamp'] - data['timestamp'].iloc[0]).dt.total_seconds() * 1000

    # Return the relevant columns for plotting
    return data[['elapsed_time_ms', 'wrench_force_x', 'wrench_force_y', 'wrench_force_z']]

def find_and_plot_wrench_csv(folder_path, output_pdf):
    """Generate wrench force plots with fixed y-limits."""
    with PdfPages(output_pdf) as pdf:
        file_paths_with_batches = []

        # Walk through the folder and subfolders
        for root, dirs, files in os.walk(folder_path):
            dirs[:] = [d for d in dirs if not d.startswith('pick_controller_')]

            # Ensure the directory name contains batch number >= 40
            batch_number = get_batch_number_from_path(root)
            if batch_number is None or batch_number < 40:
                continue  # Skip directories with batch numbers below 40

            for file in files:
                if file == "wrench.csv":
                    file_path = os.path.join(root, file)
                    file_paths_with_batches.append((file_path, batch_number))

        # Sort by batch number (ascending order)
        file_paths_with_batches.sort(key=lambda x: x[1])

        # Second pass: Generate the plots with fixed y-limits
        for file_path, _ in file_paths_with_batches:
            try:
                # Read the wrench.csv file
                wrench_data = pd.read_csv(file_path)
                if 'header_stamp_secs' in wrench_data.columns and 'header_stamp_nsecs' in wrench_data.columns:
                    wrench_data = extract_wrench_forces(wrench_data)

                    # Create the plot
                    plt.figure(figsize=(10, 6))

                    # Plot forces in X, Y, and Z directions
                    plt.plot(wrench_data['elapsed_time_ms'], wrench_data['wrench_force_x'], label='Force X (N)', color='b')
                    plt.plot(wrench_data['elapsed_time_ms'], wrench_data['wrench_force_y'], label='Force Y (N)', color='g')
                    plt.plot(wrench_data['elapsed_time_ms'], wrench_data['wrench_force_z'], label='Force Z (N)', color='r')

                    # Set fixed y-limits for all plots
                    plt.ylim(-50, 50)

                    plt.xlabel('Elapsed Time (ms)')
                    plt.ylabel('Force (N)')
                    plt.title(f"Wrench Force Plot: {os.path.relpath(file_path, folder_path)}")
                    plt.legend()
                    plt.grid(True)

                    pdf.savefig()
                    plt.close()
            except Exception as e:
                print(f"Error reading {file_path}: {e}")


if __name__ == "__main__":

    ### Adjust plot parameters (for papers) ###
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman"]
    plt.rcParams["font.size"] = 18
    plt.rc('legend', fontsize=18) # using a size in points

    # input_folder = "/home/alejo/Projects/Prosser2025_Dataset/Data"
    input_folder = "C:/Users/avela/OneDrive/Documents/01 Research/Data"

    # Specify the output PDF files
    distance_pdf_path = input_folder + "/distance_plots.pdf"
    pressure_pdf_path = input_folder + "/pressure_plots.pdf"
    combined_pdf_path = input_folder + "/combined_plots.pdf"
    wrench_pdf_path = input_folder + "/wrench_plots.pdf"  # New PDF path for wrench plots

    # Generate the distance plots and save them to the PDF
    # find_and_plot_distance_csv(input_folder, distance_pdf_path)

    # Generate the pressure plots and save them to the PDF
    find_and_plot_pressure_csv(input_folder, pressure_pdf_path)

    # Generate the combined pressure and distance plots and save them to the PDF
    # plot_distance_and_pressure(input_folder, combined_pdf_path)

    # Generate the wrench force plots and save them to the PDF
    # find_and_plot_wrench_csv(input_folder, wrench_pdf_path)

    print(f"Distance plots have been saved to {distance_pdf_path}.")
    print(f"Pressure plots have been saved to {pressure_pdf_path}.")
    print(f"Combined plots have been saved to {combined_pdf_path}.")
    print(f"Wrench force plots have been saved to {wrench_pdf_path}.")  # New print statement
