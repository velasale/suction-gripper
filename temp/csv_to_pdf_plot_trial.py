import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def extract_timestamp_and_distance(data):
    """Extracts timestamp and distance from the respective columns."""
    # Extract timestamp directly
    data['timestamp'] = pd.to_datetime(data['time'], format='%Y/%m/%d %H:%M:%S.%f')
    
    # Sort by timestamp
    data = data.sort_values('timestamp').reset_index(drop=True)
    
    # Convert timestamp to elapsed time in milliseconds
    data['elapsed_time_ms'] = (data['timestamp'] - data['timestamp'].iloc[0]).dt.total_seconds() * 1000
    
    # Extract Distance in mm from the 'data' column
    data['Distance in mm'] = data['data'].str.extract(r'Distance:\\s(\\d+)').astype(float)
    
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
    # Create a PDF file to store the plots
    with PdfPages(output_pdf) as pdf:
        # Walk through the folder and subfolders
        for root, _, files in os.walk(folder_path):
            for file in files:
                if file == "distance.csv":
                    # Construct the full file path
                    file_path = os.path.join(root, file)

                    # Read the CSV file
                    try:
                        data = pd.read_csv(file_path)

                        # Ensure the CSV has the required columns
                        if 'time' in data.columns and 'data' in data.columns:
                            # Extract timestamp and distance
                            data = extract_timestamp_and_distance(data)

                            # Create a plot
                            plt.figure()
                            plt.plot(data['elapsed_time_ms'], data['Distance in mm'], label=os.path.relpath(file_path, folder_path))
                            plt.xlabel('Elapsed Time (ms)')
                            plt.ylabel('Distance (mm)')
                            plt.title(f"Distance Plot: {os.path.relpath(file_path, folder_path)}")
                            plt.legend()
                            plt.grid(True)

                            # Add the plot to the PDF
                            pdf.savefig()
                            plt.close()
                        else:
                            print(f"Skipping file {file_path}: Missing required columns.")
                    except Exception as e:
                        print(f"Error reading file {file_path}: {e}")

def find_and_plot_pressure_csv(folder_path, output_pdf):
    # Create a PDF file to store the plots
    with PdfPages(output_pdf) as pdf:
        # Walk through the folder and subfolders
        for root, _, files in os.walk(folder_path):
            for file in files:
                if file == "pressure.csv":
                    # Construct the full file path
                    file_path = os.path.join(root, file)

                    # Read the CSV file
                    try:
                        data = pd.read_csv(file_path)

                        # Ensure the CSV has the required columns
                        if 'time' in data.columns and 'data_0' in data.columns and 'data_1' in data.columns and 'data_2' in data.columns:
                            # Extract timestamp and pressure signals
                            data = extract_pressure_signals(data)

                            # Create a plot for each pressure signal
                            plt.figure()
                            plt.plot(data['elapsed_time_ms'], data['data_0'], label='Suction Cup A')
                            plt.plot(data['elapsed_time_ms'], data['data_1'], label='Suction Cup B')
                            plt.plot(data['elapsed_time_ms'], data['data_2'], label='Suction Cup C')
                            plt.xlabel('Elapsed Time (ms)')
                            plt.ylabel('Pressure')
                            plt.title(f"Pressure Signals: {os.path.relpath(file_path, folder_path)}")
                            plt.legend()
                            plt.grid(True)

                            # Add the plot to the PDF
                            pdf.savefig()
                            plt.close()
                        else:
                            print(f"Skipping file {file_path}: Missing required columns.")
                    except Exception as e:
                        print(f"Error reading file {file_path}: {e}")

if __name__ == "__main__":
    # Specify the folder containing the subfolders with distance.csv and pressure.csv files
    input_folder = "/media/alejo/VELASALE/Data"

    # Specify the output PDF files
    distance_pdf_path = "/media/alejo/VELASALE/Data/distance_plots.pdf"
    pressure_pdf_path = "/media/alejo/VELASALE/Data/pressure_plots.pdf"

    # Generate the distance plots and save them to the PDF
    find_and_plot_distance_csv(input_folder, distance_pdf_path)

    # Generate the pressure plots and save them to the PDF
    find_and_plot_pressure_csv(input_folder, pressure_pdf_path)

    print(f"Distance plots have been saved to {distance_pdf_path}.")
    print(f"Pressure plots have been saved to {pressure_pdf_path}.")