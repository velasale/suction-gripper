import os
import csv
import sqlite3


def process_folders(root_folder):

    for dirpath, dirnames, dirfiles in os.walk(root_folder, topdown=True):

        for filename in dirfiles:
            if filename.endswith(".db3"):
                # print(filename)
                print(dirpath)

                db3_file_path = os.path.join(dirpath, filename)
                topics = extract_topics(db3_file_path)    
                print(topics)
                                  

def inspect_db3_schema():
    """ Inspect the schema of the .db3 file to find the correct table and column name"""

    db3_file_path = '/home/alejo/Downloads/day_2/batch_2/apple_0/pressure_servo_20241023_110932.db3/pressure_servo_20241023_110932.db3_0.db3'
             
    try:
        # Connect to the SQLite .db3 file
        conn = sqlite3.connect(db3_file_path)
        cursor = conn.cursor()

        # List all tables in the database
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = cursor.fetchall()
        print("Tables in database:", tables)

        # For each table, list the columns
        for table in tables:
            table_name = table[0]
            print(f"Columns in table {table_name}:")
            cursor.execute(f"PRAGMA table_info({table_name});")
            columns = cursor.fetchall()
            for column in columns:
                print(f"  {column[1]}")  # Column name is in the second position of the tuple        

        # Close the connection
        conn.close()        
    
    except sqlite3.Error as e:
        print(f'SQLite error during schema inspection: {e}')


def extract_topics(db3_file_path):
    """Extract topics from a ROS2 .db3 file"""

    try: 
        
        # Connect to SQLite .db3 file
        conn = sqlite3.connect(db3_file_path)
        cursor = conn.cursor()

        # Query to get topics
        cursor.execute("SELECT DISTINCT name FROM topics")
        topics = cursor.fetchall()

        # Close connection
        conn.close()

        # Return list of topics
        return topics
    
    except sqlite3.OperationlError as e:
        print(f'SQLite operational error: {e}')
        return []
    
    except sqlite3.Error as e:
        print(f'SQLite error: {e}')
        return []
    
    except Exception as e:
        print(f'Unexpected error opening {db3_file_path}: {e}')
        return []


def main():
    root_folder = '/media/alejo/Elements/Alejo - Apple Pick Data/Real Apple Picks/06 - 2024 fall (Prosser-WA)'
    
    # inspect_db3_schema()          # Run this first to understand how the db3 file is structured
    process_folders(root_folder)


if __name__ == "__main__":

    main()