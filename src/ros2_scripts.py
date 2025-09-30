import os
import csv
import sqlite3


def process_folders(root_folder):

    for dirpath, dirnames, dirfiles in os.walk(root_folder, topdown=True):

        for filename in dirfiles:
            if filename.endswith(".db3"):
                db3_file_path = os.path.join(dirpath, filename)
                print(f'\nFile: {db3_file_path}')
                topics = extract_topics(db3_file_path)    
                print(topics)
                                  

def inspect_db3_schema():
    """ Inspect the schema of the .db3 file to find the correct table and column name"""

    db3_file_path = '/home/alejo/franka_bags/franka_joint_bag1/franka_joint_bag_0.db3'
             
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
        conn = sqlite3.connect(db3_file_path)
        cursor = conn.cursor()

        # Get all topics with their type
        cursor.execute("SELECT id, name, type FROM topics")
        topics = cursor.fetchall()

        # Print them
        for topic in topics:
            topic_id, topic_name, topic_type = topic
            print(f"  id={topic_id}, name={topic_name}, type={topic_type}")

        conn.close()
        return topics

    except sqlite3.Error as e:
        print(f'SQLite error: {e}')
        return []



def main():
    root_folder = '/media/alejo/Elements/Alejo - Apple Pick Data/Real Apple Picks/06 - 2024 fall (Prosser-WA)'

    root_folder = '/home/alejo/Documents/temporal/'

    root_folder = "/home/alejo/franka_bags/"
    
    # inspect_db3_schema()          # Run this first to understand how the db3 file is structured
    process_folders(root_folder)


if __name__ == "__main__":

    main()