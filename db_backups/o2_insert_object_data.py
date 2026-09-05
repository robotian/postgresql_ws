import psycopg
import os

# 1. Configuration
DB_PARAMS = {
    'host': 'localhost',
    'port': 5432,
    'dbname': 'ros2',
    'user': 'admin',
    'password': 'Robotlab2019'
}

CSV_FILE = 'objects_table.csv'
TABLE_NAME = 'object_data'

def load_csv_to_postgres():
    # Verify file exists before attempting database connection
    if not os.path.exists(CSV_FILE):
        print(f"Error: The file '{CSV_FILE}' was not found.")
        return

    conn = None
    cur = None
    try:
        # Connect to the PostgreSQL database
        conn = psycopg.connect(**DB_PARAMS)
        cur = conn.cursor()

        # 2. Create the table if it does not exist
        # Note: "object" is enclosed in double quotes as it can be a reserved word in SQL environments
        create_table_query = f"""
        CREATE TABLE IF NOT EXISTS "{TABLE_NAME}" (
            object_id INTEGER PRIMARY KEY,
            row_id INTEGER,
            x_coord DOUBLE PRECISION,
            y_coord DOUBLE PRECISION,
            orientation_deg DOUBLE PRECISION
        );
        """
        cur.execute(create_table_query)

        # 3. Truncate the table to clear out any existing data
        truncate_query = f'TRUNCATE TABLE "{TABLE_NAME}";'
        cur.execute(truncate_query)
        print(f"Table '{TABLE_NAME}' ensured and truncated.")

        # 4. Read the CSV and insert data using the highly efficient COPY command
        with open(CSV_FILE, 'r') as f:
            copy_sql = f"""
            COPY "{TABLE_NAME}" (object_id, row_id, x_coord, y_coord, orientation_deg)
            FROM STDIN WITH (FORMAT CSV, HEADER TRUE, DELIMITER ',')
            """
            with cur.copy(copy_sql) as copy:
                copy.write(f.read())

        # Commit the transaction
        conn.commit()
        print(f"Successfully loaded data from '{CSV_FILE}' into the '{TABLE_NAME}' table.")

    except psycopg.DatabaseError as e:
        # Rollback in case of any database errors
        if conn:
            conn.rollback()
        print(f"Database error occurred: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # 5. Clean up connection resources
        if cur:
            cur.close()
        if conn:
            conn.close()

if __name__ == '__main__':
    load_csv_to_postgres()