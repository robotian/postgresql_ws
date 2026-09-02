import psycopg
from psycopg import OperationalError

def test_connection():
    try:
        # Connect to your PostgreSQL server
        connection = psycopg.connect(
            dbname="ros2",  # Replace with your database name
            user="admin",  # Replace with your username
            password="Robotlab2019",  # Replace with your password
            host="localhost",  # Replace with your host (e.g., 'localhost' or an IP address)
            port="5432"  # Default PostgreSQL port
        )
        
        # If the connection is successful
        print("Connection successful!")
        
    except OperationalError as e:
        # Handle connection error
        print(f"Error: {e}")
        
    finally:
        # Close the connection
        if connection:
            connection.close()

# Run the function to test the connection
test_connection()
