from .database_connection import get_database_connection


def create_tables():
    """Initialize the database"""
    connection = get_database_connection()
    cursor = connection.cursor()

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS robot (
            id INTEGER PRIMARY KEY,
            data TEXT
        );
    """)

    connection.commit()
