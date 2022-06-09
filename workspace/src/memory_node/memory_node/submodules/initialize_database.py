from .database_connection import get_database_connection


def create_tables():
    """Initialize the database"""
    connection = get_database_connection()
    cursor = connection.cursor()

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS robot
        (
            id   INTEGER PRIMARY KEY,
            data TEXT
        );
        CREATE TABLE IF NOT EXISTS qr
        (
            content     TEXT UNIQUE PRIMARY KEY,
            coordinates TEXT
        );
        CREATE TABLE IF NOT EXISTS history
        (
            content     TEXT UNIQUE PRIMARY KEY,
            coordinates TEXT,
            time        TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
        );
    """)

    connection.commit()
