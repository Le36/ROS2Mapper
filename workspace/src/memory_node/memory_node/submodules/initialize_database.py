from .database_connection import get_database_connection


def create_tables():
    """Initialize the database"""
    connection = get_database_connection()
    cursor = connection.cursor()

    tables = ["""
        CREATE TABLE IF NOT EXISTS robot
        (
            id   INTEGER PRIMARY KEY,
            data TEXT
        );
    """, """
        CREATE TABLE IF NOT EXISTS qr
        (
            content     TEXT UNIQUE PRIMARY KEY,
            coordinates TEXT
        );
    """, """
        CREATE TABLE IF NOT EXISTS history
        (
            content     TEXT,
            coordinates TEXT,
            time        TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL PRIMARY KEY
        );
    """]

    for x in tables:
        cursor.execute(x)
        connection.commit()
