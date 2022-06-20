from .database_connection import get_database_connection


def initialize_database():
    """Initialize the database"""
    connection = get_database_connection()
    cursor = connection.cursor()

    cursor.execute("DROP TABLE IF EXISTS qr_codes")
    cursor.execute("DROP TABLE IF EXISTS history")

    tables = [
        """
        CREATE TABLE IF NOT EXISTS qr_codes
        (
            id              INT   PRIMARY KEY,
            center_x        FLOAT NOT NULL,
            center_y        FLOAT NOT NULL,
            center_z        FLOAT NOT NULL,
            normal_vector_x FLOAT NOT NULL,
            normal_vector_y FLOAT NOT NULL,
            normal_vector_z FLOAT NOT NULL,
            rotation_w      FLOAT NOT NULL,
            rotation_x      FLOAT NOT NULL,
            rotation_y      FLOAT NOT NULL,
            rotation_z      FLOAT NOT NULL
        );
        """,
        """
        CREATE TABLE IF NOT EXISTS history
        (
            pk              INT   PRIMARY KEY,
            id              INT   NOT NULL,
            center_x        FLOAT NOT NULL,
            center_y        FLOAT NOT NULL,
            center_z        FLOAT NOT NULL,
            normal_vector_x FLOAT NOT NULL,
            normal_vector_y FLOAT NOT NULL,
            normal_vector_z FLOAT NOT NULL,
            rotation_w      FLOAT NOT NULL,
            rotation_x      FLOAT NOT NULL,
            rotation_y      FLOAT NOT NULL,
            rotation_z      FLOAT NOT NULL,
            time            TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL 
        );
        """,
    ]

    for table in tables:
        cursor.execute(table)

    connection.commit()
