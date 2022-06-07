import os
import sqlite3

file_dir = os.path.dirname(__file__)
DATABASE_FILENAME = os.getenv("DATABASE_FILENAME") or "database.sqlite"
DATA_DIR_PATH = os.path.join(file_dir, "..", "..", "data")

if not os.path.exists(DATA_DIR_PATH):  # pragma: no cover
    os.mkdir(DATA_DIR_PATH)

DATABASE_PATH = os.path.join(DATA_DIR_PATH, DATABASE_FILENAME)


def get_database_connection() -> sqlite3.Connection:
    """Returns the database connection"""
    connection = sqlite3.connect(DATABASE_PATH)
    connection.row_factory = sqlite3.Row
    return connection
