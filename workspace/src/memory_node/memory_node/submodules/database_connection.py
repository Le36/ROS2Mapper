import sqlite3

connection = sqlite3.connect("test.db")
connection.row_factory = sqlite3.Row


def get_database_connection() -> sqlite3.Connection:
    """Returns the database connection"""
    return connection
