import sqlite3


def add_data(data):
    """Add the data to the database"""
    conn = sqlite3.connect("test.db")
    sql = "INSERT INTO robot (data) VALUES (:data)"
    conn.cursor().execute(sql, {"data": data}),
    conn.commit()
    conn.close()
