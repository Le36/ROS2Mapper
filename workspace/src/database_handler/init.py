import sqlite3


def init():
    conn = sqlite3.connect("test.db")
    conn.execute("CREATE TABLE IF NOT EXISTS robot (id INTEGER PRIMARY KEY, data TEXT)")
    conn.close()


if __name__ == '__main__':
    init()

