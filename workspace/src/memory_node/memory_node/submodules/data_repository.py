from .database_connection import get_database_connection
from .initialize_database import create_tables


class DataRepository:

    def __init__(self) -> None:
        create_tables()
        self.connection = get_database_connection()

    def add_data(self, data: str) -> None:
        """Add the data to the database"""
        cursor = self.connection.cursor()
        sql = 'INSERT INTO robot (data) VALUES (:data)'
        cursor.execute(sql, {'data': data}),
        self.connection.commit()

    def read_data(self, data) -> None:
        """Read data from the database"""
        cursor = self.connection.cursor()
        sql = 'SELECT data FROM robot WHERE data=:data'
        cursor.execute(sql, {"data":data})
        result = cursor.fetchone()
        return result[0]

    def drop_table(self) -> None:
        """Drop current table"""
        cursor = self.connection.cursor()
        sql = 'DROP TABLE IF EXISTS robot'
        cursor.execute(sql)
        self.connection.commit()


data_repository = DataRepository()
