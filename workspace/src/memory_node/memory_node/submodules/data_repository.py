from .database_connection import get_database_connection
from .initialize_database import create_tables


class DataRepository:
    def __init__(self) -> None:
        create_tables()

    def add_data(self, data: str) -> None:
        """Add the data to the database"""
        connection = get_database_connection()
        cursor = connection.cursor()
        sql = "INSERT INTO robot (data) VALUES (:data)"
        cursor.execute(sql, {"data": data}),
        connection.commit()

    def delete_all(self) -> None:
        """Delete all data from the database"""
        connection = get_database_connection()
        cursor = connection.cursor()
        cursor.execute("DELETE from robot")
        connection.commit()


data_repository = DataRepository()
