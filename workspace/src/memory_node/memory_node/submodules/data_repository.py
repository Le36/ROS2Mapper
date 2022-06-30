from typing import Dict, List

import numpy as np
from interfaces.msg import QRCode

from .database_connection import get_database_connection
from .initialize_database import initialize_database


class DataRepository:
    def __init__(self) -> None:
        initialize_database()

    @staticmethod
    def qr_code_to_dict(qr_code: QRCode) -> Dict:
        """Convert a QRCode into a dictionary to be used in SQLite commands"""
        return {
            "id": qr_code.id,
            "center_x": qr_code.center[0],
            "center_y": qr_code.center[1],
            "center_z": qr_code.center[2],
            "normal_vector_x": qr_code.normal_vector[0],
            "normal_vector_y": qr_code.normal_vector[1],
            "normal_vector_z": qr_code.normal_vector[2],
            "rotation_w": qr_code.rotation[0],
            "rotation_x": qr_code.rotation[1],
            "rotation_y": qr_code.rotation[2],
            "rotation_z": qr_code.rotation[3],
        }

    @staticmethod
    def row_to_qr_code(row: List) -> QRCode:
        """Convert an SQLite row into a QRCode"""
        return QRCode(
            id=row[0],
            center=np.array(row[1:4]),
            normal_vector=np.array(row[4:7]),
            rotation=np.array(row[7:11]),
        )

    def add_qr_code_to_history(self, qr_code: QRCode) -> None:
        """Add a QR code to the history"""
        connection = get_database_connection()
        cursor = connection.cursor()
        sql = """
            INSERT INTO history (
                id,
                center_x, center_y, center_z,
                normal_vector_x, normal_vector_y, normal_vector_z,
                rotation_w, rotation_x, rotation_y, rotation_z
            )
            VALUES (
                :id,
                :center_x, :center_y, :center_z,
                :normal_vector_x, :normal_vector_y, :normal_vector_z,
                :rotation_w, :rotation_x, :rotation_y, :rotation_z
            )
        """
        cursor.execute(sql, self.qr_code_to_dict(qr_code)),
        connection.commit()
        connection.close()

    def add_qr_code(self, qr_code: QRCode) -> None:
        """Add the QR code to the database and if it exists replace it"""
        connection = get_database_connection()
        cursor = connection.cursor()

        sql = "DELETE FROM qr_codes WHERE id=:id"
        cursor.execute(sql, {"id": qr_code.id}),

        sql = """
            INSERT INTO qr_codes (
                id,
                center_x, center_y, center_z,
                normal_vector_x, normal_vector_y, normal_vector_z,
                rotation_w, rotation_x, rotation_y, rotation_z
            )
            VALUES (
                :id,
                :center_x, :center_y, :center_z,
                :normal_vector_x, :normal_vector_y, :normal_vector_z,
                :rotation_w, :rotation_x, :rotation_y, :rotation_z
            )
        """
        cursor.execute(sql, self.qr_code_to_dict(qr_code))

        connection.commit()
        connection.close()

    def get_qr_codes(self) -> List[QRCode]:
        """Get the list of QR codes"""
        connection = get_database_connection()
        cursor = connection.cursor()
        sql = "SELECT * FROM qr_codes"
        rows = cursor.execute(sql).fetchall()
        connection.close()
        return [self.row_to_qr_code(row) for row in rows]

    def delete_all(self) -> None:
        """Delete all data from the database"""
        connection = get_database_connection()
        cursor = connection.cursor()
        cursor.execute("DELETE FROM qr_codes")
        cursor.execute("DELETE FROM history")
        connection.commit()
        connection.close()


data_repository = DataRepository()
