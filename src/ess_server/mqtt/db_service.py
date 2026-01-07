import db
from db import get_connection
import mysql.connector
try:
	_db = get_connection()
	_cursor = _db.cursor(dictionary=True)
	_last_alert_fingerprint = None # 마지막 알람 저장 내용을 기억
	print("[DB] Connection established and shared cursor created.")
except Exception as e:
    print(f"[DB ERROR] Failed to connect to database: {e}")
    exit(1)

# ================================
# Environment / Alert 저장
# ================================
def save_environment(data):
    try:
        _cursor.execute("""
            INSERT INTO environment_data (temperature, humidity, fan, reason)
            VALUES (%s, %s, %s, %s)
        """, (data["temperature"], data["humidity"], data["fan"], data["reason"]))
        _db.commit()
        print("[DB] Environment data saved.")
    except Exception as e:
        _db.rollback()
        print("[DB ERROR] env save failed:", e)
# finally:
#       cursor.close()
#       db.close()

def save_alert(data):
#   db = get_connection()
#   cursor = db.cursor()
    global _last_alert_fingerprint
    
    # 중복 데이터 체크 (타입, 수치, 위치가 같으면 저장 안 함)
    current_fingerprint = f"{data.get('event_type')}_{data.get('value')}_{data.get('location')}"
    
    if current_fingerprint == _last_alert_fingerprint:
        print(f"[SKIP] Duplicate data ignored: {current_fingerprint}")
        return


    try:
        _cursor.execute("""
            INSERT INTO alert_events (event_type, level, value, location, message)
            VALUES (%s, %s, %s, %s, %s)
        """, (
            data["event_type"],
            data["level"],
            data["value"],
            data["location"],
            data["message"]
        ))
        _db.commit()
        print("[DB] Alert event saved.")
    except Exception as e:
        _db.rollback()
        print("[DB ERROR] alert save failed:", e)
#finally:
#       cursor.close()
#       db.close()

# ================================
# Admin / Access DB Functions
# ================================
def get_admin_by_id(admin_id: str):
    """
    admin_id(RFID)로 관리자 레코드 조회
    id(PK)와 access_points를 반환
    """
#   db = get_connection()
#   cursor = db.cursor(dictionary=True)
    try:
        _cursor.execute("""
            SELECT id, access_points 
            FROM admins 
            WHERE admin_id = %s
        """, (admin_id,))
        row = _cursor.fetchone()
        return row
    except Exception as e:
        print("[DB ERROR] admin lookup failed:", e)
        return None
# finally:
#       cursor.close()
#       db.close()

def log_access_result(admin_id: str, access_point: str, result: str):
# db = get_connection()
#   cursor = db.cursor()
    try:
        _cursor.execute("""
            INSERT INTO access_logs (admin_id, access_point, result)
            VALUES (%s, %s, %s)
        """, (admin_id, access_point, result))
        _db.commit()
        print(f"[DB] Access log saved: {admin_id} -> {access_point} ({result})")
    except Exception as e:
        _db.rollback()
        print("[DB ERROR] access log save failed:", e)
# finally:
#       cursor.close()
#       db.close()

def close_db():
	_cursor.close()
	_db.close()
