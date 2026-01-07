import mysql.connector

def get_connection():
	return mysql.connector.connect(
			host="10.10.14.109",
			user="ess",
			password="ess1234",
			database="ess_db"
			)
