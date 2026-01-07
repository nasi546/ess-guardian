-- 비상 알림 테이블 (가스 / 열 이상 이벤트)
CREATE TABLE alert_events (
	id INT AUTO_INCREMENT PRIMARY KEY,
	alert_time DATETIME DEFAULT CURRENT_TIMESTAMP,
	event_type ENUM('gas', 'thermal') NOT NULL,
	level ENUM('warning', 'critical') DEFAULT 'warning',
	value FLOAT,
	location VARCHAR(50),
	message VARCHAR(255)
);

-- 온습도 센서 데이터 테이블
CREATE TABLE environment_data (
	id INT AUTO_INCREMENT PRIMARY KEY,
	measure_time DATETIME DEFAULT CURRENT_TIMESTAMP,
	temperature FLOAT,
	humidity FLOAT
	fan VARCHAR(10) DEFAULT 'OFF',   -- 공조기 상태 (ON/OFF)
	reason VARCHAR(50) DEFAULT '-',  -- 구동 사유 (TEMP/HUMI/MANUAL/-)
);

-- 사전 등록된 출입 승인자 정보 테이블
CREATE TABLE admins (
    id INT AUTO_INCREMENT PRIMARY KEY,
    admin_id VARCHAR(50) UNIQUE NOT NULL,        -- RFID 태그 고유번호
    access_points VARCHAR(255) NOT NULL          -- 출입 가능한 지점, 콤마 구분
);

-- 출입 인증 시도 기록 테이블
CREATE TABLE access_logs (
	id INT AUTO_INCREMENT PRIMARY KEY,
	admin_id VARCHAR(50),
	access_point VARCHAR(50),
	result ENUM('success', 'fail') NOT NULL,
	access_time DATETIME DEFAULT CURRENT_TIMESTAMP
);
