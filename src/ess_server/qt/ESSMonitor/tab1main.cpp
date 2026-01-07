#include "tab1main.h"
#include "ui_tab1main.h"
#include "essmapwidget.h"
#include "batteryrackwidget.h"
#include <QTimer>
#include <QDateTime>
#include <QSqlQuery>
#include <QDebug>
#include <QSqlError>
#include <QSet>
#include <QMovie>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

Tab1Main::Tab1Main(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab1Main)
{
    ui->setupUi(this);

    pMapWidget = new ESSMapWidget(this);
    pRackWidget = new BatteryRackWidget(this);

    ui->pMapLayout->layout()->addWidget(pMapWidget);
    ui->pBatteryrackLayout->layout()->addWidget(pRackWidget);

    QTimer *timeTimer = new QTimer(this);
    connect(timeTimer, SIGNAL(timeout()), this, SLOT(updateTime()));
    timeTimer->start(1000);

    QTimer *envTimer = new QTimer(this);
    connect(envTimer, SIGNAL(timeout()), this, SLOT(updateEnvironment()));
    envTimer->start(5000);

    QTimer *mapTimer = new QTimer(this);
    connect(mapTimer, SIGNAL(timeout()), this, SLOT(updateESSMap()));
    mapTimer->start(2000);

    QTimer *rackTimer = new QTimer(this);
    connect(rackTimer, SIGNAL(timeout()), this, SLOT(updateBatteryRack()));
    rackTimer->start(2000);

    fanMovie = new QMovie(":/images/img/fan_on.gif");
    fanMovie->setScaledSize(QSize(64, 64));
    ui->pLFanimg->setMovie(fanMovie);
    fanMovie->jumpToFrame(0);

    // mqtt 초기화
    mosquitto_lib_init();
    mosq = mosquitto_new("ess_gui_client", true, NULL);

    // 브로커 연결
    if(mosquitto_connect(mosq, "10.10.14.109", 1883, 60) != MOSQ_ERR_SUCCESS){
        qDebug() << "MQTT Broker connection failed!";
    }
}

Tab1Main::~Tab1Main()
{
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    delete ui;
}

void Tab1Main::updateTime()
{
    QString now = QDateTime::currentDateTime().toString("yyyy.MM.dd hh:mm:ss");
    ui->pLabelTime->setText(now);
}

void Tab1Main::updateEnvironment()
{
    QSqlQuery query;

    if(!query.exec(
            "SELECT temperature, humidity, fan "
            "FROM environment_data "
            "ORDER BY measure_time DESC LIMIT 1"
            ))
    {
        qDebug() << "[ENV QUERY ERROR]" << query.lastError().text();
        return;
    }

    if(query.next())
    {
        qDebug() << "[ENV DATA]"
                 << query.value(0).toDouble()
                 << query.value(1).toDouble()
                 << query.value(2).toString().toUpper();

        ui->pLabelTemp->setText(
            QString::number(query.value(0).toDouble(), 'f', 1) + " °C"
            );
        ui->pLabelHumi->setText(
            QString::number(query.value(1).toDouble(), 'f', 1) + " %"
            );

        QString fanStatus = query.value(2).toString().toUpper();

        if (fanStatus == "ON"){
            if(fanMovie->state() != QMovie::Running) {
                fanMovie->start();
            }
            ui->pPBFanOnOff->setChecked(true);
            ui->pPBFanOnOff->setText("OFF");
        }
        else {
            fanMovie->stop();
            ui->pPBFanOnOff->setChecked(false);
            ui->pPBFanOnOff->setText("ON");
        }
    }
    else
    {
        qDebug() << "[ENV] no row";
    }
}

// ESS 맵 업데이트
void Tab1Main::updateESSMap()
{
    QDateTime now = QDateTime::currentDateTime();
    QSqlQuery query;
    query.exec(
        "SELECT location, level, alert_time "
        "FROM alert_events "
        "WHERE event_type='gas' "
        // 현재 시간에서 10초를 뺀 시간보다 최신인 데이터만 조회
        "AND alert_time > DATE_SUB(NOW(), INTERVAL 10 SECOND) "
        "ORDER BY alert_time DESC LIMIT 6"
        );

    // 현재 알람이 '살아있는' 구역들을 저장 (1~6번 구역)
    QSet<int> currentlyAlertedZones;
    QString maxLevel = "";

    // 새 알림 들어온 구역 업데이트
    while(query.next())
    {
        QString location = query.value(0).toString();
        QString level = query.value(1).toString();
        QDateTime alertTime = query.value(2).toDateTime();
        QStringList parts = location.split("_");
        if(parts.size() < 2) continue;

        QString type = parts.first().toLower();
        int zone = parts.last().toInt();
        if (type == "zone"){
            currentlyAlertedZones.insert(zone);
            int zoneIndex = zone - 1;

            // 위젯 색상 변경 (Warning 또는 Critical)
            pMapWidget->setZoneState(zoneIndex, (level == "warning") ? ESSMapWidget::Warning : ESSMapWidget::Critical);

            // 전체 위험 수위 체크 (텍스트 표시용)
            if (level == "critical") maxLevel = "critical";
            else if (maxLevel != "critical") maxLevel = "warning";
        }
    }

    // 1번부터 6번 구역까지 돌면서, 이번 쿼리 결과에 없는 구역은 무조건 Normal로 처리!!!!
    for (int i = 1; i <= 6; ++i) {
        if (!currentlyAlertedZones.contains(i)) {
            int zoneIndex = i - 1;
            pMapWidget->setZoneState(zoneIndex, ESSMapWidget::Normal);
        }
    }

    // 우측 하단 가스 알람 텍스트 업데이트
    if (!currentlyAlertedZones.isEmpty()) {
        ui->pLabelGas->setText(maxLevel.toUpper());
        ui->pLabelGas->setStyleSheet(maxLevel == "warning" ? "color: orange;" : "color: red; font-weight: bold;");
    } else {
        ui->pLabelGas->setText("-");
        ui->pLabelGas->setStyleSheet("color: black;");
    }
}

// 배터리 랙 업데이트
void Tab1Main::updateBatteryRack()
{
    // 최근 10초 이내의 Thermal 알람만 조회
    QSqlQuery query;
    if (!query.exec(
            "SELECT location, level, alert_time "
            "FROM alert_events "
            "WHERE event_type='thermal' "
            "AND alert_time > DATE_SUB(NOW(), INTERVAL 10 SECOND) "
            "ORDER BY alert_time DESC LIMIT 9"
            ))
    {
        qDebug() << "[RACK QUERY ERROR]" << query.lastError().text();
        return;
    }

    // 현재 알람이 활성화된 랙 위치를 저장할 Set (예: "1_2" 형태의 문자열로 저장)
    QSet<QString> activeRacks;
    QString maxLevel = "";
    const int totalLayers = 3; // 랙당 층수

    while(query.next())
    {
        QString location = query.value(0).toString(); // rack_2_3
        QString level = query.value(1).toString();

        // 10초 이내 데이터가 있다면 Set에 추가
        activeRacks.insert(location);

        // 위치 파싱 (rack_1_2 -> r=1, l=2)
        QStringList parts = location.split("_");
        if(parts.size() == 3) {
            int r = parts[1].toInt();   // 랙 번호
            int l = parts[2].toInt();   // 층 번호

            // 층 역순 매핑: 맨 위가 1, 맨 아래가 totalLayers
            int displayLayer = totalLayers - l + 1;

            // 위젯 색상 변경
            pRackWidget->setRackState(r, displayLayer, (level == "warning") ? BatteryRackWidget::Warning : BatteryRackWidget::Critical);
        }

        // 전체 위험 수위 계산
        if (level == "critical") maxLevel = "critical";
        else if (maxLevel != "critical") maxLevel = "warning";
    }

    // 전체 랙을 순회하며 DB 결과에 없는 랙은 모두 Normal로 초기화
    for (int r = 1; r <= 3; ++r) {
        for (int l = 1; l <= 3; ++l) {
            QString currentPos = QString("rack_%1_%2").arg(r).arg(l);

            // 이번 쿼리 결과(10초 이내)에 포함되지 않은 랙이라면?
            if (!activeRacks.contains(currentPos)) {
                int displayLayer = totalLayers - l + 1;
                pRackWidget->setRackState(r, displayLayer, BatteryRackWidget::Normal);
            }
        }
    }

    // 우측 하단 발열(Thermal) 알람 텍스트 업데이트
    if (!activeRacks.isEmpty()) {
        ui->pLabelThermal->setText(maxLevel.toUpper());
        ui->pLabelThermal->setStyleSheet(maxLevel == "warning" ? "color: orange;" : "color: red; font-weight: bold;");
    }
    else {
        ui->pLabelThermal->setText("-");
        ui->pLabelThermal->setStyleSheet("color: black;");
    }
}

void Tab1Main::on_pPBFanOnOff_clicked(bool checked)
{
    // JSON 객체 생성
    QJsonObject jsonObj;
    jsonObj["fan"] = checked ? "ON" : "OFF";
    jsonObj["reason"] = "MANUAL"; // 수동 조작임을 명시

    // JSON을 문자열(QByteArray)로 변환
    QJsonDocument doc(jsonObj);
    QByteArray payload = doc.toJson(QJsonDocument::Compact); // 한 줄로 압축된 JSON

    const char* topic = "ess/fan/control";

    int rc = mosquitto_publish(mosq, NULL, topic, payload.length(), payload.data(), 0, false);

    if(rc == MOSQ_ERR_SUCCESS) {
        if (checked) {
            fanMovie->start();
            ui->pPBFanOnOff->setText("OFF");
        }
        else {
            fanMovie->stop();
            ui->pPBFanOnOff->setText("ON");
        }
        qDebug() << "Published:" << payload;
    }
    else {
        qDebug() << "MQTT 서버에 연결되어 있지 않습니다.";
    }
}
