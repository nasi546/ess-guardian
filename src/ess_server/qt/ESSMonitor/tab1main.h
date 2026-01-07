#ifndef TAB1MAIN_H
#define TAB1MAIN_H

#include <QWidget>
#include "essmapwidget.h"
#include "batteryrackwidget.h"
#include <QDateTime>
#include <mosquitto.h>

namespace Ui {
class Tab1Main;
}

class Tab1Main : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1Main(QWidget *parent = nullptr);
    ~Tab1Main();

private:
    Ui::Tab1Main *ui;
    ESSMapWidget *pMapWidget;
    BatteryRackWidget *pRackWidget;

    QMap<int, QDateTime> zoneAlertTimes;                   // ESS 맵 구역별 알림 시간
    QMap<int, QString> zoneAlertLevels;                   // 구역별 warning/critical 상태 저장

    QMap<QPair<int,int>, QDateTime> rackAlertTimes;       // 배터리 랙 구역별 알림 시간
    QMap<QPair<int,int>, QString> rackAlertLevels;        // 랙별 warning/critical 상태 저장

    QMovie *fanMovie;
    struct mosquitto *mosq = nullptr;

private slots:
    void updateTime();
    void updateEnvironment();
    // void updateAlert();          // 기능을 각각 updateESSMap()과 updateBatteryRack()에 나눠서 통합
    void updateESSMap();
    void updateBatteryRack();
    void on_pPBFanOnOff_clicked(bool checked);
};

#endif // TAB1MAIN_H
