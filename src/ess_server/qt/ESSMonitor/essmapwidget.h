#ifndef ESSMAPWIDGET_H
#define ESSMAPWIDGET_H

#pragma once
#include <QWidget>
#include <QPixmap>
#include <QFrame>
#include <QVector>
#include <QTimer>
#include <QSet>

#define MAP_COLOR_NORMAL   "background-color: rgba(0,255,0,50);"
#define MAP_COLOR_WARNING  "background-color: rgba(255,215,0,170);"
#define MAP_COLOR_CRITICAL "background-color: rgba(255,0,0,190);"

namespace Ui {
class ESSMapWidget;
}

class ESSMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ESSMapWidget(QWidget *parent = nullptr);

    enum ZoneState {
        Normal,
        Warning,
        Critical
    };

    void setZoneState(int zoneIndex, ZoneState state);
    ~ESSMapWidget();

private:
    Ui::ESSMapWidget *ui;
    QWidget *overlay;
    QVector<QFrame*> zones;
    QPixmap mapPixmap;
    QTimer *pblinkTimer;
    bool blinkOn;
    QSet<int> criticalZones;

    void setupOverlay();
    void updateMapSize();

private slots:
    void onBlinkTimeout();

protected:
    void resizeEvent(QResizeEvent *event) override;
};

#endif // ESSMAPWIDGET_H
