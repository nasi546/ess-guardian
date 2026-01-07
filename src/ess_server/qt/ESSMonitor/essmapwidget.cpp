#include "essmapwidget.h"
#include "ui_essmapwidget.h"
#include <QLabel>
#include <QGridLayout>

ESSMapWidget::ESSMapWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ESSMapWidget)
    , blinkOn(true)
{
    ui->setupUi(this);

    ui->pLabelMap->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->pLabelMap->setAlignment(Qt::AlignCenter);

    mapPixmap.load(":/images/img/ess_map.png");

    overlay = new QWidget(ui->pLabelMap);
    overlay->setStyleSheet("background: transparent;");

    setupOverlay();

    pblinkTimer = new QTimer(this);
    connect(pblinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimeout()));
    pblinkTimer->start(500);

    // auto *layout = new QVBoxLayout(this);
    // layout->setContentsMargins(0,0,0,0);
    // layout->addWidget(ui->pLabelMap);
}

ESSMapWidget::~ESSMapWidget()
{
    delete ui;
}

void ESSMapWidget::setupOverlay()
{
    QGridLayout *grid = new QGridLayout(overlay);
    grid->setSpacing(2);
    grid->setContentsMargins(0,0,0,0);

    for (int i = 0; i < 6; ++i)
    {
        QFrame *zone = new QFrame(overlay);
        zone->setStyleSheet(MAP_COLOR_NORMAL);
        zones.append(zone);
        grid->addWidget(zone, i / 3, i % 3);
    }
}

void ESSMapWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    updateMapSize();
}

void ESSMapWidget::updateMapSize()
{
    // 위젯의 실제 크기가 결정되지 않았으면 리턴
    if (this->width() <= 0 || mapPixmap.isNull()) return;

    // 1. 라벨 자체의 크기를 위젯 전체 크기로 맞춤
    // ui->pLabelMap->setGeometry(this->rect());

    // 2. 이미지를 라벨 크기에 맞춰 스케일링 (비율 유지)
    QPixmap scaled = mapPixmap.scaled(ui->pLabelMap->size(),
                                      Qt::KeepAspectRatio,
                                      Qt::SmoothTransformation);
    ui->pLabelMap->setPixmap(scaled);

    // 3. 실제 이미지가 그려진 영역(중앙 정렬 기준) 계산
    int pw = scaled.width();
    int ph = scaled.height();
    int lx = (ui->pLabelMap->width() - pw) / 2;
    int ly = (ui->pLabelMap->height() - ph) / 2;

    // 4. 오버레이를 이미지 위치에 딱 맞게 배치
    overlay->setGeometry(lx, ly, pw, ph);
}

void ESSMapWidget::setZoneState(int zoneIndex, ZoneState state)
{
    if(zoneIndex < 0 || zoneIndex >= zones.size())
        return;

    QFrame *zone = zones[zoneIndex];

    switch(state)
    {
    case Normal:
        criticalZones.remove(zoneIndex);
        zone->setVisible(true);
        zone->setStyleSheet(MAP_COLOR_NORMAL);
        break;

    case Warning:
        criticalZones.remove(zoneIndex);
        zone->setVisible(true);
        zone->setStyleSheet(MAP_COLOR_WARNING);
        break;

    case Critical:
        criticalZones.insert(zoneIndex);
        zone->setStyleSheet(MAP_COLOR_CRITICAL);
        break;
    }
}

void ESSMapWidget::onBlinkTimeout()
{
    blinkOn = !blinkOn;

    for (int zone : criticalZones)
    {
        if (zone >= 0 && zone < zones.size())
            zones[zone]->setVisible(blinkOn);
    }
}
