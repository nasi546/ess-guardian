#include "batteryrackwidget.h"
#include "ui_batteryrackwidget.h"
#include <QGridLayout>

BatteryRackWidget::BatteryRackWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::BatteryRackWidget)
{
    ui->setupUi(this);

    // QLabel 위에 overlay
    overlay = new QWidget(ui->pLabelRack);
    overlay->setStyleSheet("background: transparent;");

    setupOverlay();

    pBlinkTimer = new QTimer(this);
    connect(pBlinkTimer, SIGNAL(timeout()), this, SLOT(onBlinkTimeout()));
    pBlinkTimer->start(500);
}

BatteryRackWidget::~BatteryRackWidget()
{
    delete ui;
}

void BatteryRackWidget::setupOverlay()
{
    QGridLayout *grid = new QGridLayout(overlay);
    grid->setSpacing(4);
    grid->setContentsMargins(0,0,0,0);

    // 3 랙 × 3 층
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            QFrame *cell = new QFrame(overlay);
            cell->setStyleSheet(RACK_COLOR_NORMAL);

            rackCells.append(cell);
            grid->addWidget(cell, row, col);
        }
    }
}

void BatteryRackWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    overlay->setGeometry(ui->pLabelRack->rect());
}

void BatteryRackWidget::setRackState(int rackIndex, int levelIndex, RackState state)
{
    if (rackIndex < 1 || rackIndex > 3 || levelIndex < 1 || levelIndex > 3)
    {
        qDebug() << "[RACK INDEX ERROR]" << rackIndex << levelIndex;
        return;
    }

    // levelIndex: row (1~3), rackIndex: col (1~3)
    int index = (levelIndex - 1) * 3 + (rackIndex - 1);

    if (index < 0 || index >= rackCells.size())
        return;

    QFrame *cell = rackCells[index];

    switch (state)
    {
    case Normal:
        criticalRacks.remove(index);
        cell->setVisible(true);
        cell->setStyleSheet(RACK_COLOR_NORMAL);
        break;
    case Warning:
        criticalRacks.remove(index);
        cell->setVisible(true);
        cell->setStyleSheet(RACK_COLOR_WARNING);
        break;
    case Critical:
        criticalRacks.insert(index);
        cell->setStyleSheet(RACK_COLOR_CRITICAL);
        break;
    }
}

void BatteryRackWidget::onBlinkTimeout()
{
    blinkOn = !blinkOn;

    for (int index : criticalRacks)
    {
        if (index >= 0 && index < rackCells.size())
            rackCells[index]->setVisible(blinkOn);
    }
}
