#ifndef BATTERYRACKWIDGET_H
#define BATTERYRACKWIDGET_H

#include <QWidget>
#include <QVector>
#include <QFrame>
#include <QTimer>
#include <QSet>

// Battery Rack Colors
#define RACK_COLOR_NORMAL   "background-color: rgba(0, 180, 0, 120);"
#define RACK_COLOR_WARNING  "background-color: rgba(255, 165, 0, 160);"
#define RACK_COLOR_CRITICAL "background-color: rgba(255, 0, 0, 180);"

namespace Ui {
class BatteryRackWidget;
}

class BatteryRackWidget : public QWidget
{
    Q_OBJECT

public:
    explicit BatteryRackWidget(QWidget *parent = nullptr);
    ~BatteryRackWidget();

    enum RackState {
        Normal,
        Warning,
        Critical
    };

    // rackIndex: 0~2, levelIndex: 0~2
    void setRackState(int rackIndex, int levelIndex, RackState state);

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    Ui::BatteryRackWidget *ui;

    QWidget *overlay;
    QVector<QFrame*> rackCells;   // 총 9칸 (3×3)
    QTimer *pBlinkTimer;
    bool blinkOn;
    QSet<int> criticalRacks;    // Critical 상태인 index 저장

    void setupOverlay();

private slots:
    void onBlinkTimeout();
};

#endif // BATTERYRACKWIDGET_H
