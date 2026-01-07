#ifndef TAB2ENV_H
#define TAB2ENV_H

#include <QWidget>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDateTime>
#include <QChartView>
#include <QLineSeries>
#include <QDateTimeAxis>
#include <QValueAxis>
#include <QTableWidgetItem>
#include <QDebug>
#include <QHeaderView>
#if QT_VERSION < QT_VERSION_CHECK(6,0,0)
QT_CHARTS_USE_NAMESPACE
#endif

namespace Ui {
class Tab2Env;
}

class Tab2Env : public QWidget
{
    Q_OBJECT

public:
    explicit Tab2Env(QWidget *parent = nullptr);
    ~Tab2Env();

private slots:
    void on_pPBSearchDB_clicked();
    void on_pPBDeleteDB_clicked();

private:
    Ui::Tab2Env *ui;
    QSqlDatabase qSqlDatabase;
    QLineSeries *tempLine;
    QLineSeries *humiLine;
    QChart *pQChart;
    QChartView *pQChartView;
    QDateTimeAxis *pQDateTimeAxisX;
    QDateTime firstDateTime;
    QDateTime lastDateTime;


    QTableWidgetItem* pQTableWidgetItemId = nullptr;
    QTableWidgetItem* pQTableWidgetItemDate = nullptr;
    QTableWidgetItem* pQTableWidgetItemTemp = nullptr;
    QTableWidgetItem* pQTableWidgetItemHumi = nullptr;

    void updateLastDateTime(bool);
    void updateLastDateTimeSql(bool);

    bool openDatabase();
    void loadEnvironmentData();
};

#endif // TAB2ENV_H
