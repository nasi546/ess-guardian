#ifndef TAB3ALERT_H
#define TAB3ALERT_H

#include <QWidget>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDateTime>
#include <QChartView>
#include <QTableWidgetItem>
#include <QDebug>
#include <QHeaderView>

namespace Ui {
class Tab3Alert;
}

class Tab3Alert : public QWidget
{
    Q_OBJECT

public:
    explicit Tab3Alert(QWidget *parent = nullptr);
    ~Tab3Alert();

private slots:
    void on_pPBSearchDB_clicked();

private:
    Ui::Tab3Alert *ui;
};

#endif // TAB3ALERT_H
