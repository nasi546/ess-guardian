#ifndef TAB4ACCESS_H
#define TAB4ACCESS_H

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
class Tab4Access;
}

class Tab4Access : public QWidget
{
    Q_OBJECT

public:
    explicit Tab4Access(QWidget *parent = nullptr);
    ~Tab4Access();

private slots:
    void on_pPBSearchDB_clicked();

private:
    Ui::Tab4Access *ui;
};

#endif // TAB4ACCESS_H
