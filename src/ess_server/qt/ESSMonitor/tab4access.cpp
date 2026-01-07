#include "tab4access.h"
#include "ui_tab4access.h"

Tab4Access::Tab4Access(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab4Access)
{
    ui->setupUi(this);

    ui->pAccessTable->verticalHeader()->setVisible(false); // Qt table 기본 인덱스 삭제

    if (!QSqlDatabase::database().isOpen()) {
        qDebug() << "[Tab4Access] DB not open!";
        ui->pPBSearchDB->setEnabled(false);
        return;
    }
}

Tab4Access::~Tab4Access()
{
    delete ui;
}

void Tab4Access::on_pPBSearchDB_clicked()
{
    // 테이블 초기화
    ui->pAccessTable->clearContents();
    ui->pAccessTable->setRowCount(0);

    QDateTime fromDateTime = ui->pDateTimeFrom->dateTime();
    QDateTime toDateTime = ui->pDateTimeTo->dateTime();

    QString strFromDateTime = fromDateTime.toString("yyyy-MM-dd HH:mm:ss");
    QString strToDateTime = toDateTime.toString("yyyy-MM-dd HH:mm:ss");

    // combobox 옵션 저장
    QString selectedAccessPoint = ui->pCBAccessPoints->currentText().toLower();

    QString strQuery =
        "SELECT id, admin_id, access_point, result, access_time "
        "FROM access_logs "
        "WHERE access_time BETWEEN '" + strFromDateTime +
                       "' AND '" + strToDateTime + "' ";

    if (selectedAccessPoint != "all") {
        if(selectedAccessPoint == "main") {
            strQuery += " AND access_point = 'main' ";
        }
        else if(selectedAccessPoint == "ew1"){
            strQuery += " AND access_point = 'ew1' ";
        }
        else if(selectedAccessPoint == "ew2"){
            strQuery += " AND access_point = 'ew2' ";
        }
        else if(selectedAccessPoint == "ww1"){
            strQuery += " AND access_point = 'ww1' ";
        }
        else if(selectedAccessPoint == "sw1"){
            strQuery += " AND access_point = 'sw1' ";
        }
        else if(selectedAccessPoint == "sw2"){
            strQuery += " AND access_point = 'sw2' ";
        }
        else {
            strQuery += " AND access_point = 'nw1' ";
        }
    }

    strQuery += " ORDER BY access_time ASC";

    QSqlQuery sqlQuery;
    if (!sqlQuery.exec(strQuery)) {
        qDebug() << "[ERROR] Query failed:" << sqlQuery.lastError();
        return;
    }

    int rowCount = 0;
    while (sqlQuery.next())
    {
        ui->pAccessTable->insertRow(rowCount);

        int db_id = sqlQuery.value("id").toInt();
        QString formattedID = QString("ACC-%1").arg(db_id, 3, 10, QChar('0'));

        QString adminIdStr = sqlQuery.value("admin_id").toString();
        QString accessPointStr = sqlQuery.value("access_point").toString();
        QString resultStr = sqlQuery.value("result").toString();
        QDateTime xValue = sqlQuery.value("access_time").toDateTime();
        xValue.setTimeSpec(Qt::LocalTime);

        // 테이블에 값 추가
        QTableWidgetItem *idItem = new QTableWidgetItem(formattedID);
        idItem->setTextAlignment(Qt::AlignCenter);
        ui->pAccessTable->setItem(rowCount, 0, idItem);

        QTableWidgetItem *adminIdItem = new QTableWidgetItem(adminIdStr);
        adminIdItem->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        ui->pAccessTable->setItem(rowCount, 1, adminIdItem);

        QTableWidgetItem *accessPointItem = new QTableWidgetItem(accessPointStr);
        accessPointItem->setTextAlignment(Qt::AlignCenter);
        ui->pAccessTable->setItem(rowCount, 2, accessPointItem);

        QTableWidgetItem *resultItem = new QTableWidgetItem(resultStr);
        resultItem->setTextAlignment(Qt::AlignCenter);
        ui->pAccessTable->setItem(rowCount, 3, resultItem);

        QTableWidgetItem *timeItem = new QTableWidgetItem(xValue.toString("yyyy-MM-dd HH:mm:ss"));
        timeItem->setTextAlignment(Qt::AlignCenter);
        ui->pAccessTable->setItem(rowCount, 4, timeItem);

        if(resultStr == "fail") {
            QTableWidgetItem *currentItem = ui->pAccessTable->item(rowCount, 3);
            if(currentItem) {
                currentItem->setForeground(Qt::red);
            }
        }
        else {
            QTableWidgetItem *currentItem = ui->pAccessTable->item(rowCount, 3);
            if(currentItem) {
                currentItem->setForeground(Qt::darkGreen);
            }
        }

        rowCount++;
    }
    ui->pLSearchCount->setText(QString("검색 결과:    %1건").arg(rowCount));

    // 컬럼 너비 자동 조정
    for (int col = 0; col < 5; ++col) {
        ui->pAccessTable->resizeColumnToContents(col);
    }
}

