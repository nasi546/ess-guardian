#include "tab3alert.h"
#include "ui_tab3alert.h"

Tab3Alert::Tab3Alert(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab3Alert)
{
    ui->setupUi(this);

    ui->pAlertTable->verticalHeader()->setVisible(false); // Qt table 기본 인덱스 삭제

    if (!QSqlDatabase::database().isOpen()) {
        qDebug() << "[Tab3Alert] DB not open!";
        ui->pPBSearchDB->setEnabled(false);
        return;
    }
}

Tab3Alert::~Tab3Alert()
{
    delete ui;
}

void Tab3Alert::on_pPBSearchDB_clicked()
{
    // 테이블 초기화
    ui->pAlertTable->clearContents();
    ui->pAlertTable->setRowCount(0);

    QDateTime fromDateTime = ui->pDateTimeFrom->dateTime();
    QDateTime toDateTime = ui->pDateTimeTo->dateTime();

    QString strFromDateTime = fromDateTime.toString("yyyy-MM-dd HH:mm:ss");
    QString strToDateTime = toDateTime.toString("yyyy-MM-dd HH:mm:ss");

    // combobox 옵션 저장
    QString selectedEventType = ui->pCBEventType->currentText().toLower();
    QString selectedLevel = ui->pCBLevel->currentText().toLower();

    QString strQuery =
        "SELECT id, alert_time, event_type, level, value, location, message "
        "FROM alert_events "
        "WHERE alert_time BETWEEN '" + strFromDateTime +
                       "' AND '" + strToDateTime + "' ";

    if (selectedEventType != "all") {
        if(selectedEventType == "gas") {
            strQuery += " AND event_type = 'gas' ";
        }
        else {
            strQuery += " AND event_type = 'thermal' ";
        }
    }

    if (selectedLevel != "all") {
        if(selectedLevel == "warning") {
            strQuery += " AND level = 'warning' ";
        }
        else {
            strQuery += " AND level = 'critical' ";
        }
    }
    strQuery += " ORDER BY alert_time ASC";

    QSqlQuery sqlQuery;
    if (!sqlQuery.exec(strQuery)) {
        qDebug() << "[ERROR] Query failed:" << sqlQuery.lastError();
        return;
    }

    int rowCount = 0;
    while (sqlQuery.next())
    {
        ui->pAlertTable->insertRow(rowCount);

        int db_id = sqlQuery.value("id").toInt();
        QString formattedID = QString("ALT-%1").arg(db_id, 3, 10, QChar('0'));

        // QString timeStr = sqlQuery.value("alert_time").toString();
        QDateTime xValue = sqlQuery.value("alert_time").toDateTime();
        xValue.setTimeSpec(Qt::LocalTime);

        QString eventTypeStr = sqlQuery.value("event_type").toString();
        QString levelStr = sqlQuery.value("level").toString();
        float db_value = sqlQuery.value("value").toFloat();
        QString locationStr = sqlQuery.value("location").toString();
        QString messageStr = sqlQuery.value("message").toString();

        // 테이블에 값 추가
        QTableWidgetItem *idItem = new QTableWidgetItem(formattedID);
        idItem->setTextAlignment(Qt::AlignCenter);
        ui->pAlertTable->setItem(rowCount, 0, idItem);

        QTableWidgetItem *timeItem = new QTableWidgetItem(xValue.toString("yyyy-MM-dd HH:mm:ss"));
        timeItem->setTextAlignment(Qt::AlignCenter);
        ui->pAlertTable->setItem(rowCount, 1, timeItem);

        QTableWidgetItem *eventTypeItem = new QTableWidgetItem(eventTypeStr);
        eventTypeItem->setTextAlignment(Qt::AlignCenter);
        ui->pAlertTable->setItem(rowCount, 2, eventTypeItem);

        QTableWidgetItem *levelItem = new QTableWidgetItem(levelStr);
        levelItem->setTextAlignment(Qt::AlignCenter);
        ui->pAlertTable->setItem(rowCount, 3, levelItem);

        QTableWidgetItem *valueItem = new QTableWidgetItem(QString::number(db_value, 'f', 2));
        valueItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        ui->pAlertTable->setItem(rowCount, 4, valueItem);

        QTableWidgetItem *locationItem = new QTableWidgetItem(locationStr);
        locationItem->setTextAlignment(Qt::AlignCenter);
        ui->pAlertTable->setItem(rowCount, 5, locationItem);

        QTableWidgetItem *messageItem = new QTableWidgetItem(messageStr);
        messageItem->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        ui->pAlertTable->setItem(rowCount, 6, messageItem);

        if(levelStr == "critical") {
            for(int i = 0; i < 7; i++) {
                QTableWidgetItem *currentItem = ui->pAlertTable->item(rowCount, i);
                if(currentItem) {
                    currentItem->setBackground(Qt::red);
                    currentItem->setForeground(Qt::white);
                }
            }
        }

        rowCount++;
    }
    ui->pLSearchCount->setText(QString("검색 결과:    %1건").arg(rowCount));

    // 컬럼 너비 자동 조정
    for (int col = 0; col < 7; ++col) {
        ui->pAlertTable->resizeColumnToContents(col);
    }
}

