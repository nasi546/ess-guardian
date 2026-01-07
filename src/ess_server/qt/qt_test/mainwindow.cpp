#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSqlDatabase>
#include <QSqlError>
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    qDebug() << "MainWindow started";
    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL");
    db.setHostName("10.10.14.109");
    db.setDatabaseName("ess_db");
    db.setUserName("ess");
    db.setPassword("ess1234");
    db.setPort(3306);

    if (db.open()) {
        qDebug() << "DB connected!";
    } else {
        qDebug() << "DB connect failed:" << db.lastError().text();
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
