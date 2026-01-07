#include "loginwindow.h"
#include <QApplication>
#include <QSqlDatabase>
#include <QMessageBox>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QSqlDatabase db = QSqlDatabase::addDatabase("QMYSQL", "ess_db");
    db.setHostName("10.10.14.109");
    db.setDatabaseName("ess_db");
    db.setUserName("ess");
    db.setPassword("ess1234");
    // db.open();

    if (!db.open())
    {
        QMessageBox::critical(nullptr, "DB Error", "DB connection failed");
        return -1;
    }

    LoginWindow login;
    login.show();

    return a.exec();
}
