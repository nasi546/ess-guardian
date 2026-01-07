#include "loginwindow.h"
#include "ui_loginwindow.h"
#include "mainwindow.h"

LoginWindow::LoginWindow(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::LoginWindow)
{
    ui->setupUi(this);

     this->setWindowTitle("Login");

    connect(ui->pLineEdit_PW, SIGNAL(returnPressed()), ui->pPBtton_Login, SLOT(click()));
}

LoginWindow::~LoginWindow()
{
    delete ui;
}

void LoginWindow::on_pPBtton_Login_clicked()
{
    static int cnt = 0;
    QString id = ui->pLineEdit_ID->text();
    QString pw = ui->pLineEdit_PW->text();

    if(cnt < 4)
    {
        if(id == "ess_admin" && pw == "ess1234")
        {
            MainWindow *pMainWindow = new MainWindow();
            pMainWindow->show();

            this->close();
        }
        else
        {
            cnt++;
            ui->pLineEdit_ID->clear();
            ui->pLineEdit_PW->clear();
            ui->pLabel_status->setText(QString("Login Failed. Please try again. %1/5").arg(cnt));
        }
    }
    else
    {
        ui->pPBtton_Login->setEnabled(false);
        ui->pLabel_status->setText("Too many failed login attempts. Please try again later.");
    }
}
