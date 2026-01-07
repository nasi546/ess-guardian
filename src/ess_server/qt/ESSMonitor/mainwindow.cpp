#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("ESS Monitor System");

    pTab1Main = new Tab1Main(ui->pTab1);
    ui->pTab1->setLayout(pTab1Main->layout());
    pTab2Env = new Tab2Env(ui->pTab2);
    ui->pTab2->setLayout(pTab2Env->layout());
    pTab3Alert = new Tab3Alert(ui->pTab3);
    ui->pTab3->setLayout(pTab3Alert->layout());
    pTab4Access = new Tab4Access(ui->pTab4);
    ui->pTab4->setLayout(pTab4Access->layout());

    ui->pTabWidget->setCurrentIndex(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}
