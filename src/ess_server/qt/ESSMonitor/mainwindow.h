#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "tab4access.h"
#include "tab3alert.h"
#include "tab2env.h"
#include "tab1main.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    Tab1Main *pTab1Main;
    Tab2Env *pTab2Env;
    Tab3Alert *pTab3Alert;
    Tab4Access *pTab4Access;

};
#endif // MAINWINDOW_H
