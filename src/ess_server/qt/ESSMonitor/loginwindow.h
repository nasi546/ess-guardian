#ifndef LOGINWINDOW_H
#define LOGINWINDOW_H

#include <QWidget>

namespace Ui {
class LoginWindow;
}

class LoginWindow : public QWidget
{
    Q_OBJECT

public:
    explicit LoginWindow(QWidget *parent = nullptr);
    ~LoginWindow();

private slots:
    void on_pPBtton_Login_clicked();

private:
    Ui::LoginWindow *ui;
};

#endif // LOGINWINDOW_H
