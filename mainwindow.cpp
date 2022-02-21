#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
//    ui.button_exit->setText(tr("(myExitButtonFunc)"));
    QLabel *label = new QLabel("Hello");
            label->show();
}



void MainWindow::on_pushButton_2_clicked()
{
    QLabel *label = new QLabel("Hello22222222222");
            label->show();

}
