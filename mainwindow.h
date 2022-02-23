/*
 * @Author: your name
 * @Date: 2022-02-23 13:20:42
 * @LastEditTime: 2022-02-23 13:20:56
 * @LastEditors: your name
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/mainwindow.h
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMouseEvent>
#include <QLabel>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;
//    connect(pushButton, SIGNAL(clicked()), this, SLOT(on_pushButton_clicked()));
//    void mousePressEvent(QMouseEvent *event)
};
#endif // MAINWINDOW_H
