/*
 * @Author: your name
 * @Date: 2022-02-23 13:20:42
 * @LastEditTime: 2022-03-11 11:53:57
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /code/mainwindow.cpp
 */
#include "mainwindow.h"
#include "untitled2_autogen/include/ui_mainwindow.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

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
    //         src=imread("/home/joyce/图片/rm-map-bule.png");
    //                     //toAscii()返回8位描述的string,为QByteArray,data()表示返回QByteArray的指针，QByteArray为字节指针，古老的toascii，我们使用toUtf8。网上有toLatin1，但是好像会出错
    //     //src = imread("girl.jpg");
    // //    namedWindow( "src", WINDOW_NORMAL );
    //    if(!src.empty()){
    //    imshow("src",src);
    //    }

    // //    cvtColor( src, src, COLOR_BGR2RGB );
    //    img = QImage( (const unsigned char*)(src.data), src.cols, src.rows, QImage::Format_RGB888 );
    //    ui->label->setPixmap( QPixmap::fromImage(img));
    //    ui->label->resize( ui->label->pixmap()->size());

}

void MainWindow::show_Image()
{
        src=imread("/home/joyce/图片/rm-map-bule.png");
            QLabel *label = new QLabel("Hello2333");
                        //toAscii()返回8位描述的string,为QByteArray,data()表示返回QByteArray的指针，QByteArray为字节指针，古老的toascii，我们使用toUtf8。网上有toLatin1，但是好像会出错
        //src = imread("girl.jpg");
    //    namedWindow( "src", WINDOW_NORMAL );
       if(!src.empty()){
       imshow("src",src);
       }

    //    cvtColor( src, src, COLOR_BGR2RGB );
       img = QImage( (const unsigned char*)(src.data), src.cols, src.rows, QImage::Format_RGB888 );
       img = QImage( (const unsigned char*)(src.data), src.cols, src.rows, QImage::Format_RGB888 );
       ui->label->setPixmap( QPixmap::fromImage(img));
       ui->label->resize( ui->label->pixmap()->size());
}