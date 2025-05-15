#include "main_window.h"
#include "ui_main_window.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->pushButton_mark_point1, &QPushButton::clicked, this, [=]() {
        QMessageBox::information(this, "Hi", "Button clicked!");
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}
