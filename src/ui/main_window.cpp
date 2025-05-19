#include "main_window.h"
#include "ui_main_window.h"
#include <QMessageBox>
#include <QCoreApplication>
#include <QMetaType>
#include <iostream>

#include "vive_wrapper.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType<E_POSE_TYPE>("E_POSE_TYPE");
    qRegisterMetaType<CartesianPose>("CartesianPose");

    msg_handler_ = new MessageHandler(this);

    init_connect();
    init_style();
    init_label_maps();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init_style()
{
    ui->lineEdit_reciver_window->setDisabled(true);
}

void MainWindow::init_label_maps()
{
    QStringList suffixes = {"x", "y", "z", "A", "B", "C"};

    for (int i = 1; i <= 6; ++i)
    {
        QVector<QLabel*> v_labels, r_labels;

        for (const QString& suffix : suffixes)
        {
            QString v_name = QString("label_v%1_%2").arg(i).arg(suffix);
            QString r_name = QString("label_r%1_%2").arg(i).arg(suffix);

            QLabel* v_label = this->findChild<QLabel*>(v_name);
            QLabel* r_label = this->findChild<QLabel*>(r_name);

            if (v_label) v_labels.append(v_label);
            if (r_label) r_labels.append(r_label);
        }

        v_labels_map.insert(i, v_labels);
        r_labels_map.insert(i, r_labels);
    }
}

void MainWindow::init_connect()
{
    connect(ui->pushButton_connect_ctr, SIGNAL(clicked()), this, SLOT(slot_connect_ctr()));
    connect(ui->pushButton_compute, SIGNAL(clicked()), this, SLOT(slot_compute()));
    connect(ui->pushButton_start_record, SIGNAL(clicked()), this, SLOT(slot_start_record()));
    connect(ui->pushButton_end_record, SIGNAL(clicked()), this, SLOT(slot_end_record()));
    connect(ui->pushButton_start_plackback, SIGNAL(clicked()), this, SLOT(slot_start_playback()));
    connect(ui->pushButton_end_playback, SIGNAL(clicked()), this, SLOT(slot_end_playback()));
    connect(ui->pushButton_delete_mark_result, SIGNAL(clicked()), this, SLOT(slot_delete_calib_result()));
    connect(ui->pushButton_save_mark_result, SIGNAL(clicked()), this, SLOT(slot_save_calib_result()));

    mark_buttons_.clear();
    mark_buttons_ << ui->pushButton_mark_point1 << ui->pushButton_mark_point2 << ui->pushButton_mark_point3
                    << ui->pushButton_mark_point4 << ui->pushButton_mark_point5 << ui->pushButton_mark_point6
                        << ui->pushButton_mark_point7 << ui->pushButton_mark_point8 << ui->pushButton_mark_point9;
    for (auto iter : mark_buttons_)
        connect(iter, SIGNAL(clicked()), this, SLOT(slot_mark_point()));

    connect(ui->pushButton_send, SIGNAL(clicked()), this, SLOT(slot_send()));
    connect(ui->pushButton_clear, SIGNAL(clicked()), this, SLOT(slot_clear()));

    connect(msg_handler_, &MessageHandler::signal_message_received, this, &MainWindow::slot_handle_message);
    connect(msg_handler_, &MessageHandler::signal_mark_point_received, this, &MainWindow::slot_mark_point_received);
    connect(msg_handler_, &MessageHandler::signal_compute_result_received, this, &MainWindow::slot_compute_result_received);
    connect(this, &MainWindow::signal_connect_ctr, msg_handler_, &MessageHandler::slot_handler_start);
    connect(this, &MainWindow::signal_disconnect_ctr, msg_handler_, &MessageHandler::slot_handler_stop);
    connect(this, &MainWindow::signal_send_message, msg_handler_, &MessageHandler::slot_handler_send_message);
    connect(this, &MainWindow::signal_mark_point, msg_handler_, &MessageHandler::slot_handler_mark_point);
    connect(this, &MainWindow::signal_start_record, msg_handler_, &MessageHandler::slot_handler_start_record);
    connect(this, &MainWindow::signal_end_record, msg_handler_, &MessageHandler::slot_handler_end_record);
    connect(this, &MainWindow::signal_start_playback, msg_handler_, &MessageHandler::slot_handler_start_playback);
    connect(this, &MainWindow::signal_end_playback, msg_handler_, &MessageHandler::slot_handler_end_playback);  
}

void MainWindow::slot_handle_message(const QString& msg)
{
    ui->lineEdit_reciver_window->setText(msg);
}

void MainWindow::slot_mark_point_received(E_POSE_TYPE type, int index, CartesianPose pose)
{
    if (index < 1 || index > 6) return;

    QVector<QLabel*>* labels = nullptr;

    if (type == E_POSE_TYPE_ROBOT)
    {
        if (!r_labels_map.contains(index)) return;
        labels = &r_labels_map[index];
    }
    else if (type == E_POSE_TYPE_VIVE)
    {
        if (!v_labels_map.contains(index)) return;
        labels = &v_labels_map[index];
    }
    else
    {
        return;
    }

    static QMap<QString, int> suffix_to_idx = {
        {"x", 0}, {"y", 1}, {"z", 2},
        {"A", 3}, {"B", 4}, {"C", 5}
    };

    QMap<QString, double> values = {
        {"x", pose.position.x},
        {"y", pose.position.y},
        {"z", pose.position.z},
        {"A", pose.orientation.A},
        {"B", pose.orientation.B},
        {"C", pose.orientation.C}
    };

    for (auto it = values.begin(); it != values.end(); ++it)
    {
        int idx = suffix_to_idx.value(it.key(), -1);
        if (labels && idx >= 0 && idx < labels->size())
        {
            (*labels)[idx]->setText(QString::number(it.value(), 'f', 2));
        }
    }
}


void MainWindow::slot_compute_result_received(double result)
{
    ui->label_calibration_error->setText(QString::number(result, 'f', 2));
}

void MainWindow::slot_connect_ctr()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_connect_ctr();
}

void MainWindow::slot_disconnect_ctr()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_disconnect_ctr();
}

void MainWindow::slot_start_record()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_start_record();
}
void MainWindow::slot_end_record()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_end_record();
}

void MainWindow::slot_start_playback()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_start_playback();
}
void MainWindow::slot_end_playback()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_end_playback();
}

void MainWindow::slot_delete_calib_result()
{   
    std::cout << __FUNCTION__ << std::endl;
}
void MainWindow::slot_save_calib_result()
{
    std::cout << __FUNCTION__ << std::endl;
}
void MainWindow::slot_compute()
{
    std::cout << __FUNCTION__ << std::endl;
}

void MainWindow::slot_mark_point()
{
    QObject* obj = sender();
    QPushButton* btn = qobject_cast<QPushButton*>(obj);
    if (!btn) return;
    
    int index = mark_buttons_.indexOf(btn) + 1;
    if (index != -1)
    {
        emit signal_mark_point(index);
        
        
    }
    else
        std::cout << "not found in mark button list" << std::endl;
}


void MainWindow::slot_send()
{
    QString send_text = ui->lineEdit_sender_window->text();
    emit signal_send_message(send_text);
}

void MainWindow::slot_clear()
{
    std::cout << __FUNCTION__ << std::endl;

    ui->lineEdit_reciver_window->clear();
}

