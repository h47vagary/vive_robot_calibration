#pragma once

#include <QMainWindow>
#include <QList>
#include <QPushButton>
#include <QLabel>
#include <QMap>
#include <QVector>
#include <QTimer>

#include "message_handler.h"
#include "vive_tracker_reader.h"
#include "calibration.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void slot_connect_ctr();
    void slot_disconnect_ctr();

    void slot_start_record();
    void slot_end_record();
    void slot_start_playback();
    void slot_end_playback();

    void slot_delete_calib_result();
    void slot_save_calib_result();
    void slot_compute();
    
    void slot_mark_point();

    void slot_send();
    void slot_clear();

    void slot_tracker2tcp_mark_point();
    void slot_tracker2tcp_calibrate();
    void slot_tracker2tcp_clear_point();
    
    void slot_flange2tcp_mark_point();
    void slot_flange2tcp_calibrate();
    void slot_flange2tcp_clear_point();
    
    void slot_track_pose_timeout();
    void slot_start_update_track_pose();
    void slot_stop_update_track_pose();

public slots:
    void slot_handle_message(const QString& msg);
    void slot_mark_point_received(E_POSE_TYPE type, int index, CartesianPose pose);
    void slot_compute_result_received(double result);
    void slot_fanlge2tcp_mark_point_received(int index, CartesianPose pose);
    void slot_tracker2tcp_mark_use_robot_pose(CartesianPose pose);

signals:
    void signal_connect_ctr();
    void signal_disconnect_ctr();
    void signal_send_message(QString msg);
    void signal_mark_point(int point_index);
    void signal_start_record();
    void signal_end_record();
    void signal_start_playback();
    void signal_end_playback();
    void signal_flang2tcp_mark_point(int point_index);

private:
    void init_connect();
    void init_style();
    void init_label_maps(); // 初始化绑定

    Eigen::Matrix4d get_tracker2tcp_rotation_matrix();

    Ui::MainWindow *ui;
    QTimer* track_pose_timer_;
    QList<QPushButton*> mark_buttons_;
    QMap<int, QVector<QLabel*>> v_labels_map;  // label_v1_x ~ label_v6_C
    QMap<int, QVector<QLabel*>> r_labels_map;  // label_r1_x ~ label_r6_C

    MessageHandler* msg_handler_;
    ViveTrackerReader* vive_tracker_reader_;
    CalibrationManager* calibration_manager_;
    ToolCalibration7Points* flange2tcp_calibration_;
    ToolCalibration7Points* tracker2tcp_calibration_;
    Eigen::Matrix4d* tracker2tcp_rotation_matrix_;
};

