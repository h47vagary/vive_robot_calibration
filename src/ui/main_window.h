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
#include "csv_parser.h" 
#include "filter.h"

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

    void slot_tracker2tcp_mark_point();
    void slot_tracker2tcp_calibrate();
    void slot_tracker2tcp_clear_point();
    
    void slot_flange2tcp_mark_point();
    void slot_flange2tcp_calibrate();
    void slot_flange2tcp_clear_point();
    
    void slot_track_pose_timeout();
    void slot_start_update_track_pose();
    void slot_stop_update_track_pose();
    void slot_tracker2tcp_mark_rotation_use_robotpose();

    void slot_use_robot_toolhand(bool);
    void slot_use_tracker2tcp(bool);

    void slot_vive_tracker_reader_interval();
    void slot_parse_chart();
    void slot_traj_filtering();

public slots:
    void slot_mark_point_received(E_POSE_TYPE type, int index, CartesianPose pose);
    void slot_compute_result_received(double result);
    void slot_fanlge2tcp_mark_point_received(int index, CartesianPose pose);
    void slot_tracker2tcp_mark_use_robot_pose(CartesianPose pose);

    void slot_get_linear_error_use_robot_pose(CartesianPose pose);      // 多一个网络传输时间，稳定后获取才准

signals:
    void signal_connect_ctr();
    void signal_disconnect_ctr();
    void signal_mark_point(int point_index);
    void signal_start_record();
    void signal_end_record();
    void signal_start_playback();
    void signal_end_playback();
    void signal_flang2tcp_mark_point(int point_index);
    void signal_handler_tracker2tcp_mark_rotation_use_robotpose();
    void signal_linear_error_acquire();

private:
    void init_connect();
    void init_style();

    Eigen::Matrix4d get_tracker2tcp_rotation_matrix();

    Ui::MainWindow *ui;
    QTimer* track_pose_timer_;
    QList<QPushButton*> mark_buttons_;

    MessageHandler* msg_handler_;
    ViveTrackerReader* vive_tracker_reader_;
    CalibrationManager* calibration_manager_;
    ToolCalibration7Points* flange2tcp_calibration_;
    ToolCalibration7Points* tracker2tcp_calibration_;
    Eigen::Matrix4d* tcp2tracker_rotation_matrix_;

    bool use_toolhand_ = false;
    bool use_track2tcp_ = false;
    double linear_error_ = 0.0; // 线性误差

    void device_poses_to_robot_poses(const std::vector<CartesianPose>& device_poses, std::vector<CartesianPose>& robot_poses, bool is_filtering);
    void extract_pose_vector(const std::vector<TimestampePose>& timestampe_poses, std::vector<CartesianPose>& poses_only);

    CSVParserWindow* csv_parser_window_vive;
    CSVParserWindow* csv_parser_window_vive2robot;
    CSVParserWindow* csv_parser_window_vive2robot_inter;
    CSVParserWindow* csv_parser_window_vive2robot_inter_filter;
};

