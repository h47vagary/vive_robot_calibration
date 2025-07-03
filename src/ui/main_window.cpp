#include "main_window.h"
#include "ui_main_window.h"
#include <QMessageBox>
#include <QCoreApplication>
#include <QMetaType>
#include <QFile>
#include <iostream>
#include <memory>

#include "vive_wrapper.h"
#include "interpolation.h"
#include "comm_manager.h"


MainWindow::MainWindow(QWidget  *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType<CartesianPose>("CartesianPose");

    comm_ = std::make_shared<CommManager>();
    msg_handler_ = new MessageHandler(comm_, this);
    vive_tracker_reader_ = new ViveTrackerReader();
    vive_tracker_reader_->start();
    vive_tracker_reader_->set_pose_fetch_mode(ViveTrackerReader::PoseFetchMode::NonBlocking);
    //vive_tracker_reader_->start_for_timer();

    calibration_manager_ = new CalibrationManager();
    flange2tcp_calibration_ = new ToolCalibration7Points();
    tracker2tcp_calibration_ = new ToolCalibration7Points();

    track_pose_timer_ = new QTimer(this);
    track_pose_timer_->setInterval(5);

    tcp2tracker_rotation_matrix_ = new Eigen::Matrix4d();

    csv_parser_window_vive = new CSVParserWindow(this);
    csv_parser_window_vive2robot = new CSVParserWindow(this);
    csv_parser_window_vive2robot_inter = new CSVParserWindow(this);
    csv_parser_window_vive2robot_inter_filter = new CSVParserWindow(this);

    vive_tracker_reader_->register_button_callback([](ViveTrackerReader::TrackerButton btn, bool pressed) 
    {
        std::string btn_name = (btn == ViveTrackerReader::TrackerButton::Trigger ? "Trigger" :
                            btn == ViveTrackerReader::TrackerButton::Grip ? "Grip" : "Touchpad");
        std::cout << "Button " << btn_name << (pressed ? " pressed" : " released") << std::endl;
    });

    init_connect();
    init_style();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init_style()
{
    ui->lineEdit_filter_window_size->setText("11");
    ui->lineEdit_polynomial_order->setText("2");
    ui->lineEdit_label_interpolation_interval->setText("4");
    ui->lineEdit_refresh_rate->setText("4");
    ui->checkBox_end_with_chart->setCheckable(true);
}

Eigen::Matrix4d MainWindow::get_tracker2tcp_rotation_matrix()
{
    return *tcp2tracker_rotation_matrix_;
}

void MainWindow::device_poses_to_robot_poses(const std::vector<CartesianPose> &device_poses, std::vector<CartesianPose> &robot_poses, bool is_filtering)
{
    // TCP 相对于追踪器的位置变换
    Eigen::Vector4d pos_vec;
    tracker2tcp_calibration_->get_calibration_pos_vec(pos_vec);
    Eigen::Matrix4d pos_matrix = Eigen::Matrix4d::Identity();
    pos_matrix.block<3,1>(0,3) = pos_vec.head<3>();

    // 标定出来的定位器坐标相对于机器人基座标的齐次变换矩阵
    Eigen::Matrix4d location2robotbase_mat;
    calibration_manager_->get_position_calibration_matrix(location2robotbase_mat);

    // 获取TCP姿态补偿矩阵
    Eigen::Matrix3d orientation_offset_matrix;
    calibration_manager_->get_orientation_offset_matrix(orientation_offset_matrix);

    for (auto iter : device_poses)
    {
        // 将追踪器位姿转为矩阵
        Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(iter);

        // 求TCP在定位器坐标系的位置变化矩阵
        Eigen::Matrix4d tcp2location_pos_mat = tracker_mat * pos_matrix;

        // 求TCP在基坐标系下的位置变换矩阵
        Eigen::Matrix4d tcp2robotbase_pos_mat = location2robotbase_mat * tcp2location_pos_mat;
        Eigen::Matrix3d tcp2robotbase_ori_mat = tcp2robotbase_pos_mat.block<3, 3>(0, 0);

        // 进行补偿TCP姿态
        Eigen::Matrix3d tcp2robotbase_ori_mat_offset = tcp2robotbase_ori_mat * orientation_offset_matrix;
        Eigen::Matrix4d tcp2robotbase_mat = Eigen::Matrix4d::Identity();
        tcp2robotbase_mat.block<3, 3>(0, 0) = tcp2robotbase_ori_mat_offset;
        tcp2robotbase_mat.block<3, 1>(0, 3) = tcp2robotbase_pos_mat.block<3, 1>(0, 3);
        CartesianPose robot_tcp_pose = Utils::matrix_to_pose(tcp2robotbase_mat);

        robot_poses.push_back(robot_tcp_pose);
    }
    if (is_filtering)
    {
        int window_size = ui->lineEdit_filter_window_size->text().toInt();
        std::unique_ptr<IPoseFilter> pose_filter = std::make_unique<MovingAverageFilter>(window_size);
        pose_filter->set_filter_param(window_size);
        pose_filter->filter_position(robot_poses);
        pose_filter->filter_orientation(robot_poses);
    }
}

void MainWindow::extract_pose_vector(const std::vector<TimestampePose> &timestampe_poses, std::vector<CartesianPose> &poses_only)
{
    poses_only.clear();
    for (const auto& tp : timestampe_poses)
    {
        poses_only.push_back(tp.pose);
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
    connect(ui->pushButton_vive_start, SIGNAL(clicked()), this, SLOT(slot_start_update_track_pose()));
    connect(ui->pushButton_vive_stop, SIGNAL(clicked()), this, SLOT(slot_stop_update_track_pose()));
    connect(ui->pushButton_tracker2tcp_markpoint, SIGNAL(clicked()), this, SLOT(slot_tracker2tcp_mark_point()));
    connect(ui->pushButton_tracker2tcp_clear_point, SIGNAL(clicked()), this, SLOT(slot_tracker2tcp_clear_point()));
    connect(ui->pushButton_tracker2tcp_calibrate_pos, SIGNAL(clicked()), this, SLOT(slot_tracker2tcp_calibrate()));
    connect(ui->pushButton_tracker2tcp_calibrate_ori, SIGNAL(clicked()), this, SLOT(slot_tracker2tcp_mark_rotation_use_robotpose()));
    connect(ui->pushButton_flange2tcp_markpoint, SIGNAL(clicked()), this, SLOT(slot_flange2tcp_mark_point()));
    connect(ui->pushButton_flange2tcp_clear_point, SIGNAL(clicked()), this, SLOT(slot_flange2tcp_clear_point()));
    connect(ui->pushButton_flange2tcp_calibrate, SIGNAL(clicked()), this, SLOT(slot_flange2tcp_calibrate()));
    connect(ui->checkBox_use_toolhand, SIGNAL(toggled(bool)), this, SLOT(slot_use_robot_toolhand(bool)));
    connect(ui->checkBox_use_track2tcp, SIGNAL(toggled(bool)), this, SLOT(slot_use_tracker2tcp(bool)));
    connect(ui->pushButton_once_get, SIGNAL(clicked()), this, SIGNAL(signal_linear_error_acquire()));
    connect(ui->pushButton_refresh_rate, SIGNAL(clicked()), this, SLOT(slot_vive_tracker_reader_interval()));
    connect(ui->pushButton_parse_chart, SIGNAL(clicked()), this, SLOT(slot_parse_chart()));
    connect(ui->pushButton_traj_filtering, SIGNAL(clicked()), this, SLOT(slot_traj_filtering()));

    mark_buttons_.clear();
    mark_buttons_ << ui->pushButton_mark_point1 << ui->pushButton_mark_point2 << ui->pushButton_mark_point3
                    << ui->pushButton_mark_point4 << ui->pushButton_mark_point5 << ui->pushButton_mark_point6;
    for (auto iter : mark_buttons_)
        connect(iter, SIGNAL(clicked()), this, SLOT(slot_mark_point()));

    connect(msg_handler_, &MessageHandler::signal_mark_point_received, this, &MainWindow::slot_mark_point_received);
    connect(msg_handler_, &MessageHandler::signal_compute_result_received, this, &MainWindow::slot_compute_result_received);
    connect(msg_handler_, &MessageHandler::signal_flange2tcp_mark_point_received, this, &MainWindow::slot_fanlge2tcp_mark_point_received);
    connect(msg_handler_, &MessageHandler::signal_tracker2tcp_mark_use_robot_pose, this, &MainWindow::slot_tracker2tcp_mark_use_robot_pose);
    connect(msg_handler_, &MessageHandler::signal_get_linear_error_use_robot_pose, this, &MainWindow::slot_get_linear_error_use_robot_pose);
    connect(track_pose_timer_, &QTimer::timeout, this, &MainWindow::slot_track_pose_timeout);
    connect(this, &MainWindow::signal_connect_ctr, msg_handler_, &MessageHandler::slot_handler_start);
    connect(this, &MainWindow::signal_disconnect_ctr, msg_handler_, &MessageHandler::slot_handler_stop);
    connect(this, &MainWindow::signal_mark_point, msg_handler_, &MessageHandler::slot_handler_mark_point);
    connect(this, &MainWindow::signal_start_record, msg_handler_, &MessageHandler::slot_handler_start_record);
    connect(this, &MainWindow::signal_end_record, msg_handler_, &MessageHandler::slot_handler_end_record);
    connect(this, &MainWindow::signal_start_playback, msg_handler_, &MessageHandler::slot_handler_start_playback);
    connect(this, &MainWindow::signal_end_playback, msg_handler_, &MessageHandler::slot_handler_end_playback);
    connect(this, &MainWindow::signal_flang2tcp_mark_point, msg_handler_, &MessageHandler::slot_handler_flang2tcp_mark_point);
    connect(this, &MainWindow::signal_handler_tracker2tcp_mark_rotation_use_robotpose, 
                msg_handler_, &MessageHandler::slot_handler_tracker2tcp_mark_rotation_use_robotpose);
    connect(this, &MainWindow::signal_linear_error_acquire, msg_handler_, &MessageHandler::slot_linear_error_acquire);
}

// abandon
void MainWindow::slot_mark_point_received(int index, CartesianPose pose)
{
    std::cout << __FUNCTION__ << " index: " << index << std::endl;
    std::cout << "point" << " x:" << pose.position.x << " y:" << pose.position.y << " z:" << pose.position.z
                << " A:" << pose.orientation.A << " B:" << pose.orientation.B << " C:" << pose.orientation.C << std::endl;

    if (index < 1 || index > 6) return;

    int index_save = index - 1;
    if (use_toolhand_)
    {
        // 存入点位
        calibration_manager_->set_robot_calibration_positon(index_save, pose.position);
        calibration_manager_->set_robot_calibration_orientation(index_save, pose.orientation);
    }
    else
    {
        // 获取法兰盘->tcp的旋转矩阵
        Eigen::Matrix4d pose_calibration_matrix;
        flange2tcp_calibration_->get_pose_calibration_matrix(pose_calibration_matrix);
        // 将机器人当前法兰盘点位转为旋转矩阵
        Eigen::Matrix4d flange_matrix = Utils::pose_to_matrix(pose);
        // 相乘取得 tcp 点的旋转矩阵
        Eigen::Matrix4d tcp_matrix = flange_matrix * pose_calibration_matrix;
        // 将旋转矩阵转回位姿
        CartesianPose pose_tcp = Utils::matrix_to_pose(tcp_matrix);
        // 存入点位
        calibration_manager_->set_robot_calibration_positon(index_save, pose_tcp.position);
        calibration_manager_->set_robot_calibration_orientation(index_save, pose_tcp.orientation);
    }
    

    return;
}

void MainWindow::slot_compute_result_received(double result)
{
    std::cout << __FUNCTION__ << " result: " << result << std::endl;
    ui->label_calibration_error->setText(QString::number(result, 'f', 2));
}

void MainWindow::slot_fanlge2tcp_mark_point_received(int index, CartesianPose pose)
{
    std::cout << __FUNCTION__ << " index: " << index << std::endl;
    
    flange2tcp_calibration_->set_calibration_pose(index, pose);
    std::cout << "point" << " x:" << pose.position.x << " y:" << pose.position.y << " z:" << pose.position.z
                << " A:" << pose.orientation.A << " B:" << pose.orientation.B << " C:" << pose.orientation.C << std::endl;
    ui->label_flange2tcp_marked_num->setText(QString("需6个点,已记录点数: %1").arg(index));
}

// abandon
void MainWindow::slot_tracker2tcp_mark_use_robot_pose(CartesianPose pose)
{
    // 获取定位器坐标系相对于机器人坐标系的旋转矩阵
    Eigen::Matrix4d location2robotbase_mat;
    calibration_manager_->get_position_calibration_matrix(location2robotbase_mat);
    // 获取当前追踪器的位姿，并转换为旋转矩阵（追踪器相对于定位器）
    CartesianPose tracker_pose = vive_tracker_reader_->get_latest_pose();
    Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(tracker_pose);
    // 追踪器在机器人基座标系下的旋转矩阵
    Eigen::Matrix4d tracker2robotbase;
    tracker2robotbase = location2robotbase_mat * tracker_mat;

    // TCP在追踪器坐标系下的旋转矩阵（仅取平移，目的是保持追踪器姿态，但将位置移动到TCP位置）
    Eigen::Vector4d pos_vec;
    tracker2tcp_calibration_->get_calibration_pos_vec(pos_vec);
    Eigen::Matrix4d pos_matrix = Eigen::Matrix4d::Identity();
    pos_matrix.block<3,1>(0,3) = pos_vec.head<3>();  // 只取前三个作为平移

    // 求TCP在机器人基座坐标系下的旋转矩阵（位置为TCP，但姿态为追踪器）
    Eigen::Matrix4d tcp2robotbase;
    tcp2robotbase = tracker2robotbase * pos_matrix;
    CartesianPose tcp2rbase_pose = Utils::matrix_to_pose(tcp2robotbase);

    // 计算TCP在机器人基座标下的姿态补偿
    calibration_manager_->set_calibration_orientation(pose.orientation, tcp2rbase_pose.orientation);
    calibration_manager_->calculate_orientation_offset_matrix();

}

void MainWindow::slot_get_linear_error_use_robot_pose(CartesianPose pose)
{
    std::cout << __FUNCTION__ << std::endl;
    std::cout << "point" << " x:" << pose.position.x << " y:" << pose.position.y << " z:" << pose.position.z
                << " A:" << pose.orientation.A << " B:" << pose.orientation.B << " C:" << pose.orientation.C << std::endl;
    
    // TCP 相对于追踪器的位置变换
    Eigen::Vector4d pos_vec;
    tracker2tcp_calibration_->get_calibration_pos_vec(pos_vec);
    Eigen::Matrix4d pos_matrix = Eigen::Matrix4d::Identity();
    pos_matrix.block<3,1>(0,3) = pos_vec.head<3>();

    // 当前追踪器在定位器坐标系位姿
    CartesianPose tracker_pose = vive_tracker_reader_->get_latest_pose();
    Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(tracker_pose);

    // 标定出来的定位器坐标相对于机器人基座标的齐次变换矩阵
    Eigen::Matrix4d location2robotbase_mat;
    calibration_manager_->get_position_calibration_matrix(location2robotbase_mat);

    // 求TCP在定位器坐标系的位置变化矩阵
    Eigen::Matrix4d tcp2location_pos_mat = tracker_mat * pos_matrix;
    // 求TCP在基坐标系下的位置变换矩阵
    Eigen::Matrix4d tcp2robotbase_pos_mat = location2robotbase_mat * tcp2location_pos_mat;
    Eigen::Matrix3d tcp2robotbase_ori_mat = tcp2robotbase_pos_mat.block<3, 3>(0, 0);

    // 获取TCP姿态补偿矩阵
    Eigen::Matrix3d orientation_offset_matrix;
    calibration_manager_->get_orientation_offset_matrix(orientation_offset_matrix);

    // 进行补偿TCP姿态
    // Eigen::Matrix3d tcp2robotbase_ori_mat_offset = orientation_offset_matrix * tcp2robotbase_ori_mat; //左乘
    Eigen::Matrix3d tcp2robotbase_ori_mat_offset = tcp2robotbase_ori_mat * orientation_offset_matrix;

    Eigen::Matrix4d tcp2robotbase_mat = Eigen::Matrix4d::Identity();
    tcp2robotbase_mat.block<3, 3>(0, 0) = tcp2robotbase_ori_mat_offset;
    tcp2robotbase_mat.block<3, 1>(0, 3) = tcp2robotbase_pos_mat.block<3, 1>(0, 3);


    CartesianPose robot_tcp_pose = Utils::matrix_to_pose(tcp2robotbase_mat);

    std::cout << "convert robot_tcp_pose point" << " x:" << robot_tcp_pose.position.x << " y:" << robot_tcp_pose.position.y << " z:" << robot_tcp_pose.position.z
                << " A:" << robot_tcp_pose.orientation.A << " B:" << robot_tcp_pose.orientation.B << " C:" << robot_tcp_pose.orientation.C << std::endl;

    std::cout << "read robot_tcp_pose point" << " x:" << pose.position.x << " y:" << pose.position.y << " z:" << pose.position.z
                << " A:" << pose.orientation.A << " B:" << pose.orientation.B << " C:" << pose.orientation.C << std::endl;

    double x = pose.position.x - robot_tcp_pose.position.x;
    double y = pose.position.y - robot_tcp_pose.position.y;
    double z = pose.position.z - robot_tcp_pose.position.z;
    linear_error_ = std::sqrt(x * x + y * y + z * z);
    std::cout << "#### linear_error: " << linear_error_ << std::endl;
    ui->label_linear_error->setText(QString::number(linear_error_, 'f', 2));
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
    vive_tracker_reader_->enable_record();
}

#define INTERPOLATE_FILTER 1            // 插补->滤波
#define FILTER_INTERPOLATE_FILTER 0     // 滤波->插补->滤波

void MainWindow::slot_end_record()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_end_record();
    vive_tracker_reader_->disable_record();
    
#if FILTER_INTERPOLATE_FILTER
    // 滤波
    #if 0
    std::vector<CartesianPose> poses = vive_tracker_reader_->get_recorded_poses();
    std::vector<CartesianPose> poses_tcp2rb;
    poses_tcp2rb.clear();
    bool is_filtering = ui->checkBox_filtering->isChecked();
    device_poses_to_robot_poses(poses, poses_tcp2rb, is_filtering);
    #else
    std::vector<TimestampePose> poses_timestampe = vive_tracker_reader_->get_recorded_timestamped_poses();
    std::vector<CartesianPose> poses;
    std::vector<TimestampePose> poses_tcp2rb_timestampe;
    std::vector<CartesianPose> poses_tcp2rb;
    extract_pose_vector(poses_timestampe, poses);

    bool is_filtering = ui->checkBox_filtering->isChecked();
    device_poses_to_robot_poses(poses, poses_tcp2rb, is_filtering);

    poses_tcp2rb_timestampe.clear();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        poses_tcp2rb_timestampe.push_back({poses_tcp2rb[i], poses_timestampe[i].timestamp_us});
    }
    #endif

    std::cout << "recorded poses size: " << poses.size() << std::endl;
    std::cout << "poses_tcp2rb size: " << poses_tcp2rb.size() << std::endl;

    // 插补
    std::vector<TimestampePose> poses_tcp2rb_timestampe_interpolation;
    poses_tcp2rb_timestampe_interpolation.clear();
    if (ui->checkBox_label_interpolation_interval->isCheckable())
    {
        uint64_t interval_us = ui->lineEdit_label_interpolation_interval->text().toUInt();
        LinearPoseInterpolator polator(interval_us);
        polator.interpolate_with_quaternion(poses_tcp2rb_timestampe, poses_tcp2rb_timestampe_interpolation);
        std::cout << "poses_tcp2rb_timestampe_interpolation size: " << poses_tcp2rb_timestampe_interpolation.size() << std::endl;
    }

    // 插补后二次滤波
    std::vector<TimestampePose> poses_tcp2rb_timestampe_interpolation_filter;
    poses_tcp2rb_timestampe_interpolation_filter.clear();
    std::vector<CartesianPose> poses_filted;
    extract_pose_vector(poses_tcp2rb_timestampe_interpolation, poses_filted);
    int window_size = ui->lineEdit_filter_window_size->text().toInt();
    int polynomial_order = ui->lineEdit_polynomial_order->text().toInt();
    PoseFilter pose_filter(11, 2);
    pose_filter.set_filter_param(window_size, polynomial_order, MovingAverage);
    pose_filter.filter_position(poses_filted);
    pose_filter.filter_orientation(poses_filted);
    for (size_t i = 0; i < poses_filted.size(); ++i)
    {
        poses_tcp2rb_timestampe_interpolation_filter.push_back({poses_filted[i], poses_tcp2rb_timestampe_interpolation[i].timestamp_us});
    }
    
    // 保存文件
    #if 0
    vive_tracker_reader_->save_record_poses_to_file("vive_traj.csv", poses);
    vive_tracker_reader_->save_record_poses_to_file("vive_traj2robot.csv", poses_tcp2rb);
    #else
    vive_tracker_reader_->save_record_timestamped_poses_to_file("vive_traj.csv", poses_timestampe);
    vive_tracker_reader_->save_record_timestamped_poses_to_file("vive_traj2robot.csv", poses_tcp2rb_timestampe);
    if (ui->checkBox_label_interpolation_interval->isCheckable())
    {
        vive_tracker_reader_->save_record_timestamped_poses_to_file("vive_traj2robot_interpolation.csv", poses_tcp2rb_timestampe_interpolation);
        vive_tracker_reader_->save_record_timestamped_poses_to_file("vive_traj2robot_interpolation_filted.csv", poses_tcp2rb_timestampe_interpolation_filter);
    }
        
    #endif

    // 显示图像
    csv_parser_window_vive->loadData("vive_traj.csv");
    csv_parser_window_vive->show();
    csv_parser_window_vive->plotData();
    csv_parser_window_vive->setWindowTitle("vive数据曲线");

    csv_parser_window_vive2robot->loadData("vive_traj2robot.csv");
    csv_parser_window_vive2robot->show();
    csv_parser_window_vive2robot->plotData();
    csv_parser_window_vive2robot->setWindowTitle("vive2robot数据曲线");

    if (ui->checkBox_label_interpolation_interval->isCheckable())
    {
        csv_parser_window_vive2robot_inter->loadData("vive_traj2robot_interpolation.csv");
        csv_parser_window_vive2robot_inter->show();
        csv_parser_window_vive2robot_inter->plotData();
        csv_parser_window_vive2robot_inter->setWindowTitle("vive2robot_插补_数据曲线");

        csv_parser_window_vive2robot_inter_filter->loadData("vive_traj2robot_interpolation_filted.csv");
        csv_parser_window_vive2robot_inter_filter->show();
        csv_parser_window_vive2robot_inter_filter->plotData();
        csv_parser_window_vive2robot_inter_filter->setWindowTitle("vive2robot_插补_二次滤波_数据曲线");
    }
    
    // 清理点容器
    vive_tracker_reader_->clear_recorded_poses();
    poses_tcp2rb.clear();
#endif

#if INTERPOLATE_FILTER
    // track pose -> robot pose
    std::vector<TimestampePose> poses_timestampe = vive_tracker_reader_->get_recorded_poses();
    std::vector<CartesianPose> poses;
    std::vector<TimestampePose> poses_tcp2rb_timestampe;
    std::vector<CartesianPose> poses_tcp2rb;
    extract_pose_vector(poses_timestampe, poses);
    device_poses_to_robot_poses(poses, poses_tcp2rb, false);
    poses_tcp2rb_timestampe.clear();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        poses_tcp2rb_timestampe.push_back({poses_tcp2rb[i], poses_timestampe[i].timestamp_us});
    }
    std::cout << "recorded poses size: " << poses.size() << std::endl;
    std::cout << "poses_tcp2rb size: " << poses_tcp2rb.size() << std::endl;

    // interpolate
    std::vector<TimestampePose> poses_tcp2rb_timestampe_interpolation;
    poses_tcp2rb_timestampe_interpolation.clear();
    if (ui->checkBox_label_interpolation_interval->isCheckable())
    {
        uint64_t interval_us = ui->lineEdit_label_interpolation_interval->text().toUInt();
        LinearPoseInterpolator polator(interval_us);
        polator.interpolate_with_quaternion(poses_tcp2rb_timestampe, poses_tcp2rb_timestampe_interpolation);        // 卡死//
        std::cout << "poses_tcp2rb_timestampe_interpolation size: " << poses_tcp2rb_timestampe_interpolation.size() << std::endl;
    }
    // filter
    std::vector<TimestampePose> poses_tcp2rb_timestampe_interpolation_filter;
    poses_tcp2rb_timestampe_interpolation_filter.clear();
    std::vector<CartesianPose> poses_filted;
    extract_pose_vector(poses_tcp2rb_timestampe_interpolation, poses_filted);

    int window_size = ui->lineEdit_filter_window_size->text().toInt();
    std::unique_ptr<IPoseFilter> pose_filter = std::make_unique<MovingAverageFilter>(window_size);
    pose_filter->set_filter_param(window_size);
    pose_filter->filter_position(poses_filted);
    pose_filter->filter_orientation(poses_filted);
    for (size_t i = 0; i < poses_filted.size(); ++i)
    {
        poses_tcp2rb_timestampe_interpolation_filter.push_back({poses_filted[i], poses_tcp2rb_timestampe_interpolation[i].timestamp_us});
    }
    // save to file
    vive_tracker_reader_->save_record_poses_to_file("vive_traj.csv", poses_timestampe);
    vive_tracker_reader_->save_record_poses_to_file("vive_traj2robot.csv", poses_tcp2rb_timestampe);
    if (ui->checkBox_label_interpolation_interval->isCheckable())
    {
        vive_tracker_reader_->save_record_poses_to_file("vive_traj2robot_interpolation.csv", poses_tcp2rb_timestampe_interpolation);
        vive_tracker_reader_->save_record_poses_to_file("vive_traj2robot_interpolation_filted.csv", poses_tcp2rb_timestampe_interpolation_filter);
    }

    // clear cache
    vive_tracker_reader_->clear_recorded_poses();
    poses_tcp2rb.clear();

    // show chart
    if (ui->checkBox_end_with_chart->isCheckable())
    {
        csv_parser_window_vive->loadData("vive_traj.csv");
        csv_parser_window_vive->show();
        csv_parser_window_vive->plotData();
        csv_parser_window_vive->setWindowTitle("vive数据曲线");

        csv_parser_window_vive2robot->loadData("vive_traj2robot.csv");
        csv_parser_window_vive2robot->show();
        csv_parser_window_vive2robot->plotData();
        csv_parser_window_vive2robot->setWindowTitle("vive2robot数据曲线");

        if (ui->checkBox_label_interpolation_interval->isCheckable())
        {
            csv_parser_window_vive2robot_inter->loadData("vive_traj2robot_interpolation.csv");
            csv_parser_window_vive2robot_inter->show();
            csv_parser_window_vive2robot_inter->plotData();
            csv_parser_window_vive2robot_inter->setWindowTitle("vive2robot_插补_数据曲线");

            csv_parser_window_vive2robot_inter_filter->loadData("vive_traj2robot_interpolation_filted.csv");
            csv_parser_window_vive2robot_inter_filter->show();
            csv_parser_window_vive2robot_inter_filter->plotData();
            csv_parser_window_vive2robot_inter_filter->setWindowTitle("vive2robot_插补_二次滤波_数据曲线");
        }
    }
#endif
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
    calibration_manager_->clear_calibration_position();
}
void MainWindow::slot_save_calib_result()
{
    std::cout << __FUNCTION__ << std::endl;
}
void MainWindow::slot_compute()
{
    double max_error = -1000.0;
    calibration_manager_->set_calibration_algorithm(2);
    int ret = calibration_manager_->calibrate(0, max_error);
    ui->label_calibration_error->setText(QString::number(max_error));

}

void MainWindow::slot_mark_point()
{
    bool flange2tcp_calibrated = false, tracker2tcp_calibrated = false;
    flange2tcp_calibration_->get_calibrated(flange2tcp_calibrated);
    tracker2tcp_calibration_->get_calibrated(tracker2tcp_calibrated);
    if (!flange2tcp_calibration_ || !tracker2tcp_calibration_)
    {
        std::cout << "flange2tcp and tracker2tcp need to be calibrated first" << std::endl;
        return;
    }

    QObject* obj = sender();
    QPushButton* btn = qobject_cast<QPushButton*>(obj);
    if (!btn) return;
    
    int index = mark_buttons_.indexOf(btn) + 1;
    if (index != -1)
    {
        // 获取机器人当前点位
        std::vector<double> cur_rob_pos;
        if (!comm_->nrc_get_current_position_robot(NRC_FIRST_ROBOT, NRC_PCS, cur_rob_pos))
        {
            // 卡工具手标定完，才能标坐标系之间的关系
            CartesianPosition position(cur_rob_pos.at(0), cur_rob_pos.at(1), cur_rob_pos.at(2));
            CartesianOrientation orientation(cur_rob_pos.at(3), cur_rob_pos.at(4), cur_rob_pos.at(5));
            calibration_manager_->set_robot_calibration_positon(index, position);
            calibration_manager_->set_robot_calibration_orientation(index, orientation);
        }
        
        // 获取 Tracker 当前位姿
        CartesianPose pose_tracker = vive_tracker_reader_->get_latest_pose();
        std::cout << "pose_tracker point" << " x:" << pose_tracker.position.x << " y:" << pose_tracker.position.y << " z:" << pose_tracker.position.z
                << " A:" << pose_tracker.orientation.A << " B:" << pose_tracker.orientation.B << " C:" << pose_tracker.orientation.C << std::endl;
        Eigen::Matrix4d tracker_matrix = Utils::pose_to_matrix(pose_tracker);
        std::cout << " tracker_matrix before: " << std::endl;
        Utils::print_matrix(tracker_matrix);

        // 获取 Tracker->tcp 的位置向量变换，再拼成齐次变换矩阵
        Eigen::Vector4d pos_vec;
        tracker2tcp_calibration_->get_calibration_pos_vec(pos_vec);
        std::cout << " pos_vec.(0): " << pos_vec(0) << " pos_vec(1): " << pos_vec(1) << " pos_vec(2): " << pos_vec(2) << std::endl;
        Eigen::Matrix4d pos_matrix = Eigen::Matrix4d::Identity();
        pos_matrix.block<3,1>(0,3) = pos_vec.head<3>();  // 只取前三个作为平移
        std::cout << "pos_vec_matrix: " << std::endl;
        std::cout << pos_matrix << std::endl;

        // 将追踪器的旋转矩阵与位置向量变换矩阵相乘，得到追踪器坐标系下的TCP位置
        Eigen::Matrix4d tcp2location_matrix = tracker_matrix * pos_matrix;
        std::cout << "\n tcp2location_matrix: " << std::endl;
        Utils::print_matrix(tcp2location_matrix);

        // 再将旋转矩阵转回位姿点位
        CartesianPose pose_tcp = Utils::matrix_to_pose(tcp2location_matrix);
        
        // 存入标定管理器的位置、姿态
        int index_real = index - 1;
        calibration_manager_->set_device_calibration_position(index_real, pose_tcp.position);
        calibration_manager_->set_device_calibration_orientation(index_real, pose_tcp.orientation);
        std::cout << "\n pose_tcp" << " x:" << pose_tcp.position.x << " y:" << pose_tcp.position.y << " z:" << pose_tcp.position.z
                << " A:" << pose_tcp.orientation.A << " B:" << pose_tcp.orientation.B << " C:" << pose_tcp.orientation.C << std::endl;
    }
    else
        std::cout << "not found in mark button list" << std::endl;
}

void MainWindow::slot_tracker2tcp_mark_point()
{
    CartesianPose pose = vive_tracker_reader_->get_latest_pose();
    std::vector<CartesianPose> mark_poses;
    tracker2tcp_calibration_->get_calibration_poses(mark_poses);
    int size = mark_poses.size();
    tracker2tcp_calibration_->set_calibration_pose(size, pose);
    std::cout << "point" << size << " x:" << pose.position.x << " y:" << pose.position.y << " z:" << pose.position.z
                << " A:" << pose.orientation.A << " B:" << pose.orientation.B << " C:" << pose.orientation.C << std::endl;
    ui->label_tracker2tcp_marked_num->setText(QString("需6个点,已记录点数: %1").arg(size + 1));
}

void MainWindow::slot_tracker2tcp_calibrate()
{
    int ret = tracker2tcp_calibration_->calibrate();
    if (ret)
        std::cout << "tracker2tcp calibrate fail" << std::endl;
    else
    {
        std::cout << "tracker2tcp calibrate success" << std::endl;
        Eigen::Vector4d calib_pos_vec;
        tracker2tcp_calibration_->get_calibration_pos_vec(calib_pos_vec);
        std::cout << "tcp2tracker_vec: ["
          << calib_pos_vec.x() << ", "
          << calib_pos_vec.y() << ", "
          << calib_pos_vec.z() << "]" << std::endl;
    }
}

void MainWindow::slot_tracker2tcp_clear_point()
{
    tracker2tcp_calibration_->clear_calibration_pose();
}

void MainWindow::slot_flange2tcp_mark_point()
{
    std::vector<CartesianPose> mark_poses;
    flange2tcp_calibration_->get_calibration_poses(mark_poses);
    int size = mark_poses.size();
    std::vector<double> cur_rob_pos;
    if (!comm_->nrc_get_current_position_robot(NRC_FIRST_ROBOT, NRC_PCS, cur_rob_pos))
    {
        CartesianPose pose(cur_rob_pos.at(0), cur_rob_pos.at(1), cur_rob_pos.at(2),
                            cur_rob_pos.at(3), cur_rob_pos.at(4), cur_rob_pos.at(5));
        flange2tcp_calibration_->set_calibration_pose(size, pose);
    }
}

void MainWindow::slot_flange2tcp_calibrate()
{
    int ret = flange2tcp_calibration_->calibrate();
    if (ret)
        std::cout << "flange2tcp_calibration_ calibrate fail" << std::endl;
    else
    {
        std::cout << "flange2tcp calibrate success" << std::endl;
        Eigen::Matrix4d calib_matrix;
        flange2tcp_calibration_->get_pose_calibration_matrix(calib_matrix);
        std::cout << "flange2tcp calib matrix:\n" << calib_matrix << std::endl;
    }
}

void MainWindow::slot_flange2tcp_clear_point()
{
    flange2tcp_calibration_->clear_calibration_pose();
}

void MainWindow::slot_track_pose_timeout()
{
    CartesianPose pose = vive_tracker_reader_->get_latest_pose();
    ui->label_vive_A->setText(QString::number(pose.orientation.A));
    ui->label_vive_B->setText(QString::number(pose.orientation.B));
    ui->label_vive_C->setText(QString::number(pose.orientation.C));
    ui->label_vive_x->setText(QString::number(pose.position.x));
    ui->label_vive_y->setText(QString::number(pose.position.y));
    ui->label_vive_z->setText(QString::number(pose.position.z));
}

void MainWindow::slot_start_update_track_pose()
{
    track_pose_timer_->start();
}

void MainWindow::slot_stop_update_track_pose()
{
    track_pose_timer_->stop();
}

void MainWindow::slot_tracker2tcp_mark_rotation_use_robotpose()
{
    std::cout << __FUNCTION__ << std::endl;
    bool manager_calibrated = false, flange2tcp_calibrated = false, tracker2tcp_pos_calibrated = false;
    calibration_manager_->get_calibrated(manager_calibrated);
    flange2tcp_calibration_->get_calibrated(flange2tcp_calibrated);
    tracker2tcp_calibration_->get_calibrated(tracker2tcp_pos_calibrated);
    if (!calibration_manager_ || !flange2tcp_calibration_ || !tracker2tcp_calibration_)
    {
        std::cout << "not all calibrated yet" << std::endl;
        std::cout << "calibration_manager_: " << calibration_manager_ << " flange2tcp_calibration_: " << flange2tcp_calibration_ 
                    << " tracker2tcp_calibration_: " << tracker2tcp_calibration_ << std::endl;
        return;
    }
    
    // 获取当前机器人位姿
    std::vector<double> cur_rob_pos;
    if (comm_->nrc_get_current_position_robot(NRC_FIRST_ROBOT, NRC_PCS, cur_rob_pos))
    {
        std::cerr << "nrc get current position robot" << std::endl;
        return;
    }
    CartesianPose pose(cur_rob_pos.at(0), cur_rob_pos.at(1), cur_rob_pos.at(2),
                            cur_rob_pos.at(3), cur_rob_pos.at(4), cur_rob_pos.at(5));

    // 获取定位器坐标系相对于机器人坐标系的旋转矩阵
    Eigen::Matrix4d location2robotbase_mat;
    calibration_manager_->get_position_calibration_matrix(location2robotbase_mat);
    // 获取当前追踪器的位姿，并转换为旋转矩阵（追踪器相对于定位器）
    CartesianPose tracker_pose = vive_tracker_reader_->get_latest_pose();
    Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(tracker_pose);
    // 追踪器在机器人基座标系下的旋转矩阵
    Eigen::Matrix4d tracker2robotbase;
    tracker2robotbase = location2robotbase_mat * tracker_mat;

    // TCP在追踪器坐标系下的旋转矩阵（仅取平移，目的是保持追踪器姿态，但将位置移动到TCP位置）
    Eigen::Vector4d pos_vec;
    tracker2tcp_calibration_->get_calibration_pos_vec(pos_vec);
    Eigen::Matrix4d pos_matrix = Eigen::Matrix4d::Identity();
    pos_matrix.block<3,1>(0,3) = pos_vec.head<3>();  // 只取前三个作为平移

    // 求TCP在机器人基座坐标系下的旋转矩阵（位置为TCP，但姿态为追踪器）
    Eigen::Matrix4d tcp2robotbase;
    tcp2robotbase = tracker2robotbase * pos_matrix;
    CartesianPose tcp2rbase_pose = Utils::matrix_to_pose(tcp2robotbase);

    // 计算TCP在机器人基座标下的姿态补偿
    calibration_manager_->set_calibration_orientation(pose.orientation, tcp2rbase_pose.orientation);
    calibration_manager_->calculate_orientation_offset_matrix();

    //emit signal_handler_tracker2tcp_mark_rotation_use_robotpose();
}

void MainWindow::slot_use_robot_toolhand(bool state)
{
    if (ui->checkBox_use_toolhand->isChecked())
    {
        std::cout << "use_toolhand_ is_checked true" << std::endl;
        use_toolhand_ = true;
        
    }
    else
    {
        std::cout << "use_toolhand_ is_checked false" << std::endl;
        use_toolhand_ = false;
    }
}

void MainWindow::slot_use_tracker2tcp(bool)
{
    if (ui->checkBox_use_track2tcp->isChecked())
    {
        std::cout << "use_track2tcp_ is_checked true" << std::endl;
        use_track2tcp_ = true;
        //Eigen::Vector4d tcp2tracker_pos_vec(1.77239, -22.9953, 245.185, 1);
        //Eigen::Vector4d tcp2tracker_pos_vec(0, -35, 270, 1);
        Eigen::Vector4d tcp2tracker_pos_vec(-6.32326, 37.9684, -264.24, 1);
        tracker2tcp_calibration_->set_calibration_pos_vec(tcp2tracker_pos_vec);
    }
    else
    {
        std::cout << "use_track2tcp_ is_checked false" << std::endl;
        use_track2tcp_ = false;
    }
}

void MainWindow::slot_vive_tracker_reader_interval()
{
    int interval_ms = ui->lineEdit_refresh_rate->text().toInt();
    if (interval_ms <= 0)
    {
        std::cout << "interval too small, set to 4ms" << std::endl;
        interval_ms = 4;
    }
    vive_tracker_reader_->set_loop_interval_ms(interval_ms);
}

void MainWindow::slot_parse_chart()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("选择CSV文件"), "", tr("CSV Files (*.csv);;All Files (*.*)"));
    if (!filename.isEmpty()) 
    {
        bool is_filtering = ui->checkBox_filtering->isChecked();
        int window_size = ui->lineEdit_filter_window_size->text().toInt();
        int polynomial_order = ui->lineEdit_polynomial_order->text().toInt();

        CSVParserWindow* csv_parser_window = new CSVParserWindow();
        csv_parser_window->setAttribute(Qt::WA_DeleteOnClose);
        csv_parser_window->set_filter_param(window_size);
        csv_parser_window->loadData(filename.toStdString(), is_filtering);
        csv_parser_window->plotData();
        csv_parser_window->show();
    }
}

void MainWindow::slot_traj_filtering()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("选择CSV文件"), "", tr("CSV Files (*.csv);;All Files (*.*)"));
    if (!filename.isEmpty())
    {
        QFileInfo fileInfo(filename);
        QString baseName = fileInfo.completeBaseName();
        QString suffix = fileInfo.completeSuffix();
        QString dir = fileInfo.absolutePath();
        QString filename_filter = dir + "/" + baseName + "_filtered." + suffix;

        bool is_filtering = ui->checkBox_filtering->isChecked();
        int window_size = ui->lineEdit_filter_window_size->text().toInt();
        int polynomial_order = ui->lineEdit_polynomial_order->text().toInt();
        CSVParserWindow* csv_parser_window = new CSVParserWindow();
        csv_parser_window->setAttribute(Qt::WA_DeleteOnClose);
        csv_parser_window->set_filter_param(window_size);
        csv_parser_window->loadData(filename.toStdString(), is_filtering);
        csv_parser_window->save_data_to_file(filename_filter.toStdString());
        csv_parser_window->plotData();
        csv_parser_window->show();

    }
}
