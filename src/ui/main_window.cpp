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
    vive_tracker_reader_ = new ViveTrackerReader();
    vive_tracker_reader_->start();
    calibration_manager_ = new CalibrationManager();
    flange2tcp_calibration_ = new ToolCalibration7Points();
    tracker2tcp_calibration_ = new ToolCalibration7Points();

    track_pose_timer_ = new QTimer(this);
    track_pose_timer_->setInterval(5);

    linear_error_continue_acquire_ = new QTimer(this);
    linear_error_continue_acquire_->setInterval(5);

    tcp2tracker_rotation_matrix_ = new Eigen::Matrix4d();

    csv_parser_window_vive = new CSVParserWindow(this);
    csv_parser_window_vive2robot = new CSVParserWindow(this);

    init_connect();
    init_style();
    init_label_maps();

    //init_test_calib();
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

Eigen::Matrix4d MainWindow::get_tracker2tcp_rotation_matrix()
{
    return *tcp2tracker_rotation_matrix_;
}

void MainWindow::init_test_calib()
{
    std::vector<CartesianPose> flange_test_poses_1_;
    std::vector<CartesianPose> tracker_test_poses_1_;
    std::vector<CartesianPose> flange_test_poses_2_;
    std::vector<CartesianPose> tracker_test_poses_2_;

    // 测试数据
    flange_test_poses_1_.push_back({{483.188, 42.6536, 382.135}, {-3.1357, -0.0229753, 1.16085}});
    flange_test_poses_1_.push_back({{477.357, 118.771, 350.244}, {2.9074, -0.00475082, 1.15924}});
    flange_test_poses_1_.push_back({{478.981, -64.6805, 389.427}, {-2.81605, -0.00831388, 1.16014}});
    flange_test_poses_1_.push_back({{608.515, 31.062, 400.169}, {-3.10181, -0.408744, 1.16181}});
    flange_test_poses_1_.push_back({{535.759, 37.478, 386.652}, {-3.1242, -0.0525343, 1.14832}});
    flange_test_poses_1_.push_back({{535.734, -1.01173, 386.581}, {-3.12425, -0.0528958, 1.14867}});
    
    tracker_test_poses_1_.push_back({{226.026, 681.279, -1862.37}, {1.59869, -0.0326651, -0.70241}});
    tracker_test_poses_1_.push_back({{271.676, 669.992, -1915.5}, {1.40636, -0.202555, -0.712652}});
    tracker_test_poses_1_.push_back({{161.133, 668.07, -1808.86}, {1.82494, 0.223285, -0.74037}});
    tracker_test_poses_1_.push_back({{282.832, 655.727, -1782.55}, {1.91891, -0.261512, -0.662689}});
    tracker_test_poses_1_.push_back({{224.368, 685.254, -1854.23}, {1.63115, -0.0349263, -0.713315}});
    tracker_test_poses_1_.push_back({{252.988, 686.764, -1821.67}, {1.63128, -0.0354317, -0.712291}});
    tracker_test_poses_1_.push_back({{225.273, 686.95, -1797.8}, {1.6349, -0.0337237, -0.712568}});

    flange_test_poses_2_.push_back({{490.554, 22.8642, 389.293}, {-3.1194, -0.0864863, 1.03751}});
    flange_test_poses_2_.push_back({{490.492, -109.683, 383.988}, {-2.73236, -0.0875694, 1.0378}});
    flange_test_poses_2_.push_back({{481.507, 178.453, 314.034}, {2.65889, -0.0877801, 1.03728}});
    flange_test_poses_2_.push_back({{659.687, 1.73947, 391.887}, {-3.03688, -0.607033, 1.09382}});
    flange_test_poses_2_.push_back({{489.778, 22.8312, 389.359}, {-3.11965, -0.0986332, 1.03719}});
    flange_test_poses_2_.push_back({{533.272, 22.8465, 389.304}, {-3.11963, -0.0988659, 1.03601}});
    flange_test_poses_2_.push_back({{533.257, -21.4563, 389.229}, {-3.1196, -0.0991147, 1.03609}});

    tracker_test_poses_2_.push_back({{834.229, -173.308, -3082.86}, {1.66904, -0.0240478, -0.482847}});
    tracker_test_poses_2_.push_back({{738.664, -208.484, -3028.29}, {1.8428, 0.344454, -0.563848}});
    tracker_test_poses_2_.push_back({{948.358, -271.747, -3120.06}, {1.42867, -0.506664, -0.493362}});
    tracker_test_poses_2_.push_back({{865.582, -213.73, -2961.71}, {2.18897, -0.155467, -0.407141}});
    tracker_test_poses_2_.push_back({{835.817, -177.911, -3085.96}, {1.68325, -0.0250073, -0.483887}});
    tracker_test_poses_2_.push_back({{854.306, -206.823, -3047.61}, {1.67553, -0.0276747, -0.481855}});
    tracker_test_poses_2_.push_back({{813.337, -155.399, -3029.74}, {1.69072, -0.0218522, -0.479085}});

    for (int i = 0; i < tracker_test_poses_2_.size(); i++)
    {
        tracker2tcp_calibration_->set_calibration_pose(i, tracker_test_poses_2_[i]);
    }
    for (int i = 0; i < flange_test_poses_2_.size(); i++)
    {
        flange2tcp_calibration_->set_calibration_pose(i, flange_test_poses_2_[i]);
    }

    tracker2tcp_calibration_->calibrate();
    bool pos_calibrated;
    tracker2tcp_calibration_->get_calibrated(pos_calibrated);
    if (!pos_calibrated)
    {
        std::cout << __FUNCTION__ << "tracker2tcp calibrate fail" << std::endl;
        return;
    }
    Eigen::Vector4d tcp2tracker_vec;
    tracker2tcp_calibration_->get_calibration_pos_vec(tcp2tracker_vec);
    std::cout << "tcp2tracker_vec: ["
          << tcp2tracker_vec.x() << ", "
          << tcp2tracker_vec.y() << ", "
          << tcp2tracker_vec.z() << "]" << std::endl;



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
    connect(ui->checkBox_continue_get, SIGNAL(toggled(bool)), this, SLOT(slot_continue_get(bool)));
    connect(ui->pushButton_once_get, SIGNAL(clicked()), this, SIGNAL(signal_linear_error_acquire()));

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
    connect(msg_handler_, &MessageHandler::signal_flange2tcp_mark_point_received, this, &MainWindow::slot_fanlge2tcp_mark_point_received);
    connect(msg_handler_, &MessageHandler::signal_tracker2tcp_mark_use_robot_pose, this, &MainWindow::slot_tracker2tcp_mark_use_robot_pose);
    connect(msg_handler_, &MessageHandler::signal_get_linear_error_use_robot_pose, this, &MainWindow::slot_get_linear_error_use_robot_pose);
    connect(track_pose_timer_, &QTimer::timeout, this, &MainWindow::slot_track_pose_timeout);
    connect(linear_error_continue_acquire_, &QTimer::timeout, this, &MainWindow::slot_linear_error_cotinue_acquire);
    connect(this, &MainWindow::signal_connect_ctr, msg_handler_, &MessageHandler::slot_handler_start);
    connect(this, &MainWindow::signal_disconnect_ctr, msg_handler_, &MessageHandler::slot_handler_stop);
    connect(this, &MainWindow::signal_send_message, msg_handler_, &MessageHandler::slot_handler_send_message);
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

void MainWindow::slot_handle_message(const QString& msg)
{
    std::cout << __FUNCTION__ << " msg: " << msg.toStdString() << std::endl;
    ui->lineEdit_reciver_window->setText(msg);
}

void MainWindow::slot_mark_point_received(E_POSE_TYPE type, int index, CartesianPose pose)
{
    std::cout << __FUNCTION__ << " type: " << type << " index: " << index << std::endl;
    std::cout << "point" << " x:" << pose.position.x << " y:" << pose.position.y << " z:" << pose.position.z
                << " A:" << pose.orientation.A << " B:" << pose.orientation.B << " C:" << pose.orientation.C << std::endl;

    //if (index < 1 || index > 6) return;

    QVector<QLabel*>* labels = nullptr;

    if (type == E_POSE_TYPE_ROBOT)
    {
        if (!r_labels_map.contains(index)) return;
        labels = &r_labels_map[index];
        int index_real = index - 1;
        if (use_toolhand_)
        {
            // 存入点位
            calibration_manager_->set_robot_calibration_positon(index_real, pose.position);
            calibration_manager_->set_robot_calibration_orientation(index_real, pose.orientation);
            std::cout << "svd calibration pose: " << "x=" << pose.position.x << " y=" << pose.position.y <<
                        " z=" << pose.position.z << " A=" << pose.orientation.A << " B=" << pose.orientation.B <<
                        " C=" << pose.orientation.C << std::endl;
        }
        else
        {
            // 获取tcp->法兰盘的旋转矩阵
            Eigen::Matrix4d pose_calibration_matrix;
            flange2tcp_calibration_->get_pose_calibration_matrix(pose_calibration_matrix);
            // 将机器人当前法兰盘点位转为旋转矩阵
            Eigen::Matrix4d flange_matrix = Utils::pose_to_matrix(pose);
            // 相乘取得 tcp 点的旋转矩阵
            Eigen::Matrix4d tcp_matrix = flange_matrix * pose_calibration_matrix;
            // 将旋转矩阵转回位姿
            CartesianPose pose_tcp = Utils::matrix_to_pose(tcp_matrix);

            std::cout << " tcp_matrix: " << std::endl;
            Utils::print_matrix(tcp_matrix);

            // 存入点位
            calibration_manager_->set_robot_calibration_positon(index_real, pose_tcp.position);
            calibration_manager_->set_robot_calibration_orientation(index_real, pose_tcp.orientation);
        }
        
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

void MainWindow::slot_tracker2tcp_mark_use_robot_pose(CartesianPose pose)
{
    std::cout << __FUNCTION__ << std::endl;
    #if 0       // 没使用工具手
    // 转换为相对机器人基座标的法兰盘旋转矩阵
    Eigen::Matrix4d flange_mat = Utils::pose_to_matrix(pose);
    // 获取TCP相对于机器人基座标的旋转矩阵
    Eigen::Matrix4d tcp2flange_mat;
    flange2tcp_calibration_->get_pose_calibration_matrix(tcp2flange_mat);
    // 获取定位器坐标系相对于机器人坐标系的旋转矩阵
    Eigen::Matrix4d location2robotbase_mat;
    calibration_manager_->get_calibratoin_matrix(location2robotbase_mat);
    // 获取当前追踪器的位姿，并转换为旋转矩阵（追踪器相对于定位器）
    CartesianPose tracker_pose = vive_tracker_reader_->get_latest_pose();
    Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(tracker_pose);
    
    // 联立求出TCP相对于追踪器的旋转矩阵
    *tcp2tracker_rotation_matrix_ =
    tracker_mat.inverse() * location2robotbase_mat.inverse() * flange_mat * tcp2flange_mat;
    #else       // 使用工具手
    // // 获取TCP相对于机器人基座标的旋转矩阵
    // Eigen::Matrix4d tcp_mat = Utils::pose_to_matrix(pose);
    // std::cout << "\n tcp_mat: " << std::endl;
    // Utils::print_matrix(tcp_mat);
    // // 获取定位器坐标系相对于机器人坐标系的旋转矩阵
    // Eigen::Matrix4d location2robotbase_mat;
    // calibration_manager_->get_position_calibration_matrix(location2robotbase_mat);
    // std::cout << "\n location2robotbase_mat: " << std::endl;
    // Utils::print_matrix(location2robotbase_mat);
    // // 获取当前追踪器的位姿，并转换为旋转矩阵（追踪器相对于定位器）
    // CartesianPose tracker_pose = vive_tracker_reader_->get_latest_pose();
    // Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(tracker_pose);
    // std::cout << "\n tracker_mat: " << std::endl;
    // Utils::print_matrix(tracker_mat);
    // // 联立求出TCP相对于追踪器的旋转矩阵
    // *tcp2tracker_rotation_matrix_ =
    //     tracker_mat.inverse() * location2robotbase_mat.inverse() * tcp_mat;
    // std::cout << "\n *tcp2tracker_rotation_matrix_: " << std::endl;
    // Utils::print_matrix(*tcp2tracker_rotation_matrix_);
    #endif
    // // 拼接 tcp2tracker 的旋转矩阵
    // Eigen::Vector4d tcp2tracker_pos_vec;
    // tracker2tcp_calibration_->get_calibration_pos_vec(tcp2tracker_pos_vec);
    // Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    // mat.block<3, 3>(0, 0) =  tcp2tracker_rotation_matrix_->block<3, 3>(0, 0);
    // mat.block<3, 1>(0, 3) = tcp2tracker_pos_vec.head<3>();

    // std::cout << "mat: " << std::endl;
    // Utils::print_matrix(mat);

    // tracker2tcp_calibration_->set_pose_calibration_matrix(mat);

    // test tracker in robot base
    std::cout << "##### test tracker in robot base #####" << std::endl;
    // 获取定位器坐标系相对于机器人坐标系的旋转矩阵
    Eigen::Matrix4d location2robotbase_mat;
    calibration_manager_->get_position_calibration_matrix(location2robotbase_mat);
    std::cout << "\n location2robotbase_mat: " << std::endl;
    Utils::print_matrix(location2robotbase_mat);
    // 获取当前追踪器的位姿，并转换为旋转矩阵（追踪器相对于定位器）
    CartesianPose tracker_pose = vive_tracker_reader_->get_latest_pose();
    Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(tracker_pose);
    std::cout << "\n tracker_mat: " << std::endl;
    Utils::print_matrix(tracker_mat);
    // 追踪器在机器人基座标系下的旋转矩阵
    Eigen::Matrix4d tracker2robotbase;
    tracker2robotbase = location2robotbase_mat * tracker_mat;
    std::cout << "\n *tracker2robotbase: " << std::endl;
    Utils::print_matrix(tracker2robotbase);

    // TCP在追踪器坐标系下的旋转矩阵（仅取平移，目的是保持追踪器姿态，但将位置移动到TCP位置）
    Eigen::Vector4d pos_vec;
    tracker2tcp_calibration_->get_calibration_pos_vec(pos_vec);
    Eigen::Matrix4d pos_matrix = Eigen::Matrix4d::Identity();
    pos_matrix.block<3,1>(0,3) = pos_vec.head<3>();  // 只取前三个作为平移
    std::cout << "pos_vec_matrix: " << std::endl;
    std::cout << pos_matrix << std::endl;

    // 求TCP在机器人基座坐标系下的旋转矩阵（位置为TCP，但姿态为追踪器）
    Eigen::Matrix4d tcp2robotbase;
    tcp2robotbase = tracker2robotbase * pos_matrix;
    CartesianPose tcp2rbase_pose = Utils::matrix_to_pose(tcp2robotbase);
    std::cout << "tcp2robotbase: " << std::endl;
    std::cout << tcp2robotbase << std::endl;
    std::cout << "tcp2rbase_pose" << " x:" << tcp2rbase_pose.position.x << " y:" << tcp2rbase_pose.position.y << " z:" << tcp2rbase_pose.position.z
                << " A:" << tcp2rbase_pose.orientation.A << " B:" << tcp2rbase_pose.orientation.B << " C:" << tcp2rbase_pose.orientation.C << std::endl;
    
    calibration_manager_->set_calibration_orientation(pose.orientation, tcp2rbase_pose.orientation);
    calibration_manager_->calculate_orientation_offset_matrix();
    Eigen::Matrix3d orientation_offset_matrix;
    calibration_manager_->get_orientation_offset_matrix(orientation_offset_matrix);
    std::cout << " orientation_offset_matrix: " << std::endl;
    std::cout << orientation_offset_matrix << std::endl;
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
    std::cout << "pos_vec_matrix: " << std::endl;
    std::cout << pos_matrix << std::endl;

    // 当前追踪器在定位器坐标系位姿
    CartesianPose tracker_pose = vive_tracker_reader_->get_latest_pose();
    Eigen::Matrix4d tracker_mat = Utils::pose_to_matrix(tracker_pose);
    std::cout << "\n tracker_mat: " << std::endl;
    std::cout << tracker_mat << std::endl;

    // 标定出来的定位器坐标相对于机器人基座标的齐次变换矩阵
    Eigen::Matrix4d location2robotbase_mat;
    calibration_manager_->get_position_calibration_matrix(location2robotbase_mat);
    std::cout << "\n location2robotbase_mat:" << std::endl;
    std::cout << location2robotbase_mat << std::endl;

    // 求TCP在定位器坐标系的位置变化矩阵
    Eigen::Matrix4d tcp2location_pos_mat = tracker_mat * pos_matrix;
    // 求TCP在基坐标系下的位置变换矩阵
    Eigen::Matrix4d tcp2robotbase_pos_mat = location2robotbase_mat * tcp2location_pos_mat;
    std::cout << "\n tcp2location_pos_mat: " << std::endl;
    std::cout << tcp2location_pos_mat << std::endl;
    std::cout << "\n tcp2robotbase_pos_mat: " << std::endl;
    std::cout << tcp2robotbase_pos_mat << std::endl;
    Eigen::Matrix3d tcp2robotbase_ori_mat = tcp2robotbase_pos_mat.block<3, 3>(0, 0);
    std::cout << " tcp2robotbase_ori_mat:" << std::endl;
    std::cout << tcp2robotbase_ori_mat << std::endl;

    // 获取TCP姿态补偿矩阵
    Eigen::Matrix3d orientation_offset_matrix;
    calibration_manager_->get_orientation_offset_matrix(orientation_offset_matrix);
    std::cout << "used orientation_offset_matrix:" << std::endl;
    std::cout << orientation_offset_matrix << std::endl;
    // 进行补偿TCP姿态
    Eigen::Matrix3d tcp2robotbase_ori_mat_offset = orientation_offset_matrix * tcp2robotbase_ori_mat; //左乘
    Eigen::Matrix4d tcp2robotbase_mat = Eigen::Matrix4d::Identity();
    tcp2robotbase_mat.block<3, 3>(0, 0) = tcp2robotbase_ori_mat_offset;
    std::cout << " tcp2robotbase_ori_mat_offset:" << std::endl;
    std::cout << tcp2robotbase_ori_mat_offset << std::endl;
    tcp2robotbase_mat.block<3, 1>(0, 3) = tcp2robotbase_pos_mat.block<3, 1>(0, 3);
    std::cout << "\n tcp2robotbase_mat: " << std::endl;
    std::cout << tcp2robotbase_mat << std::endl;

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
void MainWindow::slot_end_record()
{
    std::cout << __FUNCTION__ << std::endl;
    emit signal_end_record();
    vive_tracker_reader_->disable_record();
    std::vector<CartesianPose> poses = vive_tracker_reader_->get_recorded_poses();
    std::vector<CartesianPose> poses_tcp2rb;
    
    // 逐个取出点位进行旋转乘得机器人基座下的点位，再存进文件
    for (auto iter : poses)
    {
        CartesianPose robot_pose;
        calibration_manager_->device_pose_to_robot_pose(iter.position, iter.orientation, robot_pose);
        poses_tcp2rb.push_back(robot_pose);
    }

    std::cout << "recorded poses size: " << poses.size() << std::endl;
    std::cout << "poses_tcp2rb size: " << poses_tcp2rb.size() << std::endl;

    vive_tracker_reader_->save_record_poses_to_file("vive_traj.csv", poses);
    vive_tracker_reader_->save_record_poses_to_file("vive_traj2robot.csv", poses_tcp2rb);

    csv_parser_window_vive->loadData("vive_traj.csv");
    csv_parser_window_vive->show();
    csv_parser_window_vive->plotData();
    csv_parser_window_vive->setWindowTitle("vive数据曲线");

    csv_parser_window_vive2robot->loadData("vive_traj2robot.csv");
    csv_parser_window_vive2robot->show();
    csv_parser_window_vive2robot->plotData();
    csv_parser_window_vive2robot->setWindowTitle("vive2robot数据曲线");

    vive_tracker_reader_->clear_recorded_poses();
    poses_tcp2rb.clear();

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
    std::cout << __FUNCTION__ << std::endl;
    double max_error = -1000.0;
    int rz = 0;
    std::vector<CartesianPosition> robot_calib_positions;
    std::vector<CartesianPosition> tracker_calib_positions;
    calibration_manager_->get_calibration_positions(robot_calib_positions, tracker_calib_positions);
    std::cout << "robot_calib_position.size(): " << robot_calib_positions.size() << " devie_calib_position.size(): " <<
                    tracker_calib_positions.size() << std::endl;
    for (int i = 1; i <= robot_calib_positions.size(); i++)
    {
        std::cout << "position[" << i << "] robot.x=" << robot_calib_positions[i].x << " robot.y=" << robot_calib_positions[i].y
                  << " robot.z=" << robot_calib_positions[i].z << " tracker.x=" << tracker_calib_positions[i].x
                  << " trakcer.y=" << tracker_calib_positions[i].y << " tracker.z=" << tracker_calib_positions[i].z << std::endl;
    }

    calibration_manager_->set_calibration_algorithm(2);
    int ret = calibration_manager_->calibrate(0, max_error);
    if (!ret)
    {
        ui->label_connect_state->setText("calib success");
    }
    else
    {
        ui->label_connect_state->setText("calib fail");
    }

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
        // 发送获取当前机器人点位的信号
        emit signal_mark_point(index);
        
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
        
        slot_mark_point_received(E_POSE_TYPE_VIVE, index, pose_tracker);
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

void MainWindow::slot_tracker2tcp_mark_point()
{
    std::cout << __FUNCTION__ << std::endl;
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
        // Eigen::Matrix4d calib_matrix;
        // tracker2tcp_calibration_->get_pose_calibration_matrix(calib_matrix);
        // std::cout << "tracker2tcp calib matrix:\n" << calib_matrix << std::endl;
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
    emit signal_flang2tcp_mark_point(size);
    //std::cout << " mark_poses.size():" << size << std::endl;
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

void MainWindow::slot_linear_error_cotinue_acquire()
{
    emit signal_linear_error_acquire();
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

    emit signal_handler_tracker2tcp_mark_rotation_use_robotpose();
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
        Eigen::Vector4d tcp2tracker_pos_vec(1.77239, -22.9953, 245.185, 1);
        tracker2tcp_calibration_->set_calibration_pos_vec(tcp2tracker_pos_vec);
    }
    else
    {
        std::cout << "use_track2tcp_ is_checked false" << std::endl;
        use_track2tcp_ = false;
    }
}

void MainWindow::slot_continue_get(bool state)
{
    if (ui->checkBox_continue_get->isChecked())
    {
        std::cout << "is_checked true" << std::endl;
        linear_error_continue_acquire_->start();
    }
    else
    {
        std::cout << "is_checked false" << std::endl;
        linear_error_continue_acquire_->stop();
    }
}
