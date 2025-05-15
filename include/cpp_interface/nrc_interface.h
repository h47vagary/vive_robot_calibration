#ifndef INCLUDE_API_NRC_INTERFACE_H_
#define INCLUDE_API_NRC_INTERFACE_H_

#include "parameter/nrc_define.h"
#include "parameter/nrc_interface_parameter.h"
#include <string>
#include <vector>

//namespace cppinterface {

/**
 * @brief 获取库信息
 * @return 库版本相关信息
 */
EXPORT_API std::string get_library_version();

/**
 * @brief 连接控制器
 * @param ip 控制器ip,"192.168.1.13"
 * @param port 端口号,"6001"
 * @use 此函数是同步方式连接，因此函数会阻塞，直到返回连接结果。
 * @return -1-失败
 */
EXPORT_API SOCKETFD connect_robot(const std::string& ip,const std::string& port);

/**
 * @brief 断开控制器
 */
EXPORT_API Result disconnect_robot(SOCKETFD socketFd);

/**
 * @brief 获得控制器连接状态
 */
EXPORT_API Result get_connection_status(SOCKETFD socketFd);

/**
 * @brief 向控制器发送一个消息
 * @param messageId 消息id
 * @param message 消息内容
 */
EXPORT_API Result send_message(SOCKETFD socketFd,int messageID, const std::string& message);

/**
 * @brief 当收到控制器消息时，触发设置的回调函数，callback内不能做耗时操作，或阻塞。
 * @param callback(SOCKETFD socketFd,int messageID,std::string message) 收到的消息id，以及消息内容
 */
EXPORT_API Result recv_message(SOCKETFD socketFd,void(*callback)(int messageID,const char* message));

/**
 * @brief 设置接收错误或警告信息的回调函数。
 * @param fun 指向回调函数的指针。该回调函数在接收到错误或警告信息时被调用。
 * @note 回调函数由用户实现，用于处理接收到的错误信息。
 */
EXPORT_API Result set_receive_error_or_warnning_message_callback(SOCKETFD socketFd,void(*function) (int messageType, const char* message, int messageCode));

/**
* @brief 设置多机器人并行模式
* @param open true false
*/
EXPORT_API Result set_robots_parallel(SOCKETFD socketFd, bool open);

/**
* @brief 伺服清错
* @note 出错前如果时伺服运行状态，清错后需要手动进行下电操作，释放控制器的占用状态才可以继续上电（清错后不能直接上电，先下电再上电）
*/
EXPORT_API Result clear_error(SOCKETFD socketFd);
EXPORT_API Result clear_error_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 设置伺服状态
 * @param 0 停止 1 就绪
 * @warning 设置伺服就绪应该先确保系统没有错误 clear_servo_error(SOCKETFD socketFd)
 * 该函数只有伺服状态为0（停止状态）或1（就绪状态）时调用生效，伺服状态为2（报警状态）或3（运行状态）时不能直接设置伺服状态
 */
EXPORT_API Result set_servo_state(SOCKETFD socketFd,int state);
EXPORT_API Result set_servo_state_robot(SOCKETFD socketFd, int robotNum,int state);

/**
 * @brief 获取伺服状态
 * @param status 接收获取结果 0：停止状态 1：就绪状态 2：报警状态 3：运行状态
 */
EXPORT_API Result get_servo_state(SOCKETFD socketFd, int& status);
EXPORT_API Result get_servo_state_robot(SOCKETFD socketFd, int robotNum, int& status);

/**
 * @brief 机器人上电
 * @attention 调用该函数之前需要先调用set_servo_state(SOCKETFD socketFd,1)将伺服设置为1（就绪状态）
 * 			,机器人上电成功后调用 get_servo_state(SOCKETFD socketFd)为3伺服运行状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为1（就绪状态）时调用生效
 */
EXPORT_API Result set_servo_poweron(SOCKETFD socketFd);
EXPORT_API Result set_servo_poweron_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 机器人下电
 * @attention 机器人下电成功后调用 get_servo_state(SOCKETFD socketFd)为1伺服就绪状态
 * @return 机器人当前伺服状态servoStatus
 * 该函数只有伺服状态为3（运行状态）时调用生效
 */
EXPORT_API Result set_servo_poweroff(SOCKETFD socketFd);
EXPORT_API Result set_servo_poweroff_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 获取机器人当前位置
 * @param coord 入参 指定需要查询的坐标的坐标系
 * @param pos 出参 存储返回结果点位的容器，长度7
 */
EXPORT_API Result get_current_position(SOCKETFD socketFd, int coord, std::vector<double>& pos);
EXPORT_API Result get_current_position_robot(SOCKETFD socketFd, int robotNum, int coord, std::vector<double>& pos);

/**
 * @brief 获取机器人外部轴当前位置
 * @param pos 点位数组，长度5
 */
EXPORT_API Result get_current_extra_position(SOCKETFD socketFd, std::vector<double>& pos);
EXPORT_API Result get_current_extra_position_robot(SOCKETFD socketFd, int robotNum, std::vector<double>& pos);

/**
 * @brief 获取机器人运行状态
 * @param status 机器人运行状态
 *  - 0 停止
 *  - 1 暂停
 *  - 2 运行
 */
EXPORT_API Result get_robot_running_state(SOCKETFD socketFd, int& status);
EXPORT_API Result get_robot_running_state_robot(SOCKETFD socketFd, int robotNum, int& status);

/**
 * @brief 设置当前模式的速度 有三种模式 示教模式，运行模式，远程模式
 * @param speed 速度，参数范围：0<speed≤100
 */
EXPORT_API Result set_speed(SOCKETFD socketFd, int speed);
EXPORT_API Result set_speed_robot(SOCKETFD socketFd, int robotNum, int speed);

/**
 * @brief 获得当前模式的速度 有三种模式 示教模式，运行模式，远程模式
 * @param speed 速度，参数范围：0<speed≤100
 */
EXPORT_API Result get_speed(SOCKETFD socketFd, int& speed);
EXPORT_API Result get_speed_robot(SOCKETFD socketFd, int robotNum, int& speed);

/**
 * @brief 设置机器人当前坐标系
 * @param coord 坐标系 0：关节 1：直角 2：工具 3：用户
 */
EXPORT_API Result set_current_coord(SOCKETFD socketFd, int coord);
EXPORT_API Result set_current_coord_robot(SOCKETFD socketFd, int robotNum, int coord);

/**
 * @brief 获取机器人当前坐标系
 * @param coord 坐标系 0：关节 1：直角 2：工具 3：用户
 */
EXPORT_API Result get_current_coord(SOCKETFD socketFd, int& coord);
EXPORT_API Result get_current_coord_robot(SOCKETFD socketFd, int robotNum, int& coord);

/**
 * @brief 设置机器人当前模式
 * @param mode 模式 0：示教 1：远程 2：运行
 */
EXPORT_API Result set_current_mode(SOCKETFD socketFd, int mode);
EXPORT_API Result set_current_mode_robot(SOCKETFD socketFd, int robotNum, int mode);

/**
 * @brief 获取机器人当前模式
 * @param mode 当前模式 0：示教 1：远程 2：运行
 */
EXPORT_API Result get_current_mode(SOCKETFD socketFd,int& mode);
EXPORT_API Result get_current_mode_robot(SOCKETFD socketFd, int robotNum, int& mode);

/**
 * @brief 获取机器人类型
 */
EXPORT_API Result get_robot_type(SOCKETFD socketFd, RobotType& type);
EXPORT_API Result get_robot_type_robot(SOCKETFD socketFd, int robotNum, RobotType& type);

/**
 * @brief 设置示教模式类型
 * @param type 0 点动 1 拖拽
 */
EXPORT_API Result set_teach_type(SOCKETFD socketFd, int type);
EXPORT_API Result set_teach_type_robot(SOCKETFD socketFd, int robotNum, int type);

/**
 * @brief 获取示教模式下是何种类型
 * @param type 0 点动 1 拖拽
 */
EXPORT_API Result get_teach_type(SOCKETFD socketFd, int& type);
EXPORT_API Result get_teach_type_robot(SOCKETFD socketFd, int robotNum, int& type);

//=====================================工具手相关=================================================
/**
 * @brief 设置工具手编号
 * @param toolNum 工具手编号
 */
EXPORT_API Result set_tool_hand_number(SOCKETFD socketFd,int toolNum);
EXPORT_API Result set_tool_hand_number_robot(SOCKETFD socketFd, int robotNum, int toolNum);

/**
 * @brief 获取当前使用的工具手编号
 * @param toolNum 工具手编号
 */
EXPORT_API Result get_tool_hand_number(SOCKETFD socketFd,int& toolNum);
EXPORT_API Result get_tool_hand_number_robot(SOCKETFD socketFd, int robotNum, int& toolNum);

/**
 * @brief 设置工具手参数
 * @param toolNum 工具手编号
 * @param toolParam 要修改的参数 长度6 X轴偏移量 Y轴偏移量 Z轴偏移量 绕A轴旋转量 绕B轴旋转量 绕C轴旋转量
 */
EXPORT_API Result set_tool_hand_param(SOCKETFD socketFd,int toolNum,ToolParam param);
EXPORT_API Result set_tool_hand_param_robot(SOCKETFD socketFd, int robotNum, int toolNum,ToolParam param);

/**
 * @brief 获取当前工具手参数
 * @param toolNum 工具手编号
 */
EXPORT_API Result get_tool_hand_param(SOCKETFD socketFd,int toolNum,ToolParam& param);
EXPORT_API Result get_tool_hand_param_robot(SOCKETFD socketFd, int robotNum, int toolNum,ToolParam& param);

/**
 * @brief 2点标定/20点标定当前选择工具手 2/20点标定前使用 set_tool_hand_number 选择需要标定的工具手
 * @param point 标定点 1,2
 */
EXPORT_API Result tool_hand_2_or_20_point_calibrate(SOCKETFD socketFd, int point);
EXPORT_API Result tool_hand_2_or_20_point_calibrate_robot(SOCKETFD socketFd, int robotNum, int point);

/**
 * @brief 2点标定/20点标定 计算
 * @param noCalZero /true：校准⼯具尺⼨+姿态，不校准零点，false：其它情况，校准零点
 */
EXPORT_API Result tool_hand_2_or_20_point_calibrate_caculate(SOCKETFD socketFd, int calNum = 1, bool noCalZero = false);
EXPORT_API Result tool_hand_2_or_20_point_calibrate_caculate_robot(SOCKETFD socketFd, int robotNum, int calNum = 1, bool noCalZero = false);

/**
 * @brief 清除标定点
 * @param pointNum 清除工具手编号 1-20
 */
EXPORT_API Result tool_hand_2_or_20_point_calibrate_clear(SOCKETFD socketFd, int pointNum);
EXPORT_API Result tool_hand_2_or_20_point_calibrate_clear_robot(SOCKETFD socketFd, int robotNum, int pointNum);

/**
 * @brief 7点标定当前选择工具手 2/20点标定前使用 set_tool_hand_number 选择需要标定的工具手
 * @param point 标定点 1-7
 * @param toolNum 工具手
 */
EXPORT_API Result tool_hand_7_point_calibrate(SOCKETFD socketFd, int point, int toolNum);
EXPORT_API Result tool_hand_7_point_calibrate_robot(SOCKETFD socketFd, int robotNum, int point, int toolNum);

/**
 * @brief 7点标定 计算
 * @param toolNum 工具手编号
 * @param calibrationPointNum 标定点数量
 */
EXPORT_API Result tool_hand_7_point_calibrate_caculate(SOCKETFD socketFd, int toolNum, int calibrationPointNum = 7);
EXPORT_API Result tool_hand_7_point_calibrate_caculate_robot(SOCKETFD socketFd, int robotNum, int toolNum, int calibrationPointNum = 7);

/**
 * @brief 清除标定点
 * @param pointNum 清除标定点 1-7
 * @param toolNum 工具手编号
 */
EXPORT_API Result tool_hand_7_point_calibrate_clear(SOCKETFD socketFd, int pointNum,int toolNum);
EXPORT_API Result tool_hand_7_point_calibrate_clear_robot(SOCKETFD socketFd, int robotNum, int pointNum,int toolNum);

//======================================================================================
/**
 * @brief 设置用户坐标编号
 * @param userNum 用户坐标编号
 */
EXPORT_API Result set_user_coord_number(SOCKETFD socketFd,int userNum);
EXPORT_API Result set_user_coord_number_robot(SOCKETFD socketFd, int robotNum, int userNum);

/**
 * @brief 获取当前使用的用户坐标编号
 * @userNum 用户坐标编号
 */
EXPORT_API Result get_user_coord_number(SOCKETFD socketFd,int& userNum);
EXPORT_API Result get_user_coord_number_robot(SOCKETFD socketFd, int robotNum, int& userNum);

/**
 * @brief 获取当前使用的用户坐标的参数
 * @param userNum 用户坐标编号
 * @param pos 用户坐标参数
 */
EXPORT_API Result get_user_coord_para(SOCKETFD socketFd,int userNum,std::vector<double>& pos);
EXPORT_API Result get_user_coord_para_robot(SOCKETFD socketFd, int robotNum, int userNum,std::vector<double>& pos);

/**
 * @brief 标定用户坐标
 * @param userNum 用户坐标编号
 * @param pos 坐标数据
 */
EXPORT_API Result set_user_coordinate_data(SOCKETFD socketFd,int userNum, std::vector<double> pos);
EXPORT_API Result set_user_coordinate_data_robot(SOCKETFD socketFd, int robotNum, int userNum, std::vector<double> pos);

/**
 * @brief 标定OXY
 * @param userNum 用户坐标编号
 * @param xyo 值 'X' 'Y' 'O'
 */
EXPORT_API Result calibration_oxy(SOCKETFD socketFd, int userNum, std::string xyo);
EXPORT_API Result calibration_oxy_robot(SOCKETFD socketFd, int robotNum, int userNum, std::string xyo);

/**
 * @brief 计算坐标
 * @param userNum 用户坐标编号
 */
EXPORT_API Result calculate_user_coordinate(SOCKETFD socketFd, int userNumber);
EXPORT_API Result calculate_user_coordinate_robot(SOCKETFD socketFd, int robotNum, int userNum);

//==============================================================================
/**
* @brief 设置全局GP点位
* @param posName 需要修改全局位置名 例如 "GP0001"
* @param posInfo[14] [0]坐标系 0：关节 1：直角 2：工具 3：用户  [1]:0 角度制 1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
*/
EXPORT_API Result set_global_position(SOCKETFD socketFd, std::string posName, std::vector<double> posInfo);
EXPORT_API Result set_global_position_robot(SOCKETFD socketFd, int robotNum, std::string posName, std::vector<double> posInfo);

/**
 * @brief 查询全局GP点位
 * @param posName 全局位置名 例如 "GP0001"
 * @param pos 全局点位数组 长度14 前7位为点位的坐标、姿态等信息，后7位为机器人位置
 */
EXPORT_API Result get_global_position(SOCKETFD socketFd, std::string posName, std::vector<double>& pos);
EXPORT_API Result get_global_position_robot(SOCKETFD socketFd, int robotNum, std::string posName, std::vector<double>& pos);

/**
* @brief 设置全局GE点位
* @param posName 需要修改全局位置名 例如 "GE0001"
* @param posInfo[21] [0]坐标系 0：关节 1：直角 2：工具 3：用户  [1]:0 角度制 1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用
*                  [7-13] 机器人本体点位信息  [14-20]  外部轴点位信息
*/
EXPORT_API Result set_global_sync_position(SOCKETFD socketFd, const std::string& posName, std::vector<double> posInfo);
EXPORT_API Result set_global_sync_position_robot(SOCKETFD socketFd, int robotNum, const std::string& posName, std::vector<double> posInfo);

/**
 * @brief 查询全局GE点位
 * @param posName 全局位置名 例如 "GE0001"
 * @param pos 全局点位数组 长度21 前7位为点位的坐标、姿态等信息，中间7位为机器人位置 后7位为外部轴位置
 */
EXPORT_API Result get_global_sync_position(SOCKETFD socketFd, const std::string& posName, std::vector<double>& pos);
EXPORT_API Result get_global_sync_position_robot(SOCKETFD socketFd, int robotNum, const std::string& posName, std::vector<double>& pos);

/**
 * @brief 设置全局变量
 * @param varName 全局变量名 例如 "GI001" "GD001" "GB001"
 * @param varValue 变量值
 */
EXPORT_API Result set_global_variant(SOCKETFD socketFd, const std::string& varName, double varValue);
EXPORT_API Result set_global_variant_robot(SOCKETFD socketFd, int robotNum, const std::string& varName, double varValue);

/**
 * @brief 查询全局变量
 * @param varName 全局变量名 例如 "GI001" "GD001" "GB001"
 */
EXPORT_API Result get_global_variant(SOCKETFD socketFd, const std::string& varName, double& vaule);
EXPORT_API Result get_global_variant_robot(SOCKETFD socketFd, int robotNum, const std::string& varName, double& vaule);

/**
 * @brief 设置零点位置
 * @param axis 轴号
 * @return
 */
EXPORT_API Result set_axis_zero_position(SOCKETFD socketFd, int axis);
EXPORT_API Result set_axis_zero_position_robot(SOCKETFD socketFd, int robotNum, int axis);

/**
 * @brief 设置零点偏移
 * @param axis 需要偏移的轴数
 * @param shift 偏移量 -360° < shift < 360°
 */
EXPORT_API Result set_zero_pos_deviation(SOCKETFD socketFd,int axis, double shift);
EXPORT_API Result set_zero_pos_deviation_robot(SOCKETFD socketFd, int robotNum, int axis, double shift);

/**
 * @brief 获取单圈值
 * @param single_cycle 储存单圈值数组 长度7
 */
EXPORT_API Result get_single_cycle(SOCKETFD socketFd, std::vector<double>& single_cycle);
EXPORT_API Result get_single_cycle_robot(SOCKETFD socketFd, int robotNum, std::vector<double>& single_cycle);

/**
 * @brief 查询4点标定
 * @param result
 */
EXPORT_API Result get_four_point(SOCKETFD socketFd, std::vector<double>& result);
EXPORT_API Result get_four_point_robot(SOCKETFD socketFd, int robotNum, std::vector<double>& result);

/**
 * @brief 进行4点标记
 * @param point 标记点位编号 取值范围 0 - 3
 * @param status 标记状态 0：取消标记 1：标记
 */
EXPORT_API Result set_four_point_mark(SOCKETFD socketFd, int point, int status);
EXPORT_API Result set_four_point_mark_robot(SOCKETFD socketFd, int robotNum, int point, int status);

/**
 * @brief 4点标定计算
 * @param L1
 * @param L2
 * @param result
 */
EXPORT_API Result four_point_calculation(SOCKETFD socketFd, double L1, double L2, std::vector<double>& result);
EXPORT_API Result four_point_calculation_robot(SOCKETFD socketFd, int robotNum, double L1, double L2, std::vector<double>& result);

/**
 * @brief 将4点标定计算的结果写入机器人DH参数
 * @param 写入是否成功：成功/失败（1/0）
 * @return
 */
EXPORT_API Result set_result_for_DH(SOCKETFD socketFd, int& apply);
EXPORT_API Result set_result_for_DH_robot(SOCKETFD socketFd, int robotNum, int& apply);

/**
 * @brief 原坐标值转换为其他坐标值
 * @param originCoord 原坐标系    0 1 2 3 关节 直角 工具 用户
 * @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
 *        关节取值范围    0-6[-10000,10000]
 *        直角取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        工具取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 *        用户取值范围    0-2[-10000,10000] 3-6[-3.1416,3.1416]rad
 * @param targetCoord 目标坐标系  0 1 2 3 关节 直角 工具 用户
 * @param targetPos 转换后的坐标系
 */
EXPORT_API Result get_origin_coord_to_target_coord(SOCKETFD socketFd, int originCoord, std::vector<double> originPos, int targetCoord,std::vector<double>& targetPos);
EXPORT_API Result get_origin_coord_to_target_coord_robot(SOCKETFD socketFd, int robotNum, int originCoord, std::vector<double> originPos, int targetCoord,std::vector<double>& targetPos);

/**
 * @brief 获取当前机器人DH参数
 * @param param 结构体参数
*/
EXPORT_API Result get_robot_dh_param(SOCKETFD socketFd, RobotDHParam& param);
EXPORT_API Result get_robot_dh_param_robot(SOCKETFD socketFd, int robotNum, RobotDHParam& param);

/**
 * @brief 获取当前机器人关节参数
 * @param id 关节参数[1,6]
 * @param param 结构体参数
 */
EXPORT_API Result get_robot_joint_param(SOCKETFD socketFd, int id, RobotJointParam& param);
EXPORT_API Result get_robot_joint_param_robot(SOCKETFD socketFd, int robotNum, int id, RobotJointParam& param);

/**
 * @brief 查询示教盒的连接状态
 * @param connected 接收连接状态
 */
EXPORT_API Result get_teachbox_connection_status(SOCKETFD socketFd, bool &connected);
EXPORT_API Result get_teachbox_connection_status_robot(SOCKETFD socketFd, int robotNum, bool &connected);

/**
 * @brief 获取控制器序列号ID
 * @param id 返回控制器序列号ID
 */
EXPORT_API Result get_controller_id(SOCKETFD socketFd, char* id);
EXPORT_API Result get_controller_id_robot(SOCKETFD socketFd, int robotNum, char* id);

/**
 * @brief 获取控制器序列号ID
 * @param id 返回控制器序列号ID
 */
EXPORT_API Result get_controller_id_csharp(SOCKETFD socketFd, std::vector<char>& id);
EXPORT_API Result get_controller_id_csharp_robot(SOCKETFD socketFd, int robotNum, std::vector<char>& id);

/**
 * @brief 获取六维力传感器的数据
 */
EXPORT_API Result get_sensor_data(SOCKETFD socketFd, std::vector<int>& data);
EXPORT_API Result get_sensor_data_robot(SOCKETFD socketFd, int robotNum, std::vector<int>& data);

/**
 * @brief 获取静态寻位坐标
 * @param fileid 寻位文件号
 * @param tableid 寻位参数表号
 * @param delaytime 参数表延时
 */
EXPORT_API Result get_static_search_position(SOCKETFD socketFd, int fileid, int tableid, int delaytime, std::vector<double>& pos);
EXPORT_API Result get_static_search_position_robot(SOCKETFD socketFd,int robotNum, int fileid, int tableid, int delaytime, std::vector<double>& pos);

/**
 * @brief 获取当前电机扭矩
 * @param motorTorque 机器人扭矩 长度7 单位[%]
 * @param motorTorqueSync 外部轴扭矩 长度5 单位[%]
 */
EXPORT_API Result get_curretn_motor_torque(SOCKETFD socketFd, std::vector<int>& motorTorque, std::vector<int>& motorTorqueSync);
EXPORT_API Result get_curretn_motor_torque_robot(SOCKETFD socketFd,int robotNum, std::vector<int>& motorTorque, std::vector<int>& motorTorqueSync);

/**
 * @brief 获取当前电机转速
 * @param motorSpeed 机器人电机转速 长度7 单位[RPM]
 * @param motorSpeedSync 外部轴电机转速 长度5 单位[RPM]
 */
EXPORT_API Result get_curretn_motor_speed(SOCKETFD socketFd, std::vector<int>& motorSpeed, std::vector<int>& motorSpeedSync);
EXPORT_API Result get_curretn_motor_speed_robot(SOCKETFD socketFd,int robotNum, std::vector<int>& motorSpeed, std::vector<int>& motorSpeedSync);

/**
 * @brief 获取当前电机负载
 * @param motorPayload 机器人电机转速 长度7 单位[%]
 * @param motorPayloadSync 外部轴电机转速 长度5 单位[%]
 */
EXPORT_API Result get_curretn_motor_payload(SOCKETFD socketFd, std::vector<double>& motorPayload, std::vector<double>& motorPayloadSync);
EXPORT_API Result get_curretn_motor_payload_robot(SOCKETFD socketFd,int robotNum, std::vector<double>& motorPayload, std::vector<double>& motorPayloadSync);

/**
 * @brief 获取当前末端线速度和轴速度
 * @param lineSpeed 末端线速度 单位[mm/s]
 * @param jointSpeed 关节速度 长度5 单位[度/s]
 * @param jointSpeedSync 外部轴关节速度 长度5 单位[度/s]
 */
EXPORT_API Result get_curretn_line_speed_and_joint_speed(SOCKETFD socketFd, double& lineSpeed, std::vector<double>& jointSpeed, std::vector<double>& jointSpeedSync);
EXPORT_API Result get_curretn_line_speed_and_joint_speed_robot(SOCKETFD socketFd,int robotNum, double& lineSpeed, std::vector<double>& jointSpeed, std::vector<double>& jointSpeedSync);

//=============================motion=========================
/**
 * @brief 开始点动
 * @param axis 轴号
 * @param dir 方向
 */
EXPORT_API Result robot_start_jogging(SOCKETFD socketFd, int axis, bool dir);
EXPORT_API Result robot_start_jogging_robot(SOCKETFD socketFd, int robotNum, int axis, bool dir);

/**
 * @brief 停止点动
 * @param axis 轴号
 */
EXPORT_API Result robot_stop_jogging(SOCKETFD socketFd, int axis);
EXPORT_API Result robot_stop_jogging_robot(SOCKETFD socketFd, int robotNum, int axis);

/**
 * @brief 回到设定的复位点
 */
EXPORT_API Result robot_go_to_reset_position(SOCKETFD socketFd);
EXPORT_API Result robot_go_to_reset_position_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 回到设定的零点
 */
EXPORT_API Result robot_go_home(SOCKETFD socketFd);
EXPORT_API Result robot_go_home_robot(SOCKETFD socketFd, int robotNum);

/**
 * @brief 关节运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤100 单位 %
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 */
EXPORT_API Result robot_movej(SOCKETFD socketFd,MoveCmd moveCmd);
EXPORT_API Result robot_movej_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
 * @brief 直线运动
 * @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤1000 单位mm/s
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 * @param isSync 是否同步模式 true同步 false不同步
 */
EXPORT_API Result robot_movel(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result robot_movel_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
 * @brief 圆弧运动
 * @param pos1 点位数组，长度为7，n个轴就赋值前n位数组,其余置0
 * @param pos2 点位数组，长度为7，n个轴就赋值前n位数组,其余置0
 * @param pos3 点位数组，长度为7，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤100
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param acc 减速度，参数范围：0<dec≤100
 */
EXPORT_API Result robot_movec(SOCKETFD socketFd, std::vector<double> pos1, std::vector<double> pos2, std::vector<double> pos3, int vel, int coord, int acc, int dec);
EXPORT_API Result robot_movec_robot(SOCKETFD socketFd, int robotNum, std::vector<double> pos1, std::vector<double> pos2, std::vector<double> pos3,int vel,int coord,int acc, int dec);

/**
 * @brief 整圆运动
 * @param pos1 点位数组，长度为7，n个轴就赋值前n位数组,其余置0
 * @param pos2 点位数组，长度为7，n个轴就赋值前n位数组,其余置0
 * @param pos3 点位数组，长度为7，n个轴就赋值前n位数组,其余置0
 * @param vel 速度，参数范围：0<vel≤100
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param acc 减速度，参数范围：0<dec≤100
 */
EXPORT_API Result robot_moveca(SOCKETFD socketFd, std::vector<double> pos1, std::vector<double> pos2, std::vector<double> pos3, int vel, int coord, int acc, int dec);
EXPORT_API Result robot_moveca_robot(SOCKETFD socketFd, int robotNum, std::vector<double> pos1, std::vector<double> pos2, std::vector<double> pos3, int vel, int coord, int acc, int dec);

/**
 * @brief 样条曲线运动
 * @param pos 点位二维数组，一维长度代表点位个数，最少需要4个，二维长度最大为7，n轴就赋值前n位数组
 * @param vel 速度，参数范围：0<vel≤100
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 */
EXPORT_API Result robot_moves(SOCKETFD socketFd, std::vector<std::vector<double>> pos, int vel, int coord, int acc, int dec);
EXPORT_API Result robot_moves_robot(SOCKETFD socketFd, int robotNum, std::vector<std::vector<double>> pos, int vel, int coord, int acc, int dec);

/**
 * @brief 外部轴关节运动
 * @param pos 点位数组，长度14 前7位为机器人本体点位，后7位为外部轴点位，几轴就填几位，其余置0 外部轴从pos[7]开始
 * @param vel 速度，参数范围：0<vel≤100
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 */
EXPORT_API Result robot_extra_movej(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result robot_extra_movej_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
 * @brief 外部轴直线运动
 * @param pos 点位数组，长度14 前7位为机器人本体点位，后7位为外部轴点位，几轴就填几位，其余置0 外部轴从pos[7]开始
 * @param vel 速度，参数范围：1<vel≤9999
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 */
EXPORT_API Result robot_extra_movel(SOCKETFD socketFd, MoveCmd moveCmd);
EXPORT_API Result robot_extra_movel_robot(SOCKETFD socketFd, int robotNum, MoveCmd moveCmd);

/**
 * @brief 外部轴圆弧运动
 * @param pos1 点位数组，长度14 前7位为机器人本体点位，后7位为外部轴点位，几轴就填几位，其余置0
 * @param pos2 点位数组，长度14 前7位为机器人本体点位，后7位为外部轴点位，几轴就填几位，其余置0
 * @param pos3 点位数组，长度14 前7位为机器人本体点位，后7位为外部轴点位，几轴就填几位，其余置0
 * @param vel 速度，参数范围：1<vel≤9999
 * @param coord 坐标系，参数范围：0≤coord≤3
 * @param acc 加速度，参数范围：0<acc≤100
 * @param dec 减速度，参数范围：0<dec≤100
 */
EXPORT_API Result robot_extra_movec(SOCKETFD socketFd, std::vector<double> pos1, std::vector<double> pos2, std::vector<double> pos3, int vel, int coord, int acc, int dec);
EXPORT_API Result robot_extra_movec_robot(SOCKETFD socketFd, int robotNum, std::vector<double> pos1, std::vector<double> pos2, std::vector<double> pos3, int vel, int coord, int acc, int dec);

/**
 * @brief 外部点移动
 * 需要连接7000端口
	SOCKETFD fd7000 = connect_robot("192.168.1.13","7000");
	servo_move(fd7000, servoMove);
 *
 */
EXPORT_API Result servo_move(SOCKETFD socketFd, ServoMovePara servoMove);
EXPORT_API Result servo_move_robot(SOCKETFD socketFd, int robotNum, ServoMovePara servoMove);

//}
#endif /* INCLUDE_API_NRC_INTERFACE_H_ */
