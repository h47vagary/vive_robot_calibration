#ifndef INCLUDE_API_NRC_INTERFACE_DEFINE_H_
#define INCLUDE_API_NRC_INTERFACE_DEFINE_H_

#include <string>
#include <vector>

#if	defined(_WIN32) || defined(WIN32)   //windows
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

const int const_robotNum = 1;
typedef int SOCKETFD;
//namespace cppinterface {
enum RobotType {
    NOTYPE = 0,
    SIX_AXLE_GENERAL = 1,
    FOUR_AXLE_SCARA,
    FOUR_AXLE_STACK,
    FOUR_AXLE_GENERAL,
    ONE_AXLE_GENERAL,
    FIVE_AXLE_GENERAL,
    SIX_AXLE_ONE_GENERAL,
    TWO_AXLE_SCARA,
    THREE_AXLE_SCARA,
    THREE_AXLE_ANGLE,
    THREE_AXLE_ABNORMITY,
    SEVEN_AXLE_GENERAL,
    FOUR_AXLE_SCARA_ABNORMITY,
    FOUR_AXLE_PALLET_1,
    SIX_AXLE_SPRAY,
    FOUR_AXLE_ANGLE,
    FOUR_AXLE_POLAR_ABNORMITY,
    SIX_AXLE_ABNORMITY,
    GANTRY_WELD,
    R_DELTA,
    R_WINE_CHAMFER,
    R_GANTRY_WELD_2_,
    R_GENERAL_5S_COLLABORATIVE_,
    FOUR_AXLE_SCARA_ABNORMITY_3_,
};

//enum RobotType {
//    NOTYPE = 0, // 默认值
//    SIX_AXLE_GENERAL = 1,
//    FOUR_AXLE_SCARA,
//    FOUR_AXLE_STACK,
//    FOUR_AXLE_GENERAL,
//    ONE_AXLE_GENERAL,
//    FIVE_AXLE_GENERAL,
//    SIX_AXLE_ONE_GENERAL,
//    TWO_AXLE_SCARA,
//    THREE_AXLE_SCARA,
//    THREE_AXLE_ANGLE,
//    THREE_AXLE_ABNORMITY,
//    SEVEN_AXLE_GENERAL,
//    FOUR_AXLE_SCARA_ABNORMITY,
//    FOUR_AXLE_PALLET_1,
//    SIX_AXLE_SPRAY,
//    FOUR_AXLE_ANGLE,
//    FOUR_AXLE_POLAR_ABNORMITY,
//    SIX_AXLE_ABNORMITY,
//    GANTRY_WELD,
//    R_DELTA,
//    R_WINE_CHAMFER,
//    R_GANTRY_WELD_2,
//    R_FOUR_CARTESIAN_COORDINATE_1,
//    R_GANTRY_WELD_6,
//    FIVE_AXLE_MIXED,
//    R_SCARA_FOURAXIS_2,
//    SIX_AXLE_ABNORMITY_3,
//    R_SCARA_THREEAXIS_1,
//    R_DELTA_2D_,
//    R_GANTRY_WELD_3_,
//    R_GENERAL_3S_1_,
//    R_GENERAL_5S_COLLABORATIVE_,
//    FOUR_AXLE_SCARA_ABNORMITY_3_,
//    R_GENERAL_6S_CBBARA_,
//    R_HEAVY_DUTY_FOUR_AXIS_,
//};

enum Result {
    EXCEPTION = -5,
    OPERATION_NOT_ALLOWED = -4,
    PARAM_ERR = -3,
    DISCONNECT = -2,
    RECEIVE_FAILED = -1,
    SUCCESS = 0
};

//NRC系统变量值类型
enum class ParaType {
	data = 0 ,//数值型
	varINT,  //"I00xx"类型
	varDOUBLE,	//"D00xx"类型
	varBOOL,	//"B00xx"类型
	varGINT,	//"GI00xx"类型
	varGDOUBLE,	//"GD00xx"类型
	varGBOOL,	//"GB00xx"类型
};

//NRC系统变量
struct Variant {
	ParaType varType = ParaType::data; //值类型
	double varValue = 0;  //valueType类型为Data_自定义时,需要自己填值
	std::string varName = "I001";	//valueType类型不是Data 时的参数值
	Variant(ParaType type, double data,std::string varname) : varType(type),varValue(data),varName(varname){};
	void setData(ParaType type, double data,std::string varname = "") {
		varType = type;
		varValue = data;
		varName = varname;
	}
};

enum class PosType {
	data = 0,    //自定义数组
    PType = 1,
    E_TYPE = 2,
    RP_TYPE = 3,
    AP_TYPE = 4,
	GPType = 5,  //系统内置的全局点位 GP001 只含机器人本体的点位
	GEType = 6   //系统内置的全局点位 GE001 含外部轴的机器人点位
};

//移动指令参数  单独的Movj只能使用数值   作业文件只支持使用变量   
struct MoveCmd {
	PosType targetPosType;
	std::vector<double> targetPosValue; //如果posType=PosType::data 为自定义数组,需要设置该向量值,前7位为本体值，后7位为外部轴
	std::string targetPosName; //如果posType为内置点位,需要设置该值,如 posType=PosType::GPType;posName=“GP0001”;
    int coord;
	double velocity;
	double acc;
	double dec;
	int pl;
	int time;
	int toolNum;
	int userNum;
    int posidtype; //0:P GP 一级变量类型 1:P[I001] GP[i001] 二级变量类型
    int configuration; //形态
	MoveCmd() : targetPosType(PosType::data),targetPosValue(7),targetPosName("GP0001")
	,coord(0),velocity(50),acc(50),dec(50),pl(0),time(0),toolNum(0),userNum(0),posidtype(0),configuration(0){ }
};

struct ServoMovePara {
	bool clearBuffer; ///< 是否清除之前发送的，未开始插补计算的点位
	int targetMode;   ///< 0-独立点 1-连续轨迹
	int sendMode;     ///< 0-一次传输完全部轨迹 1-一次传输部分点位
	int runMode;      ///< 0-接收完再运动 1-边接受边运动
	int sum;          ///< 总传输次数
	int count;        ///< 当前是第几次

	int coord;        ///< 0-关节 1-直角
	int extMove;        ///< 0- 1-
	int size;         ///< 本次传输的点位数
	std::vector<std::vector<double>> pos; ///< 二维数组，一维表示本次传输的点位数，二维长度为7，各个关节角度或笛卡尔坐标
	std::vector<double> timeStamp;  ///< 长度为本次传输的点位数，表示到达该点位的时间，单位ms
};

struct RobotJointParam {
    double reducRatio;          //减速比
    int encoderResolution;      //编码器位数
    double posSWLimit;          //轴正限位
    double negSWLimit;          //轴反限位
    double ratedRotSpeed;       //电机额定正转速
    double ratedDeRotSpeed;     //电机额定反转速
    double maxRotSpeed;         //电机最大正转速
    double maxDeRotSpeed;       //电机最大反转速
    double ratedVel;            //额定正速度
    double deRatedVel;          //额定反速度
    double maxAcc;              //最大加速度
    double maxDecel;            //最大减速度
    int direction;              //模型方向，1：正向，-1：反向
};

enum LogicType
{
	EQUAL_TO = 1,   //  ==
	LESS,           //  <
	GREATER,        //  >
	LESS_EQUAL,     //  <=
	GREATER_EQUAL,  //  >=
	NOT_EQUAL_TO  
};
struct ParaGroup {
	double data; //数值型给这个赋值
	int secondvalue;  //变量型是否存在二级变量 1存在 0不存在 例如 "DOUT[I001]" 这种填1  
	int value; //0 数值型  1变量型
	std::string varname; //变量型给这个写变量名 例如 "I001" , "DOUT[I001]"
    ParaGroup()
    : data(0.0), secondvalue(0), value(0), varname("") {}
};

struct Condition {
    std::string desValue;
    std::string key;
	LogicType logicType;
	ParaGroup paraGroupOne;
	ParaGroup paraGroupTwo;
	Condition()
	: desValue("0"), key("I001"), logicType(EQUAL_TO), paraGroupOne(), paraGroupTwo(){}
};

struct IOCommandParams {
    std::string groupType;    // 设置输出路数 可选路数 OT#:1路输出 OGH#:4路输出 OG#：8路输出
    int errorHanding;         // 0:输出值保存 1:计时结束停止
    ParaGroup paraGroupNum;   // 设置输出IO版和输出组号 选择OT# 范围[1,16] 选择OGH# 范围[1,16] 选择OG# 范围[1,8]
    ParaGroup paraGroupTime;  // 时间 [0,9999]S
    ParaGroup paraGroupValue; // 输出值 选择OT# 端口范围1 选择OGH# 端口范围[1,4] 选择OG# 范围[1,8]
                              // 勾选端口1:1 端口2:2 端口3:4 端口4:8 端口5:16 端口6:32 端口7:64 端口8:128
                              // 例如勾选端口1和端将口2和端口3 输入 :7
};

struct AlarmdIO {
    int msgType;            // 消息类型 0：普通消息，1：警告消息，2：错误消息, 22.07没有此参数，全部设为 0 即可
    int value;              // IO有效参数设置(0/1)
    int enable;             // 使能设置(0/1)
    std::string msg;        // 消息内容
};

struct RemoteProgram {
    int port;       // 远程程序端口绑定
    int value;      // 使用远程IO功能时有效参数(0/1),使用远程状态提示功能时有效参数(0/1/2)
};

struct RemoteControl {
    int clearStashPort;     // 清除断电保持数据绑定端口
    int faultResetPort;     // 清除报警端口
    int pausePort;          // 暂停端口
    int startPort;          // 启动端口
    int stopPort;           // 停止端口

    int clearStashValue;    // 清除断电保持数据端口的有效参数(0/1),与clearStashPort相对应
    int faultResetValue;    // 清除报警端口的有效参数(0/1),与faultResetPort相对应
    int pauseValue;         // 暂停端口的有效参数(0/1),与pausePort相对应
    int startValue;         // 启动端口的有效参数(0/1),与startPort相对应
    int stopValue;          // 停止端口的有效参数(0/1),与stopPort相对应

    std::vector<RemoteProgram> program; // 远程程序端口设置,详见 RemoteProgram
};

struct RemoteProgramSetting {
    std::string job;        // 远程程序选择
    int times;              // 远程程序运行次数
};

struct PositionData
{
   std::string key;//变量名  AP0001 
   std::vector<double> posData;  //坐标数据 长度7
   int coord;  // 0关节  1直角  2工具  3用户 
   int toolNum;
   int userNum;
   PosType type; // 参考PosType根据变量名选择 AP是4

    PositionData()
    : key(""), coord(0), toolNum(0), userNum(0), type(PosType::data)
    {
    }
   PositionData(const std::string& key, const std::vector<double>& posData, int coord, int toolNum, int userNum)
    : key(key), posData(posData), coord(coord), toolNum(toolNum), userNum(userNum), type(determineType(key))
    {
    }
private:
    static PosType determineType(const std::string& key)
    {
        if (key.substr(0, 1) == "P") {
            return PosType::PType;
        }
        return PosType::data; 
    }
};

struct SafeIO {
    int quickStopPort1;     // 紧急停止端口1
    int quickStopPort2;     // 紧急停止端口2
    int quickStopValue1;    // 紧急停止参数1(0/1)
    int quickStopValue2;    // 紧急停止参数2(0/1)
    bool quickStopEnable;   // 紧急停止使能
    bool quickStopShied1;   // 屏蔽紧急停止1
    bool quickStopShied2;   // 屏蔽紧急停止2
    double quickStopTime;   // 快速停止时间，单位 毫秒(ms) 范围 [50,100]
    int quickStopShiedTime; // 屏蔽紧急停止时间， 单位 秒(s)

    int screenPort1;        // 安全光幕端口1
    int screenPort2;        // 安全光幕端口2
    int screenValue1;       // 安全光幕参数1(0/1)
    int screenValue2;       // 安全光幕参数2(0/1)
    bool screenEnable;      // 安全光幕使能
};

struct ErrorMessageModel
{
    int messageType;
    std::string message;
    int messageCode;
};

struct OffsetCommandParam
{
    std::vector<std::string> calData {7, "0"};//轴参数 手填值范围[-50000，50000] 
    std::string coord {"BF"}; //位置变量坐标系 RF-关节坐标 BF-直角坐标 UF用户坐标
    std::vector<int> datatype {0,0,0,0,0,0,0};  //变量类型 0-手填值 1 变量类型  ：例如 caldata是["1","2","3","I001","D001","GI001","GD001"]  这个datatype就是 [0,0,0,1,1,1,1]
    int tool {-1};
    int user {-1};
};
//}
#endif /* INCLUDE_API_NRC_INTERFACE_DEFINE_H_ */



