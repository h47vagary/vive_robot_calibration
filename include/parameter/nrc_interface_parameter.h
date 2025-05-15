#ifndef INCLUDE_API_NRC_INTERFACE_PARAMETER_H_
#define INCLUDE_API_NRC_INTERFACE_PARAMETER_H_

#include <string>
//namespace cppinterface {
struct ToolParam {
    double X;                       //X轴偏移方向
    double Y;                       //Y轴偏移方向
    double Z;                       //Z轴偏移方向
    double A;                       //绕A轴旋转
    double B;                       //绕B轴旋转
    double C;                       //绕C轴旋转
    double payloadMass;             //负载质量
    double payloadInertia;          //负载惯性
    double payloadMassCenter_X;     //负载质心X
    double payloadMassCenter_Y;     //负载质心Y
    double payloadMassCenter_Z;     //负载质心Z
};

struct RobotDHParam
{
    double L1{0};
    double L2{0};
    double L3{0};
    double L4{0};
    double L5{0};
    double L6{0};
    double L7{0};
    double L8{0};
    double L9{0};
    double L10{0};
    double L11{0};
    double L12{0};
    double L13{0};
    double L14{0};
    double L15{0};
    double L16{0};
    double L17{0};
    double L18{0};
    double L19{0};
    double L20{0};

    std::string Couple_Coe_1_2;
    std::string Couple_Coe_2_3;
    std::string Couple_Coe_3_2;
    std::string Couple_Coe_3_4;
    std::string Couple_Coe_4_5;
    std::string Couple_Coe_4_6;
    std::string Couple_Coe_5_6;

    double dynamicLimit_max{0};
    double dynamicLimit_min{0};


    double pitch{0};//螺距
    double sliding_lead_value{0};//滑动电动缸导程,酒槽机型用
    double uplift_lead_value{0};//顶升电动缸导程,酒槽机型用
    double spray_distance{0};//喷料距离,酒槽机型用

    double threeAxisDirection{0};//3轴方向
    double fiveAxisDirection{0};//五轴方向

    double twoAxisConversionRatio{0};
    double threeAxisConversionRatio{0};
    double amplificationRatio;

    double conversionratio_x{0};
    double conversionratio_y{0};
    double conversionratio_z{0};

    double conversionratio_J1{0};  //1轴转换比 五轴混动
    double conversionratio_J2{0};
    double conversionratio_J3{0};

    int upsideDown{0};
};
//}
#endif /* INCLUDE_API_NRC_INTERFACE_PARAMETER_H_ */
