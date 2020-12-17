//Flight Controller Example
//1 Nov 2020
//M. Wahbah
// This an example node that shows the most basic flight controller for a drone.
// It requires the global2inertial node and providers_node to provide required topics (pos and att measuerments)
// This node has no controller settings - these should be set using the flight scenario
#include <iostream>
#include <vector>
#include <pthread.h>
#include <sched.h>

#include "HEAR_core/std_logger.hpp"
#include "HEAR_core/Switch.hpp"
#include "HEAR_core/Mux3D.hpp"
#include "HEAR_core/Demux3D.hpp"
#include "HEAR_core/InvertedSwitch.hpp"
#include "HEAR_math/Differentiator.hpp"
#include "HEAR_math/Sum.hpp"
#include "HEAR_math/Saturation.hpp"
#include "HEAR_math/ConstantFloat.hpp"
#include "HEAR_math/KalmanFilter.hpp"
#include "HEAR_control/PIDController.hpp"
#include "HEAR_control/MRFTController.hpp"
#include "HEAR_control/BoundingBoxController.hpp"
#include "HEAR_actuation/HexaActuationSystem.hpp"
#include "HEAR_nav/WrapAroundFunction.hpp"
#include "HEAR_nav/Global2Inertial.hpp"
#include "HEAR_nav/RestrictedNormWaypointRefGenerator.hpp"
#include "HEAR_nav/Transform_InertialToBody.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Optitrack.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_UpdateControllerSrv.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_BroadcastData.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_IMU.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_RestNormSettings.hpp"
#include "HEAR_ROS_BRIDGE//ROSUnit_Factory.hpp"
#include "HEAR_NAVIO_Interface/ESCMotor.hpp"
#include "HEAR_NAVIO_Interface/BatteryMonitor.hpp"

#define XSENS_OVER_ROS
#define OPTITRACK
#define BIG_HEXA
#undef BATTERY_MONITOR

const int PWM_FREQUENCY = 200;
const float SATURATION_VALUE_XY = 0.2617; 
const float SATURATION_VALUE_YAW = 0.2617;
const float SATURATION_VALUE_YAWRATE = 0.3;

int main(int argc, char** argv) {
    std::cout << "Hello Flight Controller!" << std::endl;
    // //*****************************LOGGER********************************** 
    Logger::assignLogger(new StdLogger());
    // //****************************ROS UNITS*******************************
    ros::init(argc, argv, "flight_controller_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit_Factory ROSUnit_Factory_main{nh};
    ROSUnit* myROSUpdateController = new ROSUnit_UpdateControllerSrv(nh);
    ROSUnit* myROSBroadcastData = new ROSUnit_BroadcastData(nh);

    ROSUnit* myROSArm = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                            ROSUnit_msg_type::ROSUnit_Bool,
                                                            "arm"); 
    ROSUnit* myROSResetController = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Int8,
                                                                      "reset_controller");
    ROSUnit* ros_optitrack_mrft_switch_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "optitrack_mrft_switch_y");
    ROSUnit* ros_camera_mrft_switch_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "camera_mrft_switch_y");
    ROSUnit* ros_camera_pid_switch_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "camera_pid_switch_y");
    ROSUnit* ros_optitrack_mrft_switch_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "optitrack_mrft_switch_z");
    ROSUnit* ros_camera_mrft_switch_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "camera_mrft_switch_z");
    ROSUnit* ros_camera_pid_switch_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "camera_pid_switch_z");
    ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_y_camera_provider_sub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/camera/y");
    ROSUnit* rosunit_z_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_z_camera_provider_sub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/camera/z");
    ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");
    ROSUnit* rosunit_waypoint_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/x");
    ROSUnit* rosunit_waypoint_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/y");
    ROSUnit* rosunit_waypoint_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/z");
    ROSUnit* rosunit_waypoint_yaw = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/yaw");
    // ROSUnit* check_output1 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "output_provider/pid");
    // ROSUnit* check_output2 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "output_provider/MRFT_block");
    // ROSUnit* check_output3 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "output_provider/Controller_sum");
                                                               

    //**************************SETTING BLOCKS**********************************
    Block* PID_x = new PIDController(block_id::PID_X);
    Block* PID_pitch = new PIDController(block_id::PID_PITCH);
    Block* PID_y = new PIDController(block_id::PID_Y);
    Block* PID_y_camera = new PIDController(block_id::PID_Camera_Y);
    Block* PID_roll = new PIDController(block_id::PID_ROLL);
    Block* PID_z = new PIDController(block_id::PID_Z);
    Block* PID_z_camera = new PIDController(block_id::PID_Camera_Z);
    Block* PID_yaw = new PIDController(block_id::PID_YAW);
    Block* PID_yaw_rate = new PIDController(block_id::PID_YAW_RATE);

    Block* MRFT_y = new MRFTController(block_id::MRFT_Y);
    Block* MRFT_z = new MRFTController(block_id::MRFT_Z);

    Transform_InertialToBody* inertialToBody_RotMat = new Transform_InertialToBody();

    Saturation* X_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Y_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Yaw_Saturation = new Saturation(SATURATION_VALUE_YAW);
    Saturation* YawRate_Saturation = new Saturation(SATURATION_VALUE_YAWRATE);

    //*********************SETTING ACTUATION SYSTEMS************************
    
    Actuator* M1 = new ESCMotor(0, PWM_FREQUENCY);
    Actuator* M2 = new ESCMotor(1, PWM_FREQUENCY);
    Actuator* M3 = new ESCMotor(2, PWM_FREQUENCY);
    Actuator* M4 = new ESCMotor(3, PWM_FREQUENCY);
    Actuator* M5 = new ESCMotor(4, PWM_FREQUENCY);
    Actuator* M6 = new ESCMotor(5, PWM_FREQUENCY);

    std::vector<Actuator*> actuators{M1, M2, M3, M4, M5, M6};

    ActuationSystem* myActuationSystem = new HexaActuationSystem(actuators);
    #ifdef BIG_HEXA
    myActuationSystem->setESCValues(1165 ,1000, 2000);
    #endif
    // ActuationSystem* myActuationSystem = new QuadActuationSystem(actuators);

    // //***********************************SETTING CONNECTIONS***********************************
    // //========                                                                             =============
    // //|      |-------------->X_Control_System-->RM_X-->Saturation-->Roll_Control_System--->|           |
    // //| USER |-------------->Y_Control_System-->RM_Y-->Saturation-->Pitch_Control_System-->| Actuation |
    // //|      |-------------->Z_Control_System--------------------------------------------->|  System   |
    // //|      |-------------->Yaw_Control_System-->Saturation--->YawRate_Control_System---->|           |
    // //========                                                                             =============
    
    //*******************************************************************************************************************

    Sum* sum_ref_x = new Sum(std::minus<float>());
    Sum* sum_ref_dot_x = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_x = new Sum(std::minus<float>());
    Demux3D* prov_demux_x = new Demux3D();
    Mux3D* error_mux_x = new Mux3D();

    rosunit_waypoint_x->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_0]->connect(sum_ref_x->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_x_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(prov_demux_x->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_x->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_x->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_x->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_x->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_x->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_x->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_x->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_x->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_x->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_x->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);
    
    error_mux_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    // Rotation Matrix
    PID_x->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_0_X]);

    // Saturation
    inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::OP_0_DATA]->connect(X_Saturation->getPorts()[(int)Saturation::ports_id::IP_0_DATA]);
    
    // Roll
    Sum* sum_ref_roll = new Sum(std::minus<float>());
    Sum* sum_ref_dot_roll = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_roll = new Sum(std::minus<float>());
    Demux3D* prov_demux_roll = new Demux3D();
    Mux3D* error_mux_roll = new Mux3D();

    X_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(sum_ref_roll->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_5]->connect(prov_demux_roll->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_roll->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_roll->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_roll->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_roll->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_roll->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_roll->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_roll->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_roll->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_roll->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_roll->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_roll->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_roll->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_roll->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);

    PID_roll->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_0_DATA_ROLL]);
    
    //*******************************************************************************************************************

    Switch* PID_MRFT_switch_y = new Switch(std::greater_equal<float>(), 2.0);
    Switch* Translation_camera_switch_y = new Switch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* reference_switch_y = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* provider_switch_y = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    Sum* sum_ref_y = new Sum(std::minus<float>());
    Sum* sum_ref_dot_y = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_y = new Sum(std::minus<float>());
    Demux3D* prov_demux_y = new Demux3D();
    Mux3D* error_mux_y = new Mux3D();
    ConstantFloat* constant_reference_y=new ConstantFloat(0);

    ros_optitrack_mrft_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(PID_MRFT_switch_y->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
    ros_optitrack_mrft_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(constant_reference_y->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    ros_optitrack_mrft_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(reference_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);

    ros_camera_mrft_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_1]->connect(PID_MRFT_switch_y->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
    ros_camera_mrft_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_1]->connect(reference_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_camera_mrft_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_1]->connect(constant_reference_y->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    ros_camera_mrft_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_1]->connect(provider_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);

    ros_camera_pid_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_2]->connect(reference_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_camera_pid_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_2]->connect(constant_reference_y->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    ros_camera_pid_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_2]->connect(provider_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_camera_pid_switch_y->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_2]->connect(Translation_camera_switch_y->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
    
    rosunit_waypoint_y->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_1]->connect(reference_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    constant_reference_y->getPorts()[(int)ConstantFloat::ports_id::OP_0_DATA]->connect(reference_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);

    rosunit_y_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(provider_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    rosunit_y_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(constant_reference_y->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    rosunit_y_camera_provider_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(provider_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    rosunit_y_camera_provider_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(constant_reference_y->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    
    provider_switch_y->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(prov_demux_y->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    prov_demux_y->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_y->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_y->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_y->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_y->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_y->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    reference_switch_y->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(sum_ref_y->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    sum_ref_y->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_y->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_y->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_y->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_y->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_y->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(Translation_camera_switch_y->getPorts()[(int)Switch::ports_id::IP_0_DATA]);
    Translation_camera_switch_y->getPorts()[(int)Switch::ports_id::OP_0_DATA_DEFAULT]->connect(PID_MRFT_switch_y->getPorts()[(int)Switch::ports_id::IP_0_DATA]);
    Translation_camera_switch_y->getPorts()[(int)Switch::ports_id::OP_1_DATA]->connect(((PIDController*)PID_y_camera)->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    PID_MRFT_switch_y->getPorts()[(int)Switch::ports_id::OP_0_DATA_DEFAULT]->connect(((PIDController*)PID_y)->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    PID_MRFT_switch_y->getPorts()[(int)Switch::ports_id::OP_1_DATA]->connect(((MRFTController*)MRFT_y)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);


    // Rotation Matrix
    ((PIDController*)PID_y_camera)->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_1_Y]);
    ((PIDController*)PID_y)->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_1_Y]);
    ((MRFTController*)MRFT_y)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_1_Y]);
      

    

    // Saturation
    inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::OP_1_DATA]->connect(Y_Saturation->getPorts()[(int)Saturation::ports_id::IP_0_DATA]);
    // Pitch
    Sum* sum_ref_pitch = new Sum(std::minus<float>());
    Sum* sum_ref_dot_pitch = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_pitch = new Sum(std::minus<float>());
    Demux3D* prov_demux_pitch = new Demux3D();
    Mux3D* error_mux_pitch = new Mux3D();

    Y_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(sum_ref_pitch->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_6]->connect(prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_pitch->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_pitch->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_pitch->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_pitch->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_pitch->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_pitch->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_pitch->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_pitch->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_pitch->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);
 
    error_mux_pitch->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);

    PID_pitch->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_1_DATA_PITCH]);
    
    //*******************************************************************************************************************

    Switch* PID_MRFT_switch_z = new Switch(std::greater_equal<float>(), 2.0);
    Switch* Translation_camera_switch_z = new Switch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* reference_switch_z = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* provider_switch_z = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* controller_sum_switch = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    Sum* sum_ref_z = new Sum(std::minus<float>());
    Sum* sum_ref_dot_z = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_z = new Sum(std::minus<float>());
    Sum* controller_sum = new Sum(std::plus<float>());
    Demux3D* prov_demux_z = new Demux3D();
    Mux3D* error_mux_z = new Mux3D();
    ConstantFloat* constant_reference_z=new ConstantFloat(1);
    ConstantFloat* zero_constant=new ConstantFloat(0);

    ros_optitrack_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_3]->connect(PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
    ros_optitrack_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_3]->connect(constant_reference_z->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    ros_optitrack_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_3]->connect(reference_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_optitrack_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_3]->connect(controller_sum_switch->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    
    ros_camera_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_4]->connect(PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
    ros_camera_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_4]->connect(reference_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_camera_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_4]->connect(constant_reference_z->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    ros_camera_mrft_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_4]->connect(provider_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);

    ros_camera_pid_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_5]->connect(reference_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_camera_pid_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_5]->connect(constant_reference_z->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    ros_camera_pid_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_5]->connect(provider_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_camera_pid_switch_z->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_5]->connect(Translation_camera_switch_z->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
       
    rosunit_waypoint_z->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_2]->connect(reference_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    constant_reference_z->getPorts()[(int)ConstantFloat::ports_id::OP_0_DATA]->connect(reference_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);

    rosunit_z_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(provider_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    rosunit_z_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(constant_reference_z->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);
    rosunit_z_camera_provider_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_4]->connect(provider_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    rosunit_z_camera_provider_sub->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_4]->connect(constant_reference_z->getPorts()[(int)ConstantFloat::ports_id::IP_1_TRIGGER]);

    provider_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(prov_demux_z->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    reference_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(sum_ref_z->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    sum_ref_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(Translation_camera_switch_z->getPorts()[(int)Switch::ports_id::IP_0_DATA]);
    Translation_camera_switch_z->getPorts()[(int)Switch::ports_id::OP_0_DATA_DEFAULT]->connect(PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::IP_0_DATA]);
    Translation_camera_switch_z->getPorts()[(int)Switch::ports_id::OP_1_DATA]->connect(((PIDController*)PID_z_camera)->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::OP_0_DATA_DEFAULT]->connect(((PIDController*)PID_z)->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::OP_1_DATA]->connect(((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);
    
    ((PIDController*)PID_z)->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(controller_sum->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    zero_constant->getPorts()[(int)ConstantFloat::ports_id::OP_0_DATA]->connect(controller_sum_switch->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    ((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(controller_sum_switch->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    controller_sum_switch->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(controller_sum->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    
    // //Logger
    // ((PIDController*)PID_z)->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(check_output1->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    // ((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(check_output2->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

    // Rotation Matrix
    ((PIDController*)PID_z_camera)->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_3_DATA_Z]);
    controller_sum->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_3_DATA_Z]);
    
    // //Logger
    // controller_sum->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(check_output3->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

      

    


    //*******************************************************************************************************************
    // YAW CHANNEL ->  Multirotors From Takeoff to Real-Time Full Identification Using the Modified Relay Feedback Test and Deep Neural Networks //

    Sum* sum_ref_yaw = new Sum(std::minus<float>());
    Sum* sum_ref_dot_yaw = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_yaw = new Sum(std::minus<float>());
    Demux3D* prov_demux_yaw = new Demux3D();
    Mux3D* error_mux_yaw = new Mux3D();

    rosunit_waypoint_yaw->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_3]->connect(sum_ref_yaw->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_7]->connect(prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_7]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_2_YAW]);

    prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(((Block*)sum_ref_yaw)->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_yaw->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_yaw->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_yaw->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_yaw->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_yaw->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_yaw->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    
    PID_yaw->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(Yaw_Saturation->getPorts()[(int)Saturation::ports_id::IP_0_DATA]);

    // Yaw Rate
    Sum* sum_ref_yaw_rate = new Sum(std::minus<float>());
    Sum* sum_ref_dot_yaw_rate = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_yaw_rate = new Sum(std::minus<float>());
    Demux3D* prov_demux_yaw_rate = new Demux3D();
    Mux3D* error_mux_yaw_rate = new Mux3D();

    Yaw_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(sum_ref_yaw_rate->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_yaw_rate_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_8]->connect(prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_yaw_rate->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_yaw_rate->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_yaw_rate->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_yaw_rate->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_yaw_rate->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_yaw_rate->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    
    PID_yaw_rate->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_2_DATA_YAW]);
    //*******************************************************************************************************************
    
    // ROS CONTROL OUTPUTS
    X_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_0_X_OUTPUT]);
    Y_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_1_Y_OUTPUT]);
    MRFT_y->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_1_Y_OUTPUT]);
    PID_z->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_2_Z_OUTPUT]);
    MRFT_z->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_2_Z_OUTPUT]);
    PID_roll->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_3_ROLL_OUTPUT]);
    PID_pitch->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_4_PITCH_OUTPUT]);
    Yaw_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_5_YAW_OUTPUT]);
    PID_yaw_rate->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_6_YAWRATE_OUTPUT]);   

    //***********************SETTING FLIGHT SCENARIO INPUTS****************************
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_z->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_0_PID]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_1_MRFT]->connect(MRFT_z->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateControllerSrv::ports_id::OP_1_MRFT]->connect(MRFT_y->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_z->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    //((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_z_identification->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(MRFT_z->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    ((Block*)myROSResetController)->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(MRFT_y->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);

    myROSArm->getPorts()[(int)ROSUnit_SetBoolSrv::ports_id::OP_0]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_4_ARM]);
    
    //********************SETTING FLIGHT SCENARIO OUTPUTS***************************

    ((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::OP_0_CMD]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_14_MOTORS]);
    ((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::OP_1_ARM]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_15_ARMED]);

    Timer tempo;
    while(ros::ok()){
        tempo.tick();

        ros::spinOnce();

        int gone = tempo.tockMicroSeconds();
        if(gone > 5000) {
            std::cout  << "FC over 5000: " << gone << "\n";
        }
        rate.sleep();

    }
    return 0;
}
