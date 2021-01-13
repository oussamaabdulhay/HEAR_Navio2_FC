//PROVIDERS_NODE V1.0.2
// 18 June 2020
// Pedro Henrique Silva
// YawRate from Xsens

#include <iostream>

#include "HEAR_core/Timer.hpp"
#include "HEAR_core/Mux3D.hpp"
#include "HEAR_core/Demux3D.hpp"
#include "HEAR_core/std_logger.hpp"
#include "HEAR_nav/Global2Inertial.hpp"
#include "HEAR_nav/WrapAroundFunction.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_IMU.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Factory.hpp"
#include "HEAR_ROS_BRIDGE/ROSUnit_Optitrack.hpp"
#include "HEAR_math/Differentiator.hpp"
#include "HEAR_math/ButterFilter_2nd.hpp"
#include "HEAR_math/KalmanFilter.hpp"
#include "HEAR_math/InverseRotateVec.hpp"
#include "HEAR_math/DownSampler.hpp"
#include "HEAR_core/InvertedSwitch.hpp"
#include "HEAR_core/Switch.hpp"


const int OPTITRACK_FREQUENCY = 120;
const int CAMERA_FREQUENCY = 60;

int main(int argc, char **argv){
    ros::init(argc, argv, "providers_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit_Factory ROSUnit_Factory_main{nh};
    ROSUnit* rosunit_x_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_x_kalman_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/kalman/x");
    ROSUnit* rosunit_y_kalman_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/kalman/y");
    ROSUnit* rosunit_z_kalman_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/kalman/z");
    ROSUnit* rosunit_roll_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");
    ROSUnit* rosunit_y_camera_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/camera/y");
    ROSUnit* rosunit_z_camera_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/camera/z");
    ROSUnit* myCameraPosition =  ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "camera_provider");
    ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/position");
    ROSUnit* rosunit_g2i_orientation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/orientation");
    ROSUnit* rosunit_imu_acceleration = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_GeoVec,
                                                                    "/imu/acceleration");
    ROSUnit* probe1 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/posiition_opti");
    ROSUnit* probe2 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/velocity_opti");
    ROSUnit* rosunit_rotation_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/rotated_accelerometer");
    ROSUnit* bias_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/Kalman_bias/x");
    ROSUnit* bias_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/Kalman_bias/y");
    ROSUnit* bias_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/Kalman_bias/z");
    ROSUnit* ros_kalmanFilter_switch = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "kalman_filter_switch");
    ROSUnit* ros_reset_kalmanFilter = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                      ROSUnit_msg_type::ROSUnit_Float,
                                                                      "kalman_filter_reset");
    //***********************ADDING SENSORS********************************
    ROSUnit* myROSUnit_Xsens = new ROSUnit_IMU(nh);
    //***********************SETTING PROVIDERS**********************************
    Mux3D* mux_provider_x = new Mux3D();
    Mux3D* mux_provider_y = new Mux3D();
    Mux3D* mux_provider_z = new Mux3D();
    Mux3D* mux_provider_kalman_x = new Mux3D();
    Mux3D* mux_provider_kalman_y = new Mux3D();
    Mux3D* mux_provider_kalman_z = new Mux3D();

    Mux3D* mux_camera_provider_y = new Mux3D();
    Mux3D* mux_camera_provider_z = new Mux3D();
    Mux3D* mux_provider_roll = new Mux3D();
    Mux3D* mux_provider_pitch = new Mux3D();
    Mux3D* mux_provider_yaw = new Mux3D();
    Mux3D* mux_provider_yaw_rate = new Mux3D();

    Demux3D* pos_demux = new Demux3D();
    Demux3D* camera_pos_demux = new Demux3D();
    Demux3D* ori_demux = new Demux3D();
    Demux3D* rotated_IMU_demux = new Demux3D();


    // KalmanFilter* camera_y_kalmanFilter= new KalmanFilter(1);
    // KalmanFilter* camera_z_kalmanFilter= new KalmanFilter(1);
    KalmanFilter* optitrack_x_kalmanFilter= new KalmanFilter(1);
    KalmanFilter* optitrack_y_kalmanFilter= new KalmanFilter(1);
    KalmanFilter* optitrack_z_kalmanFilter= new KalmanFilter(1);

    DownSampler* downsample_y=new DownSampler(1);
    DownSampler* downsample_z=new DownSampler(1);

    WrapAroundFunction* wrap_around_yaw = new WrapAroundFunction(-M_PI, M_PI);
    
    Differentiator* optitrack_x_dot = new Differentiator(1./OPTITRACK_FREQUENCY);
    Differentiator* optitrack_y_dot = new Differentiator(1./OPTITRACK_FREQUENCY);
    Differentiator* optitrack_z_dot = new Differentiator(1./OPTITRACK_FREQUENCY);
    Differentiator* camera_y_dot = new Differentiator(1./CAMERA_FREQUENCY);
    Differentiator* camera_z_dot = new Differentiator(1./CAMERA_FREQUENCY);

    ButterFilter_2nd* filter_x_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS120FC5);
    ButterFilter_2nd* filter_y_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS120FC5);
    ButterFilter_2nd* filter_z_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS120FC5);
    

    ButterFilter_2nd* filter_roll_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS200FC50);
    ButterFilter_2nd* filter_pitch_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS200FC50);
    ButterFilter_2nd* filter_yaw_dot = new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS200FC50);

    InverseRotateVec* rotation_IMU = new InverseRotateVec();

    InvertedSwitch* x_switch_provider_pos= new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* x_switch_provider_vel= new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* y_switch_provider_pos= new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* y_switch_provider_vel= new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* z_switch_provider_pos= new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* z_switch_provider_vel= new InvertedSwitch(std::greater_equal<float>(), 2.0);
    
    //ros_kalmanFilter_switch->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(x_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    //ros_kalmanFilter_switch->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(x_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_kalmanFilter_switch->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(y_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    ros_kalmanFilter_switch->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(y_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    //ros_kalmanFilter_switch->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(z_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    //ros_kalmanFilter_switch->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(z_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);

    ros_reset_kalmanFilter->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_1]->connect(optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_2_RESET]);
    ros_reset_kalmanFilter->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_1]->connect(optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_2_RESET]);
    ros_reset_kalmanFilter->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_1]->connect(optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_2_RESET]);

    rosunit_g2i_position->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(pos_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    rosunit_g2i_orientation->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(ori_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    myCameraPosition->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(camera_pos_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    // Setting Provider -> Always leave the pv connection last. Do pv_dot and pv_dot_dor first.
    // X Provider 
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(optitrack_x_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    optitrack_x_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(((Block*)filter_x_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_x_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(x_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(x_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);

    //Y Provider
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(optitrack_y_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    optitrack_y_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(((Block*)filter_y_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_y_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(y_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(y_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(probe1->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    ((Block*)filter_y_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(probe2->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

    // //Z Provider 
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(optitrack_z_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    optitrack_z_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(((Block*)filter_z_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_z_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(z_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(z_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);

    //X Provider with kalmna filter
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_0_DATA]->connect(optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(x_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(x_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_2_BIAS]->connect(bias_x->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_provider_kalman_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    optitrack_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_provider_kalman_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    //y Provider with kalmna filter
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_1_DATA]->connect(optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(y_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(y_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_2_BIAS]->connect(bias_y->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_provider_kalman_y->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    optitrack_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_provider_kalman_y->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    //z Provider with kalmna filter
    pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_2_DATA]->connect(optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(z_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(z_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_2_BIAS]->connect(bias_z->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_provider_kalman_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    optitrack_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_provider_kalman_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    //Comparing normal differntiation with kalman filter CAMERA Y
    // camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(camera_y_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    // camera_y_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(probe2->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    
    //CAMERA Y PROVIDER WITH KALMAN FILTER
    // camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(downsample_y->getPorts()[(int)DownSampler::ports_id::IP_0_DATA]);
    // downsample_z->getPorts()[(int)DownSampler::ports_id::OP_0_DATA]->connect(camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    // rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_1_DATA]->connect(camera_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    // camera_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_camera_provider_y->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    // camera_y_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_camera_provider_y->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
       
    //Comparing normal differntiation with kalman filter CAMERA Z
    // camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(camera_z_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    // camera_z_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(probe1->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

    //CAMERA Z PROVIDER WITH KALMAN FILTER
    // camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(downsample_z->getPorts()[(int)DownSampler::ports_id::IP_0_DATA]);
    // downsample_z->getPorts()[(int)DownSampler::ports_id::OP_0_DATA]->connect(camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    // rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_2_DATA]->connect(camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    // camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_camera_provider_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    // camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_camera_provider_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
 
    //Roll Provider
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_2_ROLL_RATE]->connect(((Block*)filter_roll_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_roll_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_roll->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_0_ROLL]->connect(mux_provider_roll->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    //Pitch Provider
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_3_PITCH_RATE]->connect(((Block*)filter_pitch_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_pitch_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_pitch->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_1_PITCH]->connect(mux_provider_pitch->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    //Yaw Provider
    ori_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(wrap_around_yaw->getPorts()[(int)WrapAroundFunction::ports_id::IP_0_DATA]);
    wrap_around_yaw->getPorts()[(int)WrapAroundFunction::ports_id::OP_0_DATA]->connect(mux_provider_yaw->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);

    //Yaw Rate Provider
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_4_YAW_RATE]->connect(((Block*)filter_yaw_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::IP_0_DATA]);
    ((Block*)filter_yaw_dot)->getPorts()[(int)ButterFilter_2nd::ports_id::OP_0_DATA]->connect(mux_provider_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    
    //Rotated imu vector
    rosunit_imu_acceleration->getPorts()[(int)ROSUnit_GeoVecSub::ports_id::OP_0]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_0_VEC]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_0_ROLL]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_1_ROLL]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_1_PITCH]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_2_PITCH]);
    wrap_around_yaw->getPorts()[(int)WrapAroundFunction::ports_id::OP_0_DATA]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_3_YAW]);
    rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::OP_0_DATA]->connect(rotated_IMU_demux->getPorts()[Demux3D::ports_id::IP_0_DATA]);
    rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::OP_0_DATA]->connect(rosunit_rotation_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    
    x_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_provider_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    x_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_provider_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    y_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_provider_y->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    y_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_provider_y->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    z_switch_provider_pos->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_provider_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    z_switch_provider_vel->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_provider_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    
    mux_provider_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_x_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_y_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_z_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_kalman_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_x_kalman_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_kalman_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_y_kalman_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_kalman_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_z_kalman_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_camera_provider_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_y_camera_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_camera_provider_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_z_camera_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_roll->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_roll_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_pitch->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_pitch_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_yaw->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_yaw_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_yaw_rate->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_yaw_rate_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

    std::cout  << "###### PROVIDERS NODE ######" "\n";
    
    Timer tempo;
    int i = 0;
    while(ros::ok()){
        tempo.tick();

        ros::spinOnce();
       
        int gone = tempo.tockMicroSeconds();
        if(gone > 5000) {
             std::cout  << i << " PROV: " << gone << "\n";
        }
        i++;

        rate.sleep();

    }

    return 0;
}