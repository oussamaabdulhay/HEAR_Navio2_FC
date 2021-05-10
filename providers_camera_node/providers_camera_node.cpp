//PROVIDERS_NODE_CAMERA V1
// 09 May 2021
// OUSSAMA ABDUL HAY

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
#include "HEAR_ROS_BRIDGE/ROSUnit_VS.hpp"
#include "HEAR_math/Differentiator.hpp"
#include "HEAR_math/ButterFilter_2nd.hpp"
#include "HEAR_math/KalmanFilter.hpp"
#include "HEAR_math/InverseRotateVec.hpp"
#include "HEAR_math/DownSampler.hpp"
#include "HEAR_math/SupressPeak.hpp"
#include "HEAR_math/PolyFilter.hpp"
#include "HEAR_core/InvertedSwitch.hpp"
#include "HEAR_mission/Threshold_status.hpp"
#include "HEAR_core/Switch.hpp"

const int CAMERA_FREQUENCY = 100;

int main(int argc, char **argv){
    ros::init(argc, argv, "providers_camera_node");
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* rosunit_reset_kalman = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                            ROSUnit_msg_type::ROSUnit_Float,
                                                            "reset_kalman"); //0
    ROSUnit* rosunit_switch_pds_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Float,
                                                            "step_pds_switch_x");
    ROSUnit* rosunit_switch_pds_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Float,
                                                            "step_pds_switch_y");
    ROSUnit* rosunit_switch_pds_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                            ROSUnit_msg_type::ROSUnit_Float,
                                                            "step_pds_switch_z");
    ROSUnit* rosunit_x_camera_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/camera/x");
    ROSUnit* rosunit_y_camera_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/camera/y");
    ROSUnit* rosunit_z_camera_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/camera/z");
    ROSUnit* probe1 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/diff_x");
    ROSUnit* probe2 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/diff_z");
    ROSUnit* probe3 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/kalman_filter_output_x");
    ROSUnit* probe4 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/kalman_filter_output_z");
    ROSUnit* probe5 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/trigger_x");
    ROSUnit* probe6 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/trigger_z");
    ROSUnit* probe7 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/polyfilter/x");
    ROSUnit* probe8 = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "/polyfilter/z");


    //***********************ADDING SENSORS********************************
    ROSUnit* myROSUnit_CAMERA = new ROSUnit_VS(nh);
    ROSUnit* myROSUnit_Xsens = new ROSUnit_IMU(nh);
    //***********************SETTING PROVIDERS**********************************
    Mux3D* mux_provider_kalman_x = new Mux3D();
    //Mux3D* mux_provider_kalman_y = new Mux3D();
    Mux3D* mux_provider_kalman_z = new Mux3D();

    Mux3D* mux_camera_provider_x = new Mux3D();
    //Mux3D* mux_camera_provider_y = new Mux3D();
    Mux3D* mux_camera_provider_z = new Mux3D();

    Demux3D* camera_pos_demux = new Demux3D();
    Demux3D* rotated_IMU_demux = new Demux3D();

    WrapAroundFunction* wrap_around_yaw = new WrapAroundFunction(-M_PI, M_PI);

    InverseRotateVec* rotation_IMU = new InverseRotateVec();

    KalmanFilter* camera_x_kalmanFilter= new KalmanFilter(1);
    KalmanFilter* camera_z_kalmanFilter= new KalmanFilter(1);
    Differentiator* camera_x_dot = new Differentiator(1./CAMERA_FREQUENCY);
    Differentiator* camera_z_dot = new Differentiator(1./CAMERA_FREQUENCY);


    InvertedSwitch* mux_position_switch_x = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* mux_velocity_switch_x = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* mux_position_switch_z = new InvertedSwitch(std::greater_equal<float>(), 2.0);
    InvertedSwitch* mux_velocity_switch_z = new InvertedSwitch(std::greater_equal<float>(), 2.0);

    Threshold_status* threshold_x = new Threshold_status(0.2, 0.05, 100, CAMERA_FREQUENCY);
    Threshold_status* threshold_z = new Threshold_status(0.2, 0.05, 100, CAMERA_FREQUENCY);
    
    SupressPeak* supress_vel_x = new SupressPeak(5);
    SupressPeak* supress_vel_z = new SupressPeak(5);

    PolyFilter* poly_fit_x = new PolyFilter(100,20,2);
    PolyFilter* poly_fit_z = new PolyFilter(100,20,2);


    myROSUnit_CAMERA->getPorts()[(int)ROSUnit_VS::ports_id::OP_0_VS]->connect(camera_pos_demux->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(threshold_x->getPorts()[Threshold_status::ports_id::IP_0_VS]);
    camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(threshold_x->getPorts()[Threshold_status::ports_id::IP_1_KF]);
    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(threshold_z->getPorts()[Threshold_status::ports_id::IP_0_VS]);
    camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(threshold_z->getPorts()[Threshold_status::ports_id::IP_1_KF]);
   
   
    // threshold_x->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(mux_position_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    // threshold_z->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(mux_position_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    // threshold_x->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(mux_velocity_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    // threshold_z->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(mux_velocity_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_1_TRIGGER]);
    // threshold_x->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(rosunit_switch_pds_x->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    // threshold_z->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(rosunit_switch_pds_z->getPorts()[(int)ROSUnit_SetFloatClnt::ports_id::IP_0]);
    // threshold_x->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(probe5->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    // threshold_z->getPorts()[Threshold_status::ports_id::OP_0_HOV_TRACK]->connect(probe6->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);


    //CAMERA X PROVIDER WITH KALMAN FILTER
    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(mux_position_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);

    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(poly_fit_x->getPorts()[(int)PolyFilter::ports_id::IP_0_DATA]);
    poly_fit_x->getPorts()[(int)PolyFilter::ports_id::OP_1_FILT_DOT]->connect(probe7->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    
    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_0_DATA]->connect(camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    rosunit_reset_kalman->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_2_RESET]);
    
    camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_position_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_velocity_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    
    camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_provider_kalman_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    camera_x_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_provider_kalman_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(camera_x_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    poly_fit_x->getPorts()[(int)PolyFilter::ports_id::OP_1_FILT_DOT]->connect(mux_velocity_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    //camera_x_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(mux_velocity_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    camera_x_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(probe1->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);
    
    mux_position_switch_x->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_camera_provider_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    mux_velocity_switch_x->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(supress_vel_x->getPorts()[(int)SupressPeak::ports_id::IP_0_VEL]);
    supress_vel_x->getPorts()[(int)SupressPeak::ports_id::OP_VEL_THRESHOLDED]->connect(mux_camera_provider_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);



    //CAMERA Z PROVIDER WITH KALMAN FILTER
    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(mux_position_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    
    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(poly_fit_z->getPorts()[(int)PolyFilter::ports_id::IP_0_DATA]);
    poly_fit_z->getPorts()[(int)PolyFilter::ports_id::OP_1_FILT_DOT]->connect(probe8->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_1_POS]);
    rotated_IMU_demux->getPorts()[Demux3D::ports_id::OP_2_DATA]->connect(camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_0_ACC]);
    rosunit_reset_kalman->getPorts()[(int)ROSUnit_SetFloatSrv::ports_id::OP_0]->connect(camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::IP_2_RESET]);
    
    camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_position_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_velocity_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    
    camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_0_POS]->connect(mux_provider_kalman_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    camera_z_kalmanFilter->getPorts()[(int)KalmanFilter::ports_id::OP_1_VEL]->connect(mux_provider_kalman_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    camera_pos_demux->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(camera_z_dot->getPorts()[(int)Differentiator::ports_id::IP_0_DATA]);
    poly_fit_z->getPorts()[(int)PolyFilter::ports_id::OP_1_FILT_DOT]->connect(mux_velocity_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    //camera_z_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(mux_velocity_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    camera_z_dot->getPorts()[(int)Differentiator::ports_id::OP_0_DATA]->connect(probe2->getPorts()[(int)ROSUnit_FloatPub::ports_id::IP_0]);

    mux_position_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(mux_camera_provider_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    mux_velocity_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(supress_vel_z->getPorts()[(int)SupressPeak::ports_id::IP_0_VEL]);
    supress_vel_z->getPorts()[(int)SupressPeak::ports_id::OP_VEL_THRESHOLDED]->connect(mux_camera_provider_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);

    //Rotated imu vector
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_5_FREE_ACCELERATION]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_0_VEC]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_0_ROLL]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_1_ROLL]);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_IMU::ports_id::OP_1_PITCH]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_2_PITCH]);
    wrap_around_yaw->getPorts()[(int)WrapAroundFunction::ports_id::OP_0_DATA]->connect(rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::IP_3_YAW]);
    rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::OP_0_DATA]->connect(rotated_IMU_demux->getPorts()[Demux3D::ports_id::IP_0_DATA]);
    //rotation_IMU->getPorts()[(int)InverseRotateVec::ports_id::OP_0_DATA]->connect(rosunit_rotation_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);


    mux_camera_provider_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_x_camera_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_camera_provider_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(rosunit_z_camera_provider_pub->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_kalman_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(probe3->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);
    mux_provider_kalman_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(probe4->getPorts()[(int)ROSUnit_PointPub::ports_id::IP_0]);

    std::cout  << "###### PROVIDERS CAMERA NODE ######" "\n";
    
    Timer tempo;
    int i = 0;
    while(ros::ok()){
        tempo.tick();

        ros::spinOnce();
       
        int gone = tempo.tockMicroSeconds();
        if(gone > 5000) {
             std::cout  << i << " PROV_CAMERA: " << gone << "\n";
        }
        i++;

        rate.sleep();

    }

    return 0;
}