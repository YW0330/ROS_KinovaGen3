#include <iostream>
#include <cmath>
#include <string>
#include <vector>

// ros 相關
#include "ros/ros.h"
#include "kinova_test/kinovaMsg.h"
#include "std_msgs/Bool.h"

// 自行加入的功能
#include "kinova_test/KinovaConfig.h"
#include "kinova_test/controller.h"
#include "kinova_test/conio.h"
#include "kinova_test/HumanState.h"

bool torque_control(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient *actuator_config, ros::Publisher &kinova_pub, HumanState &humanState)
{
    // ROS
    kinova_test::kinovaMsg kinovaInfo;

    double init_tau[7];
    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();
    cout << "actuator_counts: " << actuator_count << endl;
    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch (...)
    {
        cout << "Unable to clear robot faults" << endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    // 夾爪功能開啟
    k_api::GripperCyclic::MotorCommand *gripper_motor_command;
    gripper_motor_command = base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();

    vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    cout << "Initializing the arm for torque control" << endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        for (int i = 1; i <= actuator_count; i++)
            actuator_config->SetControlMode(control_mode_message, i);
        for (int i = 0; i < actuator_count; i++)
            init_tau[i] = -base_feedback.actuators(i).torque();

        /* ------------ 初始值參數設定開始 ------------ */
        // 讀取關節角
        Matrix<double> position_curr(7, 1); // -pi~pi
        Matrix<int> round(7, 1);            // 圈數
        Matrix<double> q(7, 1), dq(7, 1);   // -inf~inf
        for (int i = 0; i < 7; i++)
        {
            position_curr[i] = base_feedback.actuators(i).position() * DEG2RAD;
            if (position_curr[i] > M_PI)
                position_curr[i] = -(2 * M_PI - position_curr[i]);
        }
        q = position_curr;
        // 順向運動學
        Matrix<double> X = forward_kinematic_6dof(position_curr), dX(DOF, 1);
        Matrix<double> X0 = X;

        // Jacobian矩陣
        double J_arr[42], Jinv_arr[42];
        Matrix<double> J67(6, 7);
        Matrix<double> J(DOF, 7), Jinv(7, DOF), dJinv(7, DOF);
        kinova_J_and_Jinv(position_curr[0], position_curr[1], position_curr[2], position_curr[3], position_curr[4], position_curr[5], J_arr, Jinv_arr);
        J67.update_from_matlab(J_arr);
        J = J67;
        Jinv.update_from_matlab(Jinv_arr);

        // 微分前一筆
        Matrix<double> prev_q = q;
        Matrix<double> prev_Jinv = Jinv;
        Matrix<double> prev_subtasks(7, 1);

        // 目標輸出
        Matrix<double> Xd0 = humanState.Xd;
        Matrix<double> Xd = X0 + humanState.Xd - Xd0;

        Matrix<double> dXd = humanState.dXd;
        Matrix<double> error = Xd - X;
        Matrix<double> derror = dXd - dX;

        // 控制器相關
        Matrix<double> param_s(DOF, 1), param_v(DOF, 1), param_a(DOF, 1), param_r(DOF, 1);
        Matrix<double> phi(NODE, 7), dW_hat(NODE, 1), W_hat(NODE, 1);
        Matrix<double> psi(7, 1), subtasks(7, 1), dsubtasks(7, 1);
        Matrix<double> controller_tau(7, 1);
        /* ------------ 初始值參數設定結束 ------------ */

        int64_t t_start = GetTickUs(), now = GetTickUs(), last = now; // 微秒
        double exp_time = (double)(now - t_start) / 1000000, dt;      // 秒
        // Real-time loop
        while (ros::ok())
        {
            if (!_kbhit())
            {
                now = GetTickUs();
                kinovaInfo.time = exp_time;
                if (now - last > 1000)
                {
                    if (exp_time < 3)
                        Xd0 = humanState.Xd;

                    // Position command to first actuator is set to measured one to avoid following error to trigger
                    // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                    //        actuator continues to move under torque command, resulting position error with command will
                    //        trigger a following error and switch back the actuator in position command to hold its position
                    for (int i = 0; i < actuator_count; i++)
                        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());

                    // 控制器參數
                    joint_angle_limit_psi(position_curr, psi);
                    manipulability_psi(position_curr, psi);
                    null_space_subtasks(J, Jinv, psi, subtasks);
                    contrller_params(J, Jinv, dJinv, error, derror, dq, subtasks, dsubtasks, param_s, param_v, param_a, param_r);
                    // RBFNN
                    get_phi(param_v, param_a, q, dq, phi);
                    get_dW_hat(phi, param_s, dW_hat);
                    // 控制器
                    controller(J, dX, dXd, param_s, param_r, phi, W_hat, controller_tau);
                    // 重力補償
                    gravity_compensation(position_curr, init_tau, controller_tau);

                    // 設定飽和器
                    torque_satuation(controller_tau);
                    // 輸入扭矩
                    for (int i = 0; i < 7; i++)
                        base_command.mutable_actuators(i)->set_torque_joint(controller_tau[i]);
                    // 夾爪開闔
                    gripper_control(gripper_motor_command, humanState.triggerVal);

                    // 讀取關節角
                    for (int i = 0; i < 7; i++)
                    {
                        kinovaInfo.jointPos[i] = base_feedback.actuators(i).position();
                        position_curr[i] = base_feedback.actuators(i).position() * DEG2RAD;
                        if (position_curr[i] > M_PI)
                            position_curr[i] = -(2 * M_PI - position_curr[i]);
                    }
                    q2inf(position_curr, prev_q, round, q);

                    // 順向運動學
                    X = forward_kinematic_6dof(position_curr);
                    kinovaInfo.kinova_X = {X[0], X[1], X[2]};
                    kinovaInfo.kinova_axis = {X[3], X[4], X[5]};
                    kinovaInfo.kinova_Xd = {Xd[0], Xd[1], Xd[2]};
                    kinovaInfo.kinova_axisd = {Xd[3], Xd[4], Xd[5]};
                    // 夾爪狀態
                    kinovaInfo.gripperPos = base_feedback.interconnect().gripper_feedback().motor(0).position();

                    // Jacobian矩陣
                    kinova_J_and_Jinv(position_curr[0], position_curr[1], position_curr[2], position_curr[3], position_curr[4], position_curr[5], J_arr, Jinv_arr);
                    J67.update_from_matlab(J_arr);
                    J = J67;
                    Jinv.update_from_matlab(Jinv_arr);

                    // 更新時間、微分、積分
                    now = GetTickUs();
                    exp_time = (double)(now - t_start) / 1000000;
                    dt = (double)(now - last) / 1000000;
                    for (int i = 0; i < 7; i++)
                        dq[i] = base_feedback.actuators(i).velocity() * DEG2RAD;
                    dX = J * dq;
                    dJinv = (Jinv - prev_Jinv) / dt;
                    dsubtasks = (subtasks - prev_subtasks) / dt;
                    W_hat = W_hat + dW_hat * dt;
                    prev_q = q;
                    prev_Jinv = Jinv;
                    prev_subtasks = subtasks;
                    last = now;

                    Xd = X0 + humanState.Xd - Xd0;
                    dXd = humanState.dXd;
                    error = Xd - X;
                    derror = dXd - dX;

                    kinova_pub.publish(kinovaInfo);
                    ros::spinOnce(); // 偵測subscriber

                    // Incrementing identifier ensures actuators can reject out of time frames
                    base_command.set_frame_id(base_command.frame_id() + 1);
                    if (base_command.frame_id() > 65535)
                        base_command.set_frame_id(0);

                    for (int idx = 0; idx < actuator_count; idx++)
                    {
                        base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                    }

                    try
                    {
                        base_feedback = base_cyclic->Refresh(base_command, 0);
                    }
                    catch (k_api::KDetailedException &ex)
                    {
                        cout << "Kortex exception: " << ex.what() << endl;

                        cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << endl;
                    }
                    catch (runtime_error &ex2)
                    {
                        cout << "runtime error: " << ex2.what() << endl;
                    }
                    catch (...)
                    {
                        cout << "Unknown error." << endl;
                    }
                }
            }
            else
            {
                if ((char)_getch() == 's')
                {
                    return_status = false;
                    break;
                }
            }
        }

        cout << "Torque control completed" << endl;

        // Set first actuator back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int i = 1; i <= actuator_count; i++)
            actuator_config->SetControlMode(control_mode_message, i);

        cout << "Torque control clean exit" << endl;
    }
    catch (k_api::KDetailedException &ex)
    {
        cout << "API error: " << ex.what() << endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        cout << "Error: " << ex2.what() << endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    for (int i = 0; i < actuator_count; i++)
        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    HumanState humanState;
    // ROS
    ros::init(argc, argv, "mobileRobotDevice"); // rosnode的名稱
    ros::NodeHandle n;
    ros::Publisher kinova_pub = n.advertise<kinova_test::kinovaMsg>("kinovaInfo", 1000); // rostopic的名稱(Publish)
    ros::Subscriber state_sub = n.subscribe("xsens2kinova", 1000, &HumanState::updateHumanData, &humanState);
    ros::Subscriber trigger_sub = n.subscribe("triggerVal", 1000, &HumanState::updateTriggerValue, &humanState);

    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    cout << "Creating transport objects" << endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    cout << "Creating transport real time objects" << endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    cout << "Creating sessions for communication" << endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    cout << "Sessions created" << endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Example core
    bool success = true;
    success &= example_move_to_home_position(base);
    while (success)
    {
        success &= torque_control(base, base_cyclic, actuator_config, kinova_pub, humanState);
    }

    if (!success)
        cout << "There has been an unexpected error." << endl;

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}