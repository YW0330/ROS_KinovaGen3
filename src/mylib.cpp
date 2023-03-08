#include <iostream>

#include "kinova_test/mylib.h"
#include "kinova_test/kinovaMsg.h"

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{WAITING_TIME};

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise)
{
    return [&finish_promise](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

bool example_move_to_home_position(k_api::Base::BaseClient *base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);

    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions());

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if (status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return true;
    }
}

bool move_to_home_position_with_ros(k_api::Base::BaseClient *base, ros::Publisher &kinova_pub)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);

    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions());

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        kinova_test::kinovaMsg kinovaInfo;
        while (1)
        {
            // Wait for future value from promise
            const auto status = finish_future.wait_for(std::chrono::milliseconds(1));

            // 讀各關節角度
            k_api::Base::JointAngles input_joint_angles;
            try
            {
                input_joint_angles = base->GetMeasuredJointAngles();
            }
            catch (k_api::KDetailedException &ex)
            {
                std::cout << "Unable to get joint angles" << std::endl;
                return false;
            }
            for (auto joint_angle : input_joint_angles.joint_angles())
                kinovaInfo.jointPos[joint_angle.joint_identifier()] = joint_angle.value();
            kinova_pub.publish(kinovaInfo);

            if (status == std::future_status::ready)
                break;
        }

        base->Unsubscribe(promise_notification_handle);

        const auto promise_event = finish_future.get();
        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;
        return true;
    }
}

void torque_satuation(Matrix<double> &tau)
{
    for (int i = 0; i < 7; i++)
    {
        if (i < 4)
        {
            // other joints 0-3
            if (abs(tau[i]) > 39)
                tau[i] = tau[i] > 0 ? 39 : -39;
        }
        else
        {
            // wrist joint 4-7
            if (abs(tau[i]) > 9)
                tau[i] = tau[i] > 0 ? 9 : -9;
        }
    }
}

void gravity_compensation(const Matrix<double> &q, const double init_tau[7], Matrix<double> &tau)
{
    double G_arr[7];
    Matrix<double> G(7, 1);
    kinova_G_gripper(GRAVITY, q[0], q[1], q[2], q[3], q[4], q[5], q[6], G_arr);
    G.update_from_matlab(G_arr);
    tau[0] += G[0] + init_tau[0] * 0.05;
    tau[1] += G[1] + init_tau[1] * 0.03;
    tau[2] += G[2] + init_tau[2] * 0.1;
    tau[3] += G[3] + init_tau[3] * 0.05;
    tau[4] += G[4] + init_tau[4];
    tau[5] += G[5] * 0.95 + init_tau[5] * 0.2;
    tau[6] += G[6] + init_tau[6];
}

void q2inf(const Matrix<double> &curr_pos, const Matrix<double> &prev_q, Matrix<int> &round, Matrix<double> &q)
{
    q = curr_pos + 2 * M_PI * round;
    for (int i = 0; i < 7; i++)
    {
        if (q[i] - prev_q[i] > 330 * DEG2RAD)
        {
            q[i] -= 2 * M_PI;
            round[i]--;
        }
        else if (q[i] - prev_q[i] < -330 * DEG2RAD)
        {
            q[i] += 2 * M_PI;
            round[i]++;
        }
    }
}

void gripper_control(k_api::GripperCyclic::MotorCommand *gripper_motor_command, double finger_pitch)
{
    gripper_motor_command->set_velocity(100);
    if (finger_pitch < 0.5)
        gripper_motor_command->set_position(0); // open gripper
    else
        gripper_motor_command->set_position(100); // close gripper
}