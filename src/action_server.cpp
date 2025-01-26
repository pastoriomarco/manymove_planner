#include "manymove_planner/planner_interface.hpp"
#include "manymove_planner/action/plan_manipulator.hpp"
#include "manymove_planner/action/move_manipulator.hpp"
#include "manymove_planner/action/move_manipulator_sequence.hpp"
#include "manymove_planner/action/execute_trajectory.hpp"
#include "manymove_planner/action/unload_traj_controller.hpp"
#include "manymove_planner/action/load_traj_controller.hpp"

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>

class MoveManipulatorActionServer
{
public:
    using MoveManipulator = manymove_planner::action::MoveManipulator;
    using MoveManipulatorSequence = manymove_planner::action::MoveManipulatorSequence;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;
    using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<MoveManipulatorSequence>;

    using PlanManipulator = manymove_planner::action::PlanManipulator;
    using GoalHandlePlanManipulator = rclcpp_action::ServerGoalHandle<PlanManipulator>;

    using ExecuteTrajectory = manymove_planner::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    using UnloadTrajController = manymove_planner::action::UnloadTrajController;
    using GoalHandleUnloadTrajController = rclcpp_action::ServerGoalHandle<UnloadTrajController>;

    using LoadTrajController = manymove_planner::action::LoadTrajController;
    using GoalHandleLoadTrajController = rclcpp_action::ServerGoalHandle<LoadTrajController>;

    MoveManipulatorActionServer(const rclcpp::Node::SharedPtr &node,
                                const std::shared_ptr<PlannerInterface> &planner,
                                const std::string &planner_prefix = "")
        : node_(node), planner_(planner), planner_prefix_(planner_prefix)
    {
        // Create Reentrant Callback Groups
        action_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        param_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Initialize Service Clients
        unload_controller_client_ = node_->create_client<controller_manager_msgs::srv::UnloadController>(
            "/controller_manager/unload_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        load_controller_client_ = node_->create_client<controller_manager_msgs::srv::LoadController>(
            "/controller_manager/load_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        configure_controller_client_ = node_->create_client<controller_manager_msgs::srv::ConfigureController>(
            "/controller_manager/configure_controller",
            rmw_qos_profile_services_default,
            param_callback_group_);

        // get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(
        //     "lite6_traj_controller/get_parameters",
        //     rmw_qos_profile_services_default,
        // param_callback_group_);

        // set_parameters_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
        //     "lite6_traj_controller/set_parameters",
        //     rmw_qos_profile_services_default,
        //     param_callback_group_);

        // **Wait for Services to Be Available**
        bool all_services_available = true;

        if (!unload_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/unload_controller' not available.");
            all_services_available = false;
        }

        if (!load_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/load_controller' not available.");
            all_services_available = false;
        }

        if (!switch_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/switch_controller' not available.");
            all_services_available = false;
        }

        if (!configure_controller_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '/controller_manager/configure_controller' not available.");
            all_services_available = false;
        }

        // if (!get_parameters_client_->wait_for_service(std::chrono::seconds(5)))
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Service 'lite6_traj_controller/get_parameters' not available.");
        //     all_services_available = false;
        // }

        // if (!set_parameters_client_->wait_for_service(std::chrono::seconds(5)))
        // {
        //     RCLCPP_ERROR(node_->get_logger(), "Service 'lite6_traj_controller/set_parameters' not available.");
        //     all_services_available = false;
        // }

        if (!all_services_available)
        {
            RCLCPP_ERROR(node_->get_logger(), "Not all required services are available. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Initialize Action Servers with Reentrant Callback Group
        single_action_server_ = rclcpp_action::create_server<MoveManipulator>(
            node_,
            planner_prefix_ + "move_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_single_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_single_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_single_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        sequence_action_server_ = rclcpp_action::create_server<MoveManipulatorSequence>(
            node_,
            planner_prefix_ + "move_manipulator_sequence",
            std::bind(&MoveManipulatorActionServer::handle_sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_sequence_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_sequence_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        plan_action_server_ = rclcpp_action::create_server<PlanManipulator>(
            node_,
            planner_prefix_ + "plan_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_plan_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_plan_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_plan_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        execute_action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            node_,
            planner_prefix_ + "execute_manipulator_traj",
            std::bind(&MoveManipulatorActionServer::handle_execute_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_execute_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_execute_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        stop_motion_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            node_,
            planner_prefix_ + "stop_motion",
            std::bind(&MoveManipulatorActionServer::handle_stop_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_stop_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_stop_accept, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        unload_traj_controller_server_ = rclcpp_action::create_server<UnloadTrajController>(
            node_,
            planner_prefix_ + "unload_trajectory_controller",
            std::bind(&MoveManipulatorActionServer::handle_unload_traj_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_unload_traj_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_unload_traj_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);

        load_traj_controller_server_ = rclcpp_action::create_server<LoadTrajController>(
            node_,
            planner_prefix_ + "load_trajectory_controller",
            std::bind(&MoveManipulatorActionServer::handle_load_traj_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_load_traj_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_load_traj_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_callback_group_);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PlannerInterface> planner_;
    std::string planner_prefix_;

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;
    rclcpp::CallbackGroup::SharedPtr param_callback_group_;

    // Action Servers
    rclcpp_action::Server<MoveManipulator>::SharedPtr single_action_server_;
    rclcpp_action::Server<MoveManipulatorSequence>::SharedPtr sequence_action_server_;

    rclcpp_action::Server<PlanManipulator>::SharedPtr plan_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr execute_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr stop_motion_server_;
    rclcpp_action::Server<UnloadTrajController>::SharedPtr unload_traj_controller_server_;
    rclcpp_action::Server<LoadTrajController>::SharedPtr load_traj_controller_server_;

    // Service Clients
    rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;

    // -------------------------------------
    // MoveManipulator callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_single_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        [[maybe_unused]] std::shared_ptr<const MoveManipulator::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received single MoveManipulator goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_single_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel single move goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_single_accepted(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        // Run in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_single, this, goal_handle)}.detach();
    }

    void execute_single(const std::shared_ptr<GoalHandleMoveManipulator> &goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing single MoveManipulator goal");
        auto result = std::make_shared<MoveManipulator::Result>();
        const auto &goal_msg = goal_handle->get_goal();

        // Plan single move
        auto [success, trajectory] = planner_->plan(*goal_msg);
        if (!success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed for single move.");
            result->success = false;
            result->message = "Planning failed";
            goal_handle->abort(result);
            return;
        }

        // Apply time parameterization
        std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
        std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {goal_msg->goal.config};
        std::vector<size_t> sizes;
        auto [param_success, final_trajectory] = planner_->applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

        if (!param_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed for single move.");
            result->success = false;
            result->message = "Time parametrization failed";
            goal_handle->abort(result);
            return;
        }

        // Execute WITHOUT feedback since it's a single move
        bool exec_success = planner_->executeTrajectory(final_trajectory);
        if (exec_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Single move succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Single move failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // MoveManipulatorSequence callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_sequence_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveManipulatorSequence::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received MoveManipulatorSequence goal with %zu moves", goal->goals.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_sequence_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleMoveManipulatorSequence> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel sequence goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_sequence_accepted(const std::shared_ptr<GoalHandleMoveManipulatorSequence> goal_handle)
    {
        // Run in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_sequence, this, goal_handle)}.detach();
    }

    void execute_sequence(std::shared_ptr<GoalHandleMoveManipulatorSequence> &goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing sequence goal");
        auto result = std::make_shared<MoveManipulatorSequence::Result>();
        const auto &goals_msg = goal_handle->get_goal();

        // Plan the sequence
        RCLCPP_INFO(node_->get_logger(), "Planning Sequence");
        auto [trajectories, configs] = planner_->planSequence(*goals_msg);

        if (trajectories.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning sequence failed.");
            result->success = false;
            result->message = "Planning sequence failed.";
            goal_handle->abort(result);
            return;
        }

        // Time parameterization
        std::vector<size_t> sizes;
        auto [param_success, final_trajectory] = planner_->applyTimeParametrizationSequence(trajectories, configs, sizes);

        if (!param_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time parametrization sequence failed.");
            result->success = false;
            result->message = "Time parametrization sequence failed.";
            goal_handle->abort(result);
            return;
        }

        // Execute WITH feedback for the sequence
        bool exec_success = planner_->executeTrajectoryWithFeedback(final_trajectory, sizes, goal_handle);
        if (exec_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Sequence goal succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Sequence goal failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // PlanManipulator callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_plan_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        [[maybe_unused]] std::shared_ptr<const PlanManipulator::Goal> goal_msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Received PlanManipulator action goal request");
        // Optionally validate goal_msg->goal ...
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_plan_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel PlanManipulator goal");
        // For a pure planning action, you might accept or reject cancellation
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_plan_accepted(
        const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        // Spin off a new thread to do planning so we don't block the executor
        std::thread{std::bind(&MoveManipulatorActionServer::execute_plan_goal, this, goal_handle)}.detach();
    }

    void execute_plan_goal(
        const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        auto result = std::make_shared<PlanManipulator::Result>();
        const auto &goal_msg = goal_handle->get_goal(); // plan_manipulator::Goal

        // 1) Actually plan
        manymove_planner::action::MoveManipulator::Goal internal_goal;
        internal_goal.goal = goal_msg->goal; // reusing the existing "MoveManipulatorGoal"

        auto [success, trajectory] = planner_->plan(internal_goal);
        if (!success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed for plan_manipulator action");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 2) Time parameterization
        std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
        std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {goal_msg->goal.config};
        std::vector<size_t> sizes;

        auto [param_success, final_trajectory] =
            planner_->applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

        if (!param_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time param failed for plan_manipulator action");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // If you want to send feedback in real-time, you can do so in applyTimeParameterization() or
        //   break it into steps. For simplicity, let's just finalize the result:
        result->success = true;
        result->trajectory = final_trajectory;

        RCLCPP_INFO(node_->get_logger(), "PlanManipulator action Succeeded");
        goal_handle->succeed(result);
    }

    // -------------------------------------
    // ExecuteTrajectory callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_execute_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        if (goal->trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Received an empty trajectory");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(node_->get_logger(), "Received ExecuteTrajectory goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_execute_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel trajectory execution");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_execute_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        // Run in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_trajectory, this, goal_handle)}.detach();
    }

    void execute_trajectory(const std::shared_ptr<GoalHandleExecuteTrajectory> &goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing trajectory");
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        const auto &goal_msg = goal_handle->get_goal();

        // Execute the trajectory
        bool exec_success = planner_->executeTrajectory(goal_msg->trajectory);

        if (exec_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Trajectory execution succeeded");
            result->success = true;
            result->message = "Trajectory execution complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory execution failed");
            result->success = false;
            result->message = "Trajectory execution failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // StopMotion callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_stop_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        [[maybe_unused]] std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Received request to STOP");
        // Always accept the request:
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_stop_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Received request to cancel STOP motion");
        // Typically we just accept the cancel, though there's not much sense in "canceling a stop."
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_stop_accept(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        // Execute in a separate thread so as not to block the executor
        std::thread{std::bind(&MoveManipulatorActionServer::execute_stop, this, goal_handle)}.detach();
    }

    void execute_stop(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Executing STOP motion...");
        auto result = std::make_shared<ExecuteTrajectory::Result>();

        // 1) Check if canceled before we start
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(node_->get_logger(), "[ExecuteTrajectory] STOP goal canceled before execution.");
            result->success = false;
            result->message = "Canceled before execution";
            goal_handle->canceled(result);
            return;
        }

        // 2) Actually send the short stop trajectory
        double dec_time = 0.5;
        bool ok = planner_->sendControlledStop(dec_time);

        // 3) If canceled mid-stop
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(node_->get_logger(), "[ExecuteTrajectory] STOP goal canceled mid‐execution.");
            result->success = false;
            result->message = "Canceled mid‐execution";
            goal_handle->canceled(result);
            return;
        }

        // 4) Finalize
        if (ok)
        {
            RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] STOP completed successfully.");
            result->success = true;
            result->message = "Stop complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "[ExecuteTrajectory] STOP motion failed or was aborted by the controller.");
            result->success = false;
            result->message = "Stop motion failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // UnloadTrajController callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_unload_traj_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const UnloadTrajController::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received UnloadTrajController goal request for controller: %s",
                    goal->controller_name.c_str());

        if (goal->controller_name.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Controller name is empty. Rejecting.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_unload_traj_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleUnloadTrajController> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received request to CANCEL UnloadTrajController goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_unload_traj_accepted(
        const std::shared_ptr<GoalHandleUnloadTrajController> goal_handle)
    {
        std::thread{std::bind(&MoveManipulatorActionServer::execute_unload_traj_controller, this, goal_handle)}.detach();
    }

    void execute_unload_traj_controller(
        const std::shared_ptr<GoalHandleUnloadTrajController> &goal_handle)
    {
        auto result = std::make_shared<UnloadTrajController::Result>();
        std::string controller_name = goal_handle->get_goal()->controller_name;

        // 1) Deactivate the controller
        deactivateControllerAsync(
            controller_name,
            [this, goal_handle, result, controller_name]()
            {
                // 2) Unload the controller
                unloadControllerAsync(
                    controller_name,
                    [this, goal_handle, result, controller_name]()
                    {
                        result->success = true;
                        result->message = "Controller unloaded successfully.";
                        goal_handle->succeed(result);
                    },
                    [goal_handle, result, controller_name](const std::string &err)
                    {
                        result->success = false;
                        result->message = "Unload error: " + err;
                        goal_handle->abort(result);
                    });
            },
            [goal_handle, result, controller_name](const std::string &err)
            {
                result->success = false;
                result->message = "Deactivate error: " + err;
                goal_handle->abort(result);
            });
    }

    // -------------------------------------
    // LoadTrajController callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_load_traj_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const LoadTrajController::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received LoadTrajController goal request for controller: %s",
                    goal->controller_name.c_str());

        if (goal->controller_name.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Controller name is empty. Rejecting.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_load_traj_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleLoadTrajController> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received request to CANCEL LoadTrajController goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_load_traj_accepted(
        const std::shared_ptr<GoalHandleLoadTrajController> goal_handle)
    {
        std::thread{std::bind(&MoveManipulatorActionServer::execute_load_traj_controller, this, goal_handle)}.detach();
    }

    void execute_load_traj_controller(
        const std::shared_ptr<GoalHandleLoadTrajController> &goal_handle)
    {
        auto result = std::make_shared<LoadTrajController::Result>();
        std::string controller_name = goal_handle->get_goal()->controller_name;

        // 1) Load the controller
        loadControllerAsync(
            controller_name,
            [this, goal_handle, result, controller_name]()
            {
                // 2) Configure the controller
                configureControllerAsync(
                    controller_name,
                    [this, goal_handle, result, controller_name]()
                    {
                        // 3) Activate the controller
                        activateControllerAsync(
                            controller_name,
                            [this, goal_handle, result, controller_name]()
                            {
                                result->success = true;
                                result->message = "Controller loaded and activated successfully.";
                                goal_handle->succeed(result);
                            },
                            [goal_handle, result, controller_name](const std::string &err)
                            {
                                result->success = false;
                                result->message = "Activate error: " + err;
                                goal_handle->abort(result);
                            });
                    },
                    [goal_handle, result, controller_name](const std::string &err)
                    {
                        result->success = false;
                        result->message = "Configure error: " + err;
                        goal_handle->abort(result);
                    });
            },
            [goal_handle, result, controller_name](const std::string &err)
            {
                result->success = false;
                result->message = "Load error: " + err;
                goal_handle->abort(result);
            });
    }

    // -------------------------------------
    // Helpers:
    // -------------------------------------

    void unloadControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[unloadControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        if (!unload_controller_client_->service_is_ready())
        {
            std::string msg = "[unloadControllerAsync] Service '/controller_manager/unload_controller' unavailable";
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
        request->name = controller_name;

        unload_controller_client_->async_send_request(request,
                                                      [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedFuture future_response)
                                                      {
                                                          auto response = future_response.get();
                                                          if (!response)
                                                          {
                                                              std::string msg = "[unloadControllerAsync] Null response for " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                              return;
                                                          }

                                                          if (response->ok)
                                                          {
                                                              RCLCPP_INFO(node_->get_logger(), "[unloadControllerAsync] SUCCESS: Unloaded '%s'", controller_name.c_str());
                                                              on_success();
                                                          }
                                                          else
                                                          {
                                                              std::string msg = "[unloadControllerAsync] Failed to unload " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                          }
                                                      });

        RCLCPP_INFO(node_->get_logger(), "[unloadControllerAsync] Request sent, awaiting response...");
    }

    void loadControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[loadControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        if (!load_controller_client_->service_is_ready())
        {
            std::string msg = "[loadControllerAsync] Service '/controller_manager/load_controller' unavailable for: " + controller_name;
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
        request->name = controller_name;

        load_controller_client_->async_send_request(request,
                                                    [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedFuture future_response)
                                                    {
                                                        auto response = future_response.get();
                                                        if (!response)
                                                        {
                                                            std::string msg = "[loadControllerAsync] Null response for " + controller_name;
                                                            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                            on_error(msg);
                                                            return;
                                                        }

                                                        if (response->ok)
                                                        {
                                                            RCLCPP_INFO(node_->get_logger(), "[loadControllerAsync] SUCCESS: Loaded '%s'", controller_name.c_str());
                                                            on_success();
                                                        }
                                                        else
                                                        {
                                                            std::string msg = "[loadControllerAsync] Failed to load " + controller_name;
                                                            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                            on_error(msg);
                                                        }
                                                    });

        RCLCPP_INFO(node_->get_logger(), "[loadControllerAsync] Request sent, awaiting response...");
    }

    void activateControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[activateControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        if (!switch_controller_client_->service_is_ready())
        {
            std::string msg = "[activateControllerAsync] Service '/controller_manager/switch_controller' unavailable";
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.push_back(controller_name);
        request->deactivate_controllers.clear();
        request->strictness = request->STRICT;
        request->start_asap = false;
        request->timeout.sec = 0;
        request->timeout.nanosec = 0;

        switch_controller_client_->async_send_request(request,
                                                      [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response)
                                                      {
                                                          RCLCPP_INFO(node_->get_logger(), "[activateControllerAsync] Received response for '%s'.", controller_name.c_str());

                                                          auto response = future_response.get();
                                                          if (!response)
                                                          {
                                                              std::string msg = "[activateControllerAsync] Null response for " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                              return;
                                                          }

                                                          if (response->ok)
                                                          {
                                                              RCLCPP_INFO(node_->get_logger(), "[activateControllerAsync] SUCCESS: Activated '%s'", controller_name.c_str());
                                                              on_success();
                                                          }
                                                          else
                                                          {
                                                              std::string msg = "[activateControllerAsync] Failed to activate " + controller_name;
                                                              RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                                                              on_error(msg);
                                                          }
                                                      });

        RCLCPP_INFO(node_->get_logger(), "[activateControllerAsync] Request sent, awaiting response...");
    }

    void deactivateControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[deactivateControllerAsync] Called for controller: '%s'",
                    controller_name.c_str());

        // Check if the service is ready
        if (!switch_controller_client_->service_is_ready())
        {
            std::string msg = "[deactivateControllerAsync] Service '/controller_manager/switch_controller' unavailable";
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        // Prepare the SwitchController request
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.clear();                      // We are not starting any new controllers
        request->deactivate_controllers.push_back(controller_name); // We want to deactivate/stop this controller
        request->strictness = request->STRICT;                      // STRICT means all requested switches must succeed
        request->start_asap = false;                                // You can set this to true or false depending on your usage
        request->timeout.sec = 0;                                   // Timeout for the switch
        request->timeout.nanosec = 0;

        // Send asynchronous request
        switch_controller_client_->async_send_request(
            request,
            [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[deactivateControllerAsync] Received response for '%s'.",
                            controller_name.c_str());

                auto response = future_response.get();
                if (!response)
                {
                    std::string msg = "[deactivateControllerAsync] Null response for " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                    return;
                }

                if (response->ok)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "[deactivateControllerAsync] SUCCESS: Deactivated '%s'",
                                controller_name.c_str());
                    on_success();
                }
                else
                {
                    std::string msg = "[deactivateControllerAsync] Failed to deactivate " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                }
            });

        RCLCPP_INFO(node_->get_logger(),
                    "[deactivateControllerAsync] Request sent, awaiting response...");
    }

    void configureControllerAsync(
        const std::string &controller_name,
        std::function<void()> on_success,
        std::function<void(const std::string &)> on_error)
    {
        RCLCPP_INFO(node_->get_logger(), "[configureControllerAsync] Configuring '%s'", controller_name.c_str());

        if (!configure_controller_client_->service_is_ready())
        {
            std::string msg = "[configureControllerAsync] Service unavailable for " + controller_name;
            RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
            on_error(msg);
            return;
        }

        auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
        request->name = controller_name;

        configure_controller_client_->async_send_request(
            request,
            [this, controller_name, on_success, on_error](rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedFuture future)
            {
                auto response = future.get();
                if (!response)
                {
                    std::string msg = "[configureControllerAsync] Null response for " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                    return;
                }

                if (response->ok)
                {
                    RCLCPP_INFO(node_->get_logger(), "[configureControllerAsync] Successfully configured '%s'", controller_name.c_str());
                    on_success();
                }
                else
                {
                    std::string msg = "[configureControllerAsync] Failed to configure " + controller_name;
                    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
                    on_error(msg);
                }
            });
    }
};
