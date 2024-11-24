#ifndef REACTIVE_BEHAVIORS
#define REACTIVE_BEHAVIORS

#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <string>

namespace reactive_behaviors
{

/**
 * @brief Custom type
 */
struct Pose2D
{
    double x, y, theta;
    // double y;
    // double theta;
};

class MoveBaseAction : public BT::StatefulActionNode
{

  public:
    /**
     * @brief Any TreeNode with ports must have this constructor signature
     */
    MoveBaseAction(const std::string &name, const BT::NodeConfig &config);

    /**
     * @brief Mandatory method
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Invoked once at the beginning
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief If onStart() returned RUNNING, this method is constantly called until it returns something other than
     * RUNNING
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Callback to execute if the action was aborted by another node
     */
    void onHalted() override;

  private:
    Pose2D _goal;
    std::chrono::system_clock::time_point _completion_time;
};

} // namespace reactive_behaviors

namespace BT
{

/**
 * @brief Template specialization to convert a string to Pose2D
 */
template <> reactive_behaviors::Pose2D convertFromString(BT::StringView str);

} // namespace BT
#endif
