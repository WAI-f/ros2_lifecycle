#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <unordered_map>

typedef typename rclcpp::Client<lifecycle_msgs::srv::GetState> GetStateClient;
typedef typename lifecycle_msgs::msg::State LifecycleState;

void get_node_name_namespace(const std::string &fully_name, std::string &name, std::string &name_space)
{
    auto pos = fully_name.rfind("/");
    if(pos!=std::string::npos)
    {
        name = fully_name.substr(pos+1, std::string::npos);
        name_space = fully_name.substr(0, pos);
    }
    else
    {
        name = fully_name.substr(0, 1);
    }
}

void get_lifecycle_clients(rclcpp::Node::SharedPtr utility_node, 
                            std::unordered_map<std::string, std::shared_ptr<GetStateClient>> &lifecycle_client)
{
    for(const auto &kv:lifecycle_client)
    {
        lifecycle_client[kv.first] = nullptr;
    }

    auto node_names = utility_node->get_node_names();
    for(auto &fully_name: node_names)
    {
        std::string node_name;
        std::string node_namespace;
        get_node_name_namespace(fully_name, node_name, node_namespace);

        auto services_name_types = utility_node->get_service_names_and_types_by_node(node_name, node_namespace);
        bool lifecycle_found = false;
        for(const auto &kv:services_name_types)
        {
            for(const auto &type_name :kv.second)
            {
                if(type_name.find("lifecycle")!=std::string::npos)
                {
                    std::string node_name;
                    std::string node_namespace;
                    get_node_name_namespace(fully_name, node_name, node_namespace);

                    std::string service_name = fully_name+"/get_state";
                    lifecycle_client[fully_name] = utility_node->create_client<lifecycle_msgs::srv::GetState>(service_name);
                
                    lifecycle_found = true;
                    break;
                }
            }

            if(lifecycle_found) break;
        }
    }
}

void request_lifecycle_node_state(const std::string fully_name,
                                    std::unordered_map<std::string, std::shared_ptr<GetStateClient>> &lifecycle_client,
                                    std::unordered_map<std::string, LifecycleState> &lifecycle_node_states)
{
    auto client = lifecycle_client[fully_name];
    if(!client)
    {
        lifecycle_node_states[fully_name] = lifecycle_msgs::msg::State();
        return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    client->async_send_request(request, [&lifecycle_node_states, fully_name](GetStateClient::SharedFuture response){
                lifecycle_node_states[fully_name] = response.get()->current_state;
            });
}

void monitor(rclcpp::Node::SharedPtr utility_node, 
            std::unordered_map<std::string, std::shared_ptr<GetStateClient>> &lifecycle_client,
            std::unordered_map<std::string, LifecycleState> &lifecycle_node_states)
{
    while(true)
    {
        get_lifecycle_clients(utility_node, lifecycle_client);
        for(const auto &kv:lifecycle_client)
        {
            request_lifecycle_node_state(kv.first, lifecycle_client, lifecycle_node_states);
            std::cout<<kv.first<<":"<<lifecycle_node_states[kv.first].label<<std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);   

    rclcpp::Node::SharedPtr utility_node_ = rclcpp::Node::make_shared("lifecycle_monitor");
    std::unordered_map<std::string, LifecycleState> lifecycle_node_states_;
    std::unordered_map<std::string, std::shared_ptr<GetStateClient>> lifecycle_client_;


    std::shared_ptr<std::thread> spin_thread = std::make_shared<std::thread>([utility_node_](){
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(utility_node_);
        executor.spin();
    });
    
    monitor(utility_node_, lifecycle_client_, lifecycle_node_states_);

    return 0;
}