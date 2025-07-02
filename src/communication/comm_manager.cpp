#include "comm_manager.h"
#include "nrc_comm.h"
#include <iostream>

class CommManager::Impl
{
public:
    NrcComm nrc_comm;

    bool nrc_init(const std::string&ip, const std::string& port)
    {
        return nrc_comm.connect(ip, port);
    }

    bool nrc_send_message(int msg_id, const std::string& msg)
    {
        return nrc_comm.send(msg_id, msg);
    }

    void nrc_register_callback(CommManager::MessageCallback callback)
    {
        nrc_comm.set_callback(callback);
    }

    int nrc_get_current_position_robot(int robot_numm, int coord, std::vector<double>& pos)
    {
        return nrc_comm.nrc_get_current_position_robot(robot_numm, coord, pos);
    }

    void shutdown()
    {
        nrc_comm.disconnect();
    }
};


CommManager::CommManager() : impl_(std::unique_ptr<Impl>(new Impl()))
{
}

CommManager::~CommManager() = default;

bool CommManager::nrc_init(const std::string& ip, const std::string& port)
{
    return impl_->nrc_init(ip, port);
}

bool CommManager::nrc_send_message(int message_id, const std::string& message)
{
    return impl_->nrc_send_message(message_id, message);
}

void CommManager::nrc_register_callback(MessageCallback callback)
{
    impl_->nrc_register_callback(callback);
}

int CommManager::nrc_get_current_position_robot(int robot_num, int coord, std::vector<double> &pos)
{
    return impl_->nrc_get_current_position_robot(robot_num, coord, pos);
}

void CommManager::nrc_shutdown()
{
    impl_->shutdown();
}