#include "data_logger.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <fstream>

using namespace DataLogger;

std::vector<PublisherLogger*> PublisherLogger::loggerList;
std::vector<SubscriptionLogger*> SubscriptionLogger::loggerList;

SubscriptionLogger::SubscriptionLogger(std::string topic_name)
{
    /* fix this to smart pointers */
    topic = topic_name;
    recv_counter = 0;
    next_id = 0;
    lost_count = 0;
    loggerList.push_back(this);
}
std::vector<SubscriptionLogger*> SubscriptionLogger::get_list_of_loggers()
{
    return loggerList;
}

std::vector<PublisherLogger*> PublisherLogger::get_list_of_loggers()
{
    return loggerList;
}


SubscriptionLogger::~SubscriptionLogger()
{
    auto vec = SubscriptionLogger::get_list_of_loggers();
    vec.erase(std::remove(vec.begin(), vec.end(), this), vec.end());
}
PublisherLogger::~PublisherLogger()
{
    auto vec = PublisherLogger::get_list_of_loggers();
    vec.erase(std::remove(vec.begin(), vec.end(), this), vec.end());
}


PublisherLogger::PublisherLogger(std::string topic_name)
{
    topic = topic_name;
    sent_counter = 0;
    loggerList.push_back(this);
}

uint64_t SubscriptionLogger::get_mean()
{
    uint64_t average = std::accumulate(time_diffs.begin(), time_diffs.end(), 0.0) / time_diffs.size();
    return average;
}

// https://stackoverflow.com/questions/9874802/how-can-i-get-the-maximum-or-minimum-value-in-a-vector
uint64_t SubscriptionLogger::get_max_latency()
{
    double max_latency = *std::max_element(time_diffs.begin(), time_diffs.end());
    return max_latency;
}

uint64_t SubscriptionLogger::get_min_latency()
{
    double min_latency = *std::min_element(time_diffs.begin(), time_diffs.end());
    return min_latency;
}

/** Iterates through lists of loggers, calculating latencies and stores them in separate files
 *  per logger type.
 * 
 *  @param filename The name of the file to be written to
 *  @return nothing
 * 
 */

void DataLogger::save_logged_data(std::string filename)
{
    std::ofstream fRecordedSubs;
    std::ofstream fRecordedPubs;

    auto loggers_sub = SubscriptionLogger::get_list_of_loggers();

    if(loggers_sub.size() != 0) {
        /* Assuming the code is launched from the ROS2 workspace! */
        fRecordedSubs.open("src/latency_test/logged_data/subscriptions/"+filename);
        if(!fRecordedSubs.is_open())
            throw std::runtime_error("Could not open file subs:");

        fRecordedSubs << "topic,received(#),lost(#),mean[us],max_latency[us],min_latency[us]\n";
        for (auto& logger : loggers_sub) {
            if(logger->recv_counter != 0)
                fRecordedSubs << logger->topic << "," << logger->recv_counter << "," << logger->lost_count << "," << logger->get_mean() << "," << logger->get_max_latency() << "," <<  logger->get_min_latency() << "\n";
            else
                fRecordedSubs << logger->topic << "," << logger->recv_counter << "," << ","  << "," << "\n";
        }
        fRecordedSubs.close();
    }
    auto loggers_pub = PublisherLogger::get_list_of_loggers();

    if(loggers_pub.size() != 0) {
        /* Assuming the code is launched from the ROS2 workspace! */
        fRecordedPubs.open("src/latency_test/logged_data/publishers/"+filename);
        
        if(!fRecordedPubs.is_open())
            throw std::runtime_error("Could not open file pubs:");

        fRecordedPubs << "topic,sent(#)\n";
        for (auto& logger : loggers_pub) {
            fRecordedPubs << logger->topic << "," << logger->sent_counter << "\n";
        }
        fRecordedPubs.close();
    }

    /*
    * Not implemented.
    * according to this https://github.com/irobot-ros/ros2-performance/tree/master/performances/irobot-benchmark

    SubscriptionLogger::get_late(float period)
    {
        
            min(0.2*period, 5ms)
        
    }
    SubscriptionLogger::get_too_late(float period)
    {
        
            max(period, 50ms)
        
    }
    */
}