#include <chrono>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait()
    // to wait for and receive new messages and pull them from the queue using move semantics.
    // The received object should then be returned by the receive function.
    std::unique_lock<std::mutex> ulck(_mutex);
    _cond.wait(ulck, [this](){ return !_queue.empty(); });
    T msg = std::move(_queue.back());
    _queue.clear();  // reference: https://knowledge.udacity.com/questions/98313
    return msg;
}

template <typename T>
void MessageQueue<T>::send(T&& msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex>
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
    std::lock_guard<std::mutex> lck(_mutex);
    _queue.push_back(std::move(msg));
    _cond.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop
    // runs and repeatedly calls the receive function on the message queue.
    // Once it receives TrafficLightPhase::green, the method returns.
    while (true) {
        TrafficLightPhase phase = _queue.receive();
        if (phase == TrafficLightPhase::green) {
            break;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    std::lock_guard<std::mutex> lck(_mutex);
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method "cycleThroughPhases" should be started in a thread
    // when the public method "simulate" is called. To do this, use the thread queue in the base class.
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// generate a random duration between traffic light cycles
double TrafficLight::getCycleDuration()
{
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_real_distribution<double> distr(4.0, 6.0);
    return distr(eng);
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles
    // and toggles the current phase of the traffic light between red and green
    // and sends an update method to the message queue using move semantics.
    // The cycle duration should be a random value between 4 and 6 seconds.
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles.

    double cycleDuration = getCycleDuration();  // duration of a single traffic light cycle in second
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    lastUpdate = std::chrono::system_clock::now();
    while (true) {
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate > cycleDuration) {
            std::lock_guard<std::mutex> lck(_mutex);
            // toggle the current phase of traffice light
            TrafficLightPhase nextPhase = (_currentPhase == TrafficLightPhase::green)
                                        ? TrafficLightPhase::red : TrafficLightPhase::green;
            _currentPhase = nextPhase;
            lastUpdate = std::chrono::system_clock::now();
            // send an update to the message queue
            _queue.send(std::move(nextPhase));
        }
    }
}
