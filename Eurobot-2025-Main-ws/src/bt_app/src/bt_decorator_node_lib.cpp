#include "bt_app/bt_decorator_node_lib.h"

template <> inline int BT::convertFromString(StringView str) {
    auto value = convertFromString<double>(str);
    return (int) value;
}

BT::PortsList TickFlow::providedPorts() {
    return { 
        BT::InputPort<int>("max_tick_count")
    };
}

BT::NodeStatus TickFlow::tick() {
    
    // Get the input port
    if (!getInput<int>("max_tick_count", max_tick_count)) {
        throw BT::RuntimeError("[TickFlow]: missing required input 'max_tick_count'");
    }
    
    // Tick the child node
    // Because the child() function is not thread-safe, it may be called by multiple threads at the same time.
    // So we need to lock the mutex to ensure that only one thread can call the child() function at the same time.
    BT::NodeStatus child_status;
    {
        std::unique_lock<std::mutex> lock(mutex_lock_);
        child_status = child()->executeTick();
    }

    // Increment the tick count
    tick_count++;
    
    // Check if the child is completed
    if (isStatusCompleted(child_status)) {
        resetChild();
    }

    // If flow is larger than max_tick_count, call the child to halt
    // !!! Remember: If the child is running, it will get into the "onHalted" function of the child,
    //               then you can add some cancellation logic in the "onHalted" function.
    if (tick_count > max_tick_count) {
        std::unique_lock<std::mutex> lock(mutex_lock_);

        haltChild();
        child_status = BT::NodeStatus::FAILURE;
        
        tick_count = 0;
    }

    return child_status;
}

// ElapseTimeCheck
BT::PortsList ElapseTimeCheck::providedPorts() {
    return { 
        BT::InputPort<double>("max_time_count")
    };
}

BT::NodeStatus ElapseTimeCheck::tick() {
    
    // Get the input port
    if (!getInput<double>("max_time_count", max_time_count)) {
        throw BT::RuntimeError("[ElapseTimeCheck]: missing required input 'max_time_count'");
    }

    if (time_started == false) {
        time_started = true;
        // kernel_->getCurrentTimeInSecond(start_time);
    }
    
    // Tick the child node
    // Because the child() function is not thread-safe, it may be called by multiple threads at the same time.
    // So we need to lock the mutex to ensure that only one thread can call the child() function at the same time.
    BT::NodeStatus child_status;
    {
        std::unique_lock<std::mutex> lock(mutex_lock_);
        child_status = child()->executeTick();
    }

    // Get the current time from the kernel, and calculate the elapsed time
    // kernel_->getCurrentTimeInSecond(current_time);
    double elapsed_time = current_time - start_time;
    
    // Check if the child is completed
    if (isStatusCompleted(child_status)) {
        resetChild();
    }

    // If flow is larger than max_tick_count, call the child to halt
    // !!! Remember: If the child is running, it will get into the "onHalted" function of the child,
    //               then you can add some cancellation logic in the "onHalted" function.
    if (elapsed_time > max_time_count) {
        std::unique_lock<std::mutex> lock(mutex_lock_);

        haltChild();
        child_status = BT::NodeStatus::FAILURE;

        time_started = false;
    }

    return child_status;
}

// RaceTime
BT::PortsList RaceTimeCheck::providedPorts() {
    return { 
        BT::InputPort<double>("max_time_count")
    };
}

BT::NodeStatus RaceTimeCheck::tick() {
    
    // Get the input port
    if (!getInput<double>("max_time_count", max_time_count)) {
        throw BT::RuntimeError("[ElapseTimeCheck]: missing required input 'max_time_count'");
    }
    
    // Tick the child node
    // Because the child() function is not thread-safe, it may be called by multiple threads at the same time.
    // So we need to lock the mutex to ensure that only one thread can call the child() function at the same time.
    BT::NodeStatus child_status;
    {
        std::unique_lock<std::mutex> lock(mutex_lock_);
        child_status = child()->executeTick();
    }

    // Get the race elapsed time from the kernel
    // kernel_->getRaceElapsedTime(elapsed_time);
    std::cout << "[RaceTimeCheck]: Elapsed time: " << elapsed_time << std::endl;
    
    // Check if the child is completed
    if (isStatusCompleted(child_status)) {
        resetChild();
    }

    // If flow is larger than max_tick_count, call the child to halt
    // !!! Remember: If the child is running, it will get into the "onHalted" function of the child,
    //               then you can add some cancellation logic in the "onHalted" function.
    if (elapsed_time > max_time_count) {
        std::unique_lock<std::mutex> lock(mutex_lock_);

        haltChild();
        child_status = BT::NodeStatus::FAILURE;
    }

    return child_status;
}