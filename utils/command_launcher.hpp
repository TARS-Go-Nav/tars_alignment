#pragma once

#include <iostream>
#include <memory>
#include <array>
#include <vector>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <signal.h>

struct LaunchConfig {
    std::string workspace_path;
    std::string package_name;
    std::string launch_file;
    std::string setup_file; // setup.bash or setup.zsh
};

class CommandLauncher {
public:
    explicit CommandLauncher() {}

    bool startLaunch(const LaunchConfig& config) {
        
        if (!validateWorkspace(config)) {
            std::cerr << "Workspace validation failed" << std::endl;
            return false;
        }
        
        try {
            std::string command = buildLaunchCommand(config);
            return executeTempBashCommand(command);
        } catch (const std::exception& e) {
            std::cerr << "Exception occurred: " << e.what() << std::endl;
            return false;
        }
    }

    bool executeExistedBashCommand(const std::string& command) {
        std::string cmd = "gnome-terminal -t \"GUI\" -- bash -c \"./" + command + "; exec bash\"";
        int status = system((cmd).c_str());
        
        if (status != 0) {
            std::cerr << "Command failed with status " << status << std::endl;
            return false;
        }
        
        return true;
    }

private:
    bool validateWorkspace(const LaunchConfig& config) {
        if (!std::filesystem::exists(config.workspace_path)) {
            std::cerr << "Workspace path does not exist: " << config.workspace_path << std::endl;
            return false;
        }
        
        std::string setup_path = config.workspace_path + "/install/" + config.setup_file;
        if (!std::filesystem::exists(setup_path)) {
            std::cerr << "Setup file does not exist: " << setup_path << std::endl;
            return false;
        }
        
        return true;
    }

    std::string buildLaunchCommand(const LaunchConfig& config) {
        std::stringstream cmd;

        cmd << "gnome-terminal -t \"GUI\" -- bash -c"
            << "\"cd " << config.workspace_path << ";"
            << "source install/" << config.setup_file << ";"
            << "ros2 launch " << config.package_name << " " << config.launch_file << ";"
            << "exec bash";
        
        return cmd.str();
    }

    bool executeTempBashCommand(const std::string& command) {
        std::string shell = "bash";
        
        std::string temp_script = "/home/eatwhat/tmp_launch_script_" + std::to_string(std::time(nullptr));
        std::ofstream script_file(temp_script);
        script_file << "#!/bin/" << shell << "\n";
        script_file << command << "\n";
        script_file.close();
        
        std::string chmod_cmd = "chmod +x " + temp_script;
        system(chmod_cmd.c_str());
        
        int status = system((temp_script).c_str());
        
        std::remove(temp_script.c_str());
        
        if (status != 0) {
            std::cerr << "Command failed with status " << status << std::endl;
            return false;
        }
        
        return true;
    }

};

bool killExistingProcess(const std::string& package_name, const std::string& launch_file) {
    // Find all possible process IDs
    std::string find_cmd = "pgrep -f '" + package_name + "'";
    
    // Execute search command and get output
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(find_cmd.c_str(), "r"), pclose);
    
    if (!pipe) {
        std::cerr << "Failed to search for processes" << std::endl;
        return false;
    }
    
    // Read process IDs
    std::vector<int> pids;
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        int pid = std::stoi(buffer.data());
        pids.push_back(pid);
    }
    
    if (pids.empty()) {
        std::cout << "No related processes found" << std::endl;
        return true; // Nothing to kill is also considered success
    }
    
    // Attempt to terminate each found process
    bool all_killed = true;
    for (int pid : pids) {
        if (kill(pid, SIGTERM) != 0) {
            std::cerr << "Failed to send SIGTERM to process " << pid << ": " << strerror(errno) << std::endl;
            
            if (kill(pid, SIGKILL) != 0) {
                std::cerr << "Failed to send SIGKILL to process " << pid << ": " << strerror(errno) << std::endl;
                all_killed = false;
            }
        }
    }
    
    // Wait a moment to ensure processes have terminated
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Verify whether processes have actually exited
    for (int pid : pids) {
        if (kill(pid, 0) == 0) {
            std::cerr << "Process " << pid << " still exists" << std::endl;
            all_killed = false;
        }
    }
    
    return all_killed;
}
