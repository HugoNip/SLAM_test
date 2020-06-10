#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include "common_include.h"

namespace myslam {
    /**
     * use SetParameterFile for configure
     * then, use Get to get value
     * singleton
     */
    class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {}

    public:
        ~Config();

        // set a new config file
        static bool SetParameterFile(const std::string &filename);

        // access the parameter values
        template<typename T>
        static T Get(const std::string &key) {
            std::cout << std::string(Config::config_->file_[key]) << std::endl;
            return T(Config::config_->file_[key]);
        }
    };
} // namespace myslam

#endif // CONFIG_H
