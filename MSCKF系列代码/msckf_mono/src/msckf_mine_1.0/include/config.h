#ifndef CONFIG_H
#define CONFIG_H

#include "common_include.h"

namespace MSCKF_MINE
{

    class Config
    {
    private:
        static std::shared_ptr<Config>  config_;
        cv::FileStorage                 file_;
        Config() {}

    public:
        ~Config();

        /*set a new config file*/
        static void setParameterFile( const std::string& filename);

        /*access the parameter values*/
        template<typename T>
        static T get(const std::string& key)
        {
            return T(Config::config_->file_[key]);
        }

    };
}









#endif // CONFIG_H
