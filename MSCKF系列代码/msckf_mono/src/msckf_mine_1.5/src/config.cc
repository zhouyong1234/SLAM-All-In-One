#include "config.h"

namespace MSCKF_MINE
{
    void Config::setParameterFile(const string &filename)
    {
        if( config_ == nullptr )
        {
            config_ = shared_ptr<Config> (new Config);
            config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
            if( config_->file_.isOpened() == false )
            {
                std::cerr << "parameter file " << filename << " does not exist.." << std::endl;
                config_->file_.release();
                return;
            }
        }

    }

    Config::~Config()
    {
        if( file_.isOpened() )
        {
            file_.release();
        }
    }

    Mat Config::GetTbs()
    {
        /*Tbs*/
        string Tbs = this->get<string>("T_BS");
        //cout << BOLDCYAN"Camera.Tbs = " << Tbs << WHITE << endl;

        stringstream ss(Tbs);
        Mat tbs(4,4,CV_64F);
        for(int i = 0; i <4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                ss >> tbs.at<double>(i,j);
            }
        }

        return tbs.clone();
    }

    shared_ptr<Config> Config::config_ = nullptr;
}


