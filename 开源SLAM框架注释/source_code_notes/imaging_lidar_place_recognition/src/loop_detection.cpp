#include "loop_detection.h"

LoopDetector::LoopDetector(){}


void LoopDetector::loadVocabulary(std::string voc_path)
{
    voc = new DBoW3::Vocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void LoopDetector::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    int loop_index = -1;
    if (flag_detect_loop)
    {
        // 检测回环，并返回回环的关键帧索引
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        // 如果不检测回环，则不断的向DBoW数据库中添加新的关键帧，扩充数据库
        addKeyFrameIntoVoc(cur_kf);
    }

    // check loop if valid using ransan and pnp
    // 检查回环帧与当前关键帧之间的pnp误差是否小于给定阈值，避免误回环
    if (loop_index != -1)
    {
        KeyFrame* old_kf = getKeyFrame(loop_index); // 获取回环关键帧

        if (abs(cur_kf->time_stamp - old_kf->time_stamp) > MIN_LOOP_SEARCH_TIME) // 确定回环帧与当前关键帧之间的时间差超过了给定的阈值
        {
            // 利用ransac pnp方法，检测当前关键帧和回环关键帧之间的匹配是否有效
            if (cur_kf->findConnection(old_kf))
            {
                std_msgs::Float64MultiArray match_msg;
                match_msg.data.push_back(cur_kf->time_stamp);
                match_msg.data.push_back(old_kf->time_stamp);
                pub_match_msg.publish(match_msg);

                index_match_container[cur_kf->index] = old_kf->index; // 记录回环信息
            }
        }
    }

    // add keyframe
    cur_kf->freeMemory(); // 释放内存
    keyframelist.push_back(cur_kf);
}

KeyFrame* LoopDetector::getKeyFrame(int index)
{
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int LoopDetector::detectLoop(KeyFrame* keyframe, int frame_index)
{
    //first query; then add this frame into database!
    DBoW3::QueryResults ret;
    // 在DBoW3数据库中检索当前图像的描述子
    db.query(keyframe->bow_descriptors, ret, 4, frame_index - MIN_LOOP_SEARCH_GAP);
    // 向DBoW3数据库中添加新的当前检索图像的描述子
    db.add(keyframe->bow_descriptors); // ret[0] is the nearest neighbour's score. threshold change with neighour score

    // 显示用(跳过)
    if (DEBUG_IMAGE)
    {
        image_pool[frame_index] = keyframe->image.clone();

        cv::Mat bow_images = keyframe->image.clone();

        if (ret.size() > 0)
            putText(bow_images, "Index: " + to_string(frame_index), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);

        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "Index:  " + to_string(tmp_index) + ", BoW score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);
            cv::vconcat(bow_images, tmp_image, bow_images);
        }

        cv::imshow("BoW images", bow_images);
        cv::waitKey(10);
    }

    // 如果当前关键帧索引小于最小回环检测间隔阈值，则直接退出
    if (frame_index - MIN_LOOP_SEARCH_GAP < 0)
        return -1;
    
    // a good match with its nerghbour
    // 这里做进一步验证，如果检测到满足回环要求的关键帧，那它相邻的关键帧的得分也应该满足回环要求(原因是因为相邻关键帧差异很小，这个条件应该是满足的)
    bool find_loop = false;
    if (ret.size() >= 1 && ret[0].Score > MIN_LOOP_BOW_TH)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            if (ret[i].Score > MIN_LOOP_BOW_TH)
            {          
                find_loop = true;
            }
        }
    }
    
    // 返回满足回环要求的所有关键帧中的索引最小的关键帧
    if (find_loop && frame_index > 5)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || ((int)ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;
}

void LoopDetector::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    if (DEBUG_IMAGE)
    {
        image_pool[keyframe->index] = keyframe->image.clone();
    }

    db.add(keyframe->bow_descriptors);
}