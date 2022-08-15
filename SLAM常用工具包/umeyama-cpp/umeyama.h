/*  Copyright (C) <2020>  <Yong WU>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UMEYAMA_UMEYAMA_H_
#define UMEYAMA_UMEYAMA_H_

#include <opencv2/opencv.hpp>

namespace utils {
cv::Mat Umeyama(const cv::Mat& src, const cv::Mat& dst, bool with_scale);
namespace internal {
static cv::Mat MeanRow(const cv::Mat& src);
static cv::Mat DemeanRow(const cv::Mat& src, const cv::Mat& mean);
}  // namespace internal
}  // namespace utils

#endif  // UMEYAMA_UMEYAMA_H_
