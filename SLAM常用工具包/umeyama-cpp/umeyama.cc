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
#include "umeyama.h"

namespace utils {
cv::Mat Umeyama(const cv::Mat& src, const cv::Mat& dst, bool with_scale) {
  /* Mat layout
   * |x1, x2, x3, x4|
   * |y1, y2, y3, y4|
   */
  const int m = src.rows;  // dimension
  const int n = src.cols;  // number of measurements
  cv::Mat src_mean, dst_mean;

  // computation of mean
  src_mean = internal::MeanRow(src);
  dst_mean = internal::MeanRow(dst);

  // demeaning of src and dst points
  cv::Mat src_demean = internal::DemeanRow(src, src_mean);
  cv::Mat dst_demean = internal::DemeanRow(dst, dst_mean);

  // Eq. (36)-(37)
  double src_var = src_demean.dot(src_demean) / n;

  // Eq. (38)
  cv::Mat sigma = dst_demean * src_demean.t() / n;
  cv::SVD svd(sigma, cv::SVD::FULL_UV);

  // initialized the resulting transformation with an identity matrix...
  cv::Mat rt = cv::Mat::eye(m + 1, m + 1, CV_32FC1);

  // Eq. (39)
  cv::Mat s = cv::Mat::ones(m, 1, CV_32FC1);
  if (cv::determinant(svd.u) * cv::determinant(svd.vt) < 0) {
    s.at<float>(m - 1, 0) = -1;
  }

  // Eq. (40) and (43)
  rt.rowRange(0, m).colRange(0, m) = svd.u * cv::Mat::diag(s) * svd.vt;

  double scale = 1.0f;
  if (with_scale) {
    // Eq. (42)
    scale = scale / src_var * svd.w.dot(s);
  }
  // Eq. (41)
  cv::Mat top_left_mXm = rt.rowRange(0, m).colRange(0, m);
  cv::Mat col = dst_mean - scale * top_left_mXm * src_mean;
  col.copyTo(rt.rowRange(0, m).colRange(m, m + 1));
  top_left_mXm *= scale;
  return rt;
}
namespace internal {
cv::Mat MeanRow(const cv::Mat& src) {
  assert(src.channels() == 1);
  cv::Mat mean = cv::Mat(src.rows, 1, CV_32FC1);
  for (int i = 0; i < src.rows; ++i) {
    cv::Mat row = src.rowRange(i, i + 1);
    cv::Scalar mean_row = cv::mean(row);
    mean.at<float>(0, i) = mean_row[0];
  }
  return mean;
}

cv::Mat DemeanRow(const cv::Mat& src, const cv::Mat& mean) {
  assert(src.channels() == 1 && src.rows == mean.rows);
  cv::Mat demean = src.clone();
  for (int i = 0; i < demean.rows; ++i) {
    cv::Mat row = demean.rowRange(i, i + 1);
    cv::subtract(row, mean.at<float>(0, i), row, cv::noArray(), CV_32FC1);
  }
  return demean;
}
}  // namespace internal

}  // namespace utils
