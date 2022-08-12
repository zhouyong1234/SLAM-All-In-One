/*!
*	\file         sbg_vector3.h
*	\author       SBG Systems
*	\date         13/03/2020
*	
*	\brief        Handle a three components vector.
*
*   This custom vector3 class is compatible for operations with SbgMatrix3 in
*   order to avoid a dependancy on Eigen.
*   This class also defines SbgVector3f and SbgVector3d for floats and doubles.
*
*	\section CodeCopyright Copyright Notice
*	MIT License
*	
*	Copyright (c) 2020 SBG Systems
*	
*	Permission is hereby granted, free of charge, to any person obtaining a copy
*	of this software and associated documentation files (the "Software"), to deal
*	in the Software without restriction, including without limitation the rights
*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*	copies of the Software, and to permit persons to whom the Software is
*	furnished to do so, subject to the following conditions:
*	
*	The above copyright notice and this permission notice shall be included in all
*	copies or substantial portions of the Software.
*	
*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*	SOFTWARE.
*/

#ifndef SBG_VECTOR_3_H
#define SBG_VECTOR_3_H

// ROS headers
#include <geometry_msgs/Vector3.h>

// Sbg headers
#include <sbgDefines.h>

namespace sbg
{
/*!
 * Check if the two input numbers are equals, taking into account the machine precision (float/double).
 *
 * \template  T                           Numeric template type.
 * \param[in] firstValue                  First numeric value.
 * \param[in] secondValue                 Second numeric value.
 * \return                                True if the numbers are considered as equals.
 */
template <typename T>
bool areEquals(T firstValue, T secondValue)
{
  //
  // The machine epsilon has to be scaled to the magnitude of the values used.
  //
  return std::fabs(firstValue - secondValue) <= (std::numeric_limits<T>::epsilon() * std::fabs(firstValue + secondValue));
}

/*!
 * Class to define a Vector3.
 */
template <class T>
class SbgVector3
{
private:

  //---------------------------------------------------------------------//
  //- Private variables                                                 -//
  //---------------------------------------------------------------------//

  std::array<T, 3> m_data;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Empty constructor.
   */
  SbgVector3(void)
  {
    m_data[0] = static_cast<T>(0.0);
    m_data[1] = static_cast<T>(0.0);
    m_data[2] = static_cast<T>(0.0);
  }

  /*!
   * Constructor.
   *
   * \param[in] x_value                     Vector X value.
   * \param[in] y_value                     Vector Y value.
   * \param[in] z_value                     Vector Z value.
   */
  SbgVector3(T x_value, T y_value, T z_value)
  {
    m_data[0] = x_value;
    m_data[1] = y_value;
    m_data[2] = z_value;
  };

  /*!
   * Constructor.
   *
   * \param[in] p_raw_data                  Pointer to data array.
   * \param[in] array_size                  Array size (Should be defined as 3).
   */
  SbgVector3(const T* p_raw_data, size_t array_size)
  {
    assert(array_size == 3);

    m_data[0] = p_raw_data[0];
    m_data[1] = p_raw_data[1];
    m_data[2] = p_raw_data[2];
  };

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Comparison equal operator.
   *
   * \param[in] ref_vector                  Vector to compare.
   * \return                                True if the vector are equals.
   */
  bool operator==(const SbgVector3<T> &ref_vector)
  {
    return ((areEquals(m_data[0], ref_vector.m_data[0]))
          && (areEquals(m_data[1], ref_vector.m_data[1]))
          && (areEquals(m_data[2], ref_vector.m_data[2])));
  };

  /*!
   * Comparison not equal operator.
   *
   * \param[in] ref_vector                  Vector to compare.
   * \return                                True if the vector are equals.
   */
  bool operator!=(const SbgVector3<T> &ref_vector)
  {
    return !(*this == ref_vector);
  };

  /*!
   * Getter parenthesis operator
   * \param[in] index					Index of value to retrieve.
   * \return 							Value at index.
   */
  const T operator()(size_t index) const
  {
      assert(index < 3);

      return m_data[index];
  }

  /*!
   * Get the raw data of the sbgVector.
   *
   * \return                                Raw vector data.
   */
  const T *data(void) const
  {
    return static_cast<const T*>(m_data.data());
  };
};

typedef SbgVector3<float>  SbgVector3f;
typedef SbgVector3<double> SbgVector3d;

}

#endif // SBG_VECTOR_3_H
