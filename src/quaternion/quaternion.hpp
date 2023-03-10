#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_

#include <vector>
#include <exception>
#include <cmath>
#include "linalg/linalg.hpp"

namespace quaternion
{
void q_to_euler(linalg::Vector& q, linalg::Vector& euler);
linalg::Vector q_to_euler(linalg::Vector& q);
void q_to_mat4(linalg::Vector& q, linalg::Matrix& mat);
linalg::Matrix q_to_mat4(linalg::Vector& q);
}; // namespace quaternion

#endif