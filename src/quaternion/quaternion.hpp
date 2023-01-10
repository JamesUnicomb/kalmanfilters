#ifndef _QUATERNION_HPP_
#define _QUATERNION_HPP_

#include <vector>
#include <exception>
#include <cmath>

namespace quaternion
{
void q_to_euler(std::vector<double>& q, std::vector<double>& euler);
std::vector<double> q_to_euler(std::vector<double> q);
void q_to_mat4(std::vector<double>& q, std::vector<std::vector<double>>& mat);
std::vector<std::vector<double>> q_to_mat4(std::vector<double> q);
}; // namespace quaternion

#endif