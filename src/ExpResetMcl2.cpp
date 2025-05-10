// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/ExpResetMcl2.h"

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>

#include <cmath>
#include <iostream>

namespace emcl2
{
ExpResetMcl2::ExpResetMcl2(
  const Pose & p, int num, const Scan & scan1, const Scan & scan2, const std::shared_ptr<OdomModel> & odom_model,
  const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
  double expansion_radius_position, double expansion_radius_orientation, double extraction_rate,
  double range_threshold, bool sensor_reset)
: Mcl::Mcl(p, num, scan1, scan2, odom_model, map),
  alpha_threshold_(alpha_th),
  expansion_radius_position_(expansion_radius_position),
  expansion_radius_orientation_(expansion_radius_orientation),
  extraction_rate_(extraction_rate),
  range_threshold_(range_threshold),
  sensor_reset_(sensor_reset)
{
}

ExpResetMcl2::~ExpResetMcl2() {}

void ExpResetMcl2::sensorUpdate(pcl::PointCloud<pcl::PointXYZ>::Ptr& combined_cloud)
{

	// 각 파티클 weight 계산.
	for (auto & p : particles_) {
		p.w_ *= p.likelihood(map_.get(), combined_cloud);
	}

	if (normalizeBelief() > 0.000001) {
		resampling();
	} else {
		resetWeight();
	}



}

// 즉, 현재 파티클들 중 얼마나 많은 파티클이 장애물과 겹치지 않고 적절한 위치에 있는지를 나타내는 지표.
double ExpResetMcl2::nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for (size_t i = shift % skip; i < particles_.size(); i += skip) {
		counter++;
		if (particles_[i].wallConflict(map, scan, range_threshold_, sensor_reset_)) {
			penetrating++;
		}
	}
	shift++;

	std::cout << penetrating << " " << counter << std::endl;
	return static_cast<double>((counter - penetrating)) / counter;
}

// 그냥 무작위로 확 흩어버리는 게 아니라
// 현재 위치 주변으로 확산(Expansion) 시키는 방식이야.
// 이걸 expansion reset이라 불러.
void ExpResetMcl2::expansionReset(void)
{
	for (auto & p : particles_) {
		double length =
		  2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * expansion_radius_position_;
		double direction = 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * M_PI;

		p.p_.x_ += length * cos(direction);
		p.p_.y_ += length * sin(direction);
		p.p_.t_ += 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) *
			   expansion_radius_orientation_;
		p.w_ = 1.0 / particles_.size();
	}
}

}  // namespace emcl2
