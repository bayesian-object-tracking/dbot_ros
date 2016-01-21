/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file object_tracker_publisher.cpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbot/tracker/rbc_particle_filter_object_tracker.hpp>
#include <dbot_ros/object_tracker_publisher.h>
#include <dbot_ros/object_tracker_publisher.hpp>

namespace dbot
{
template class ObjectTrackerPublisher<RbcParticleFilterObjectTracker>;
}

