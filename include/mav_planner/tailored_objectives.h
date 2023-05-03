/*
Motivation:
Previously, we considered optimal planning in terms of minimizing the length of the path found.
However, observation shows this path tends to steer very close to obstacles, which can sometimes be unsafe for quads. 
*/

#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>


namespace ob = ompl::base; 
namespace og = ompl::geometric;


/**
 * @brief This class inherits as subclass from  subclass of ompl::base::StateCostIntegralObjective. 
 * This is because ompl::base::StateCostIntegralObjective represents objectives as summations of state costs
 * 
 */
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:

    /**
     * @brief Construct a new Clearance Objective object that inhertis characteristics from cost integral
     * @note beside passing the state, the flag here changes the default behaviour of the objective to use motion cost interpolation 
     * when summing up state costs along the path. Consquently, calls to motionCost() will divide the motion segment into smaller parts 
     * (the number of parts being defined by StateSpace::validSegmentCount()) for more accurate cost integral computation+
     * (but this takes more computation time).revise -> https://ompl.kavrakilab.org/classompl_1_1base_1_1StateCostIntegralObjective.html
     * The reasonong behind the flag is that it's proven by default (false), ompl::base::StateCostIntegralObjective simply takes the individual states 
     * that make up a given path, and sums up those costs. However, this approach can result in an inaccurate estimation of the path cost if successive 
     * states on the path are far apart. Yet, if motion cost interpolation is enabled, the path cost computation will interpolate between distant states 
     * in order to get a more accurate approximation of the true path cost. This interpolation of states along a path is the same as the one used in ompl::base::DiscreteMotionValidator. 
     * 
     * @param state information 
     */
	ClearanceObjective(const ob::SpaceInformationPtr& si) :
		ob::StateCostIntegralObjective(si, true)
	{
	
	}
	
    /**
     * @brief The objective here to steer the robot away from obstacles.
     * @note By default, optimization objectives seek to minimize path cost. For our path clearance objective, we want paths to maximize path clearance. 
     * Therefore, state cost function is objectified to return smaller costs when states have greater clearance from obstacles
     * @param state space to be examined 
     * @return ob::Cost as integral cost (summations of state costs)
     */
	ob::Cost stateCost(const ob::State* s) const
	{
		return ob::Cost( (1 / si_->getStateValidityChecker()->clearance(s)));  // The integral cost is inverse propotioned to the clearance.
	}
};


class MaximizeMinClearance : public ob::OptimizationObjective
{
public:
    MaximizeMinClearance(const ob::SpaceInformationPtr &si) :
        ob::OptimizationObjective(si) {}
 
    virtual ob::Cost stateCost(const ob::State* s) const;
    virtual bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const;
    virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
    virtual ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const;
    virtual ob::Cost identityCost() const;
    virtual ob::Cost infiniteCost() const;
};