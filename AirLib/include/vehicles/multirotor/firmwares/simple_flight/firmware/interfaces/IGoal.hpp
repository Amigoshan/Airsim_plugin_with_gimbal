#pragma once

#include "IUpdatable.hpp"
#include "CommonStructs.hpp"

namespace simple_flight {

class IGoal {
public:
    virtual const Axis4r& getGoalValue() const = 0;
    virtual const GoalMode& getGoalMode() const = 0;
	virtual const Axis4r getCamGoalValue() const // for gimbal control, single camera supported -- amigo
	{
		return Axis4r(); // this function should not be called, call the derived ones. 
	} 
};

} //namespace