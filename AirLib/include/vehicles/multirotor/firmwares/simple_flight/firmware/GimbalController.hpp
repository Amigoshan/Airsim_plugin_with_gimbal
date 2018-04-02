#pragma once

#include <string>
#include <exception>
#include "interfaces/IController.hpp"
#include "interfaces/IStateEstimator.hpp"
#include "interfaces/ICommLink.hpp"
#include "interfaces/IGoal.hpp"
#include "PidController.hpp"
#include "Params.hpp"
#include "common/common_utils/Utils.hpp"

#define OUTPUT(x) if(GEngine){GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Yellow, TEXT(x));}

namespace simple_flight {

	class GimbalController : public IController {
	public:
		GimbalController(const Params* params, const IBoardClock* clock, ICommLink* comm_link)
			: params_(params), clock_(clock), comm_link_(comm_link)
		{
		}

		virtual void initialize(const IGoal* goal, const IStateEstimator* state_estimator) override
		{
			goal_ = goal;
			state_estimator_ = state_estimator;

			for (unsigned int k = 0; k < axisnum; k++)
			{
				axis_controllers_[k].reset(new PidController<float>(clock_,
					PidConfig<float>(params_->angle_rate_pid.p[0], 0, 0)));
			}
		}

		virtual void reset() override
		{
			IController::reset();

			//last_goal_mode_ = GoalMode::getUnknown();
			output_ = Axis4r();

			for (unsigned int axis = 0; axis < axisnum; ++axis) {
				if (axis_controllers_[axis] != nullptr)
					axis_controllers_[axis]->reset();
			}

		}

		virtual void update() override
		{
			IController::update();

			auto& goal_val = goal_->getCamGoalValue();

			for (unsigned int axis = 0; axis < axisnum; ++axis) {

				axis_controllers_[axis]->setGoal(goal_val[axis]);
				axis_controllers_[axis]->setMeasured(state_estimator_->getCameraVelocity()[axis]);
				//update axis controller
				if (axis_controllers_[axis] != nullptr) {
					axis_controllers_[axis]->update();
					output_[axis] = axis_controllers_[axis]->getOutput();
				}
				else
					comm_link_->log(std::string("Axis controller type is not set for axis ").append(std::to_string(axis)), ICommLink::kLogLevelInfo);
			}
		}

		virtual const Axis4r& getOutput() override
		{
			return output_;
		}


	private:
		const Params* params_;
		const IBoardClock* clock_;

		const IGoal* goal_;
		const IStateEstimator* state_estimator_;
		ICommLink* comm_link_;

		Axis4r output_;

		unsigned int axisnum = 3; // roll, pitch and yaw
		std::unique_ptr<PidController<float>> axis_controllers_[3]; // roll, pitch and yaw
	};

}