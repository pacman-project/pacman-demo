/** @file ControlImpl.h
 * 
 * Bham Control interface implementation
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_CONTROL_CONTROL_IMPL_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_CONTROL_CONTROL_IMPL_H_

#include <Golem/Ctrl/Controller.h>
#include <pacman/Bham/Control/Control.h>

/** PaCMan name space */
namespace pacman {
	/** Birmingham control interface implementation */
	class BhamControlImpl : public BhamControl {
	public:
		/** Current time counted from an arbitrary moment
		*/
		virtual float_t time() const;

		/** Controller cycle duration
		*/
		virtual float_t cycleDuration() const;

		/**	State of a robot at time t
		*/
		virtual void lookupState(float_t t, void* state) const;

		/**	Command sent to a robot at time t
		*/
		virtual void lookupCommand(float_t t, void* command) const;

		/**	Robot control
		*/
		virtual void send(const void* command, std::uintptr_t size = 1);

		/**	Wait for begin of the robot control cycle
		*/
		virtual bool waitForCycleBegin(double timewait = std::numeric_limits<float_t>::max());

		/**	Wait for end of the robot trajectory execution
		*/
		virtual bool waitForTrajectoryEnd(double timewait = std::numeric_limits<float_t>::max());


		/** Get command */
		const void* getCommand(const void* ptr, std::uintptr_t n) const;

		/** State conversion */
		void convertState(const ::golem::Controller::State& src, void* dst) const;
		/** Command conversion */
		void convertCommand(const ::golem::Controller::State& src, void* dst) const;
		/** Command conversion */
		void convertCommand(const void* src, ::golem::Controller::State& dst) const;


		/** State conversion */
		void convert(const ::golem::Controller::State& src, RobotUIBK::State& dst) const;
		/** Command conversion */
		void convert(const ::golem::Controller::State& src, RobotUIBK::Command& dst) const;
		/** Command conversion */
		void convert(const RobotUIBK::Command& src, ::golem::Controller::State& dst) const;

		/** Assert robot configuration compatibility */
		template <typename _Config, golem::U32 JOINTS> static void assertConfig(const ::golem::Controller::State::Info& info) {
			if (info.getChains().size() != _Config::CHAINS)
				throw ::golem::Message(::golem::Message::LEVEL_ERROR, "BhamControlImpl::assertConfig(): invalid number of chains = %u", info.getChains().size());
			if (info.getJoints().size() != JOINTS)
				throw ::golem::Message(::golem::Message::LEVEL_ERROR, "BhamControlImpl::assertConfig(): invalid number of joints = %u", info.getJoints().size());
		}
		/** Assert robot compatibility */
		static void assertRobotUIBK(const ::golem::Controller& controller);

		/** Interface creation */
		BhamControlImpl(RobotType type, ::golem::Controller& controller);

	protected:
		const RobotType type;
		::golem::Context& context;
		::golem::Controller& controller;
		const ::golem::Controller::State::Info info;
	};
};

#endif // _PACMAN_BHAM_CONTROL_CONTROL_IMPL_H_