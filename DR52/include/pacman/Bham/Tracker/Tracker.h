/** @file Tracker.h
*
* Bham object tracking library
*
*/

#pragma once
#ifndef _PACMAN_BHAM_TRACKER_TRACKER_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_TRACKER_TRACKER_H_

#include <Grasp/Core/Defs.h>
#include <Golem/Tools/XMLData.h>

/** PaCMan name space */
namespace pacman {

class Tracker
{
public:
	typedef golem::shared_ptr<Tracker> Ptr;

	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		size_t nstep; // number of steps to rotate 360deg about z-axis

		Desc() { setToDefault(); }

		virtual ~Desc() {} // for subclassing

		void setToDefault() {
			nstep = 100;
		}

		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(nstep > 0, ac, "nstep: < 1");
		}

		//virtual Tracker::Ptr create(golem::Context &context) const;
		GRASP_CREATE_FROM_OBJECT_DESC1(Tracker, Tracker::Ptr, golem::Context &)

		virtual void load(golem::Context &context, const golem::XMLContext* xmlcontext);
	};

	virtual ~Tracker() {}

protected:
	golem::Context& context;

	size_t nstep; // number of steps to rotate 360deg about z-axis

	void create(const Desc& desc);
	Tracker(golem::Context &_context) : context(_context) {}
};

} // namespace pacman

#endif // _PACMAN_BHAM_TRACKER_TRACKER_H_