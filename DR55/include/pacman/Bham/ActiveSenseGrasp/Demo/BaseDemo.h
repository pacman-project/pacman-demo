/** @file Grasp.h
 *
 * Adapated from Demo Grasp by Ermano Arruda
 *
 * @author	Marek Kopicki
 * @author Ermano Arruda
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GRASP_DEMO_GRASP_H_ // if #pragma once is not supported
#define _GRASP_DEMO_GRASP_H_

#include <pacman/Bham/Demo/BaseDemo.h>

/** Pacman name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Grasp demo. */
class BaseDemo : public pacman::BaseDemoDR55 {
public:

	//friend class Data;
	//
	/** Grasp demo description */
	class Desc : public pacman::BaseDemoDR55::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;


		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			pacman::BaseDemoDR55::Desc::setToDefault();

		
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			pacman::BaseDemoDR55::Desc::assertValid(ac);

		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(BaseDemo, golem::Object::Ptr, golem::Scene&)
	};

protected:
	

	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	BaseDemo(golem::Scene &scene);
	virtual ~BaseDemo();
};


//------------------------------------------------------------------------------

};

#endif // _GRASP_DEMO_GRASP_H_
