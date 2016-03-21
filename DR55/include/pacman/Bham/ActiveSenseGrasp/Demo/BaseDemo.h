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

#include <pacman/Bham/Demo/Demo.h>

/** Pacman name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Grasp demo. */
class BaseDemo : public grasp::DemoDR55 {
public:

	/** Data */
	//class Data : public grasp::DemoDR55::Data {
	//public:

	//	/** Data bundle description */
	//	class Desc : public grasp::DemoDR55::Data::Desc {
	//	public:
	//		/** Creates the object from the description. */
	//		virtual grasp::data::Data::Ptr create(golem::Context &context) const;
	//	};

	//	/** Manager */
	//	virtual void setOwner(grasp::Manager* owner);

	//	/** Creates render buffer of the bundle without items */
	//	virtual void createRender();

	//protected:
	//	/** BaseDemo */
	//	BaseDemo* owner;

	//	/** Load from xml context */
	//	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap);
	//	/** Save to xml context */
	//	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	//	/** Creates data bundle */
	//	void create(const Desc& desc);
	//	/** Creates data bundle */
	//	Data(golem::Context &context);
	//};

	//friend class Data;
	//
	/** Grasp demo description */
	class Desc : public grasp::DemoDR55::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;


		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::DemoDR55::Desc::setToDefault();

		
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::DemoDR55::Desc::assertValid(ac);

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
