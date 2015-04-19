/** @file Demo.h
*
* PaCMan DR 5.2. demo
*
*/

#pragma once
#ifndef _PACMAN_BHAM_DEMO_DEMO_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_DEMO_DEMO_H_

#include <Grasp/App/Player/Player.h>
#include <Grasp/Grasp/Model.h>

/** PaCMan name space */
namespace pacman {

//------------------------------------------------------------------------------

/** Demo. */
class Demo : public grasp::Player {
public:
	/** Data */
	class Data : public grasp::Player::Data {
	public:
		/** Data bundle default name */
		std::string dataName;

		/** Drainer pose */
		golem::Mat34 drainerPose;
		/** Drainer model triangles */
		grasp::Vec3Seq drainerVertices;
		/** Drainer model triangles */
		grasp::TriangleSeq drainerTriangles;
		/** Drainer model triangles */
		grasp::Contact3D::Triangle::Seq drainerModelTriangles;

		/** Data bundle description */
		class Desc : public grasp::Player::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual grasp::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Manager */
		virtual void setOwner(grasp::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** Demo */
		Demo* owner;

		/** Load from xml context */
		virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap);
		/** Save to xml context */
		virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	friend class Data;

	/** Demo description */
	class Desc : public grasp::Player::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Data bundle default name */
		std::string dataName;

		/** Drainer pose estimation camera */
		std::string drainerCamera;
		/** Drainer data handler */
		std::string drainerHandler;
		/** Drainer data item */
		std::string drainerItem;
		/** Drainer scan pose */
		grasp::ConfigMat34 drainerScanPose;
		/** Drainer colour solid */
		golem::RGBA drainerColourSolid;
		/** Drainer colour wireframe */
		golem::RGBA drainerColourWire;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Desc::setToDefault();

			dataDesc.reset(new Data::Desc);

			dataName.clear();

			drainerCamera.clear();
			drainerHandler.clear();
			drainerItem.clear();
			drainerScanPose.setToDefault();
			drainerColourSolid.set(golem::RGBA::GREEN._U8[0], golem::RGBA::GREEN._U8[1], golem::RGBA::GREEN._U8[2], golem::numeric_const<golem::U8>::MAX / 8);
			drainerColourWire.set(golem::RGBA::GREEN);
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Player::Desc::assertValid(ac);

			grasp::Assert::valid(dataDesc != nullptr && grasp::is<Data::Desc>(dataDesc.get()), ac, "dataDesc: unknown type");

			grasp::Assert::valid(dataName.length() > 0, ac, "dataName: invalid");

			grasp::Assert::valid(drainerCamera.length() > 0, ac, "drainerCamera: invalid");
			grasp::Assert::valid(drainerHandler.length() > 0, ac, "drainerHandler: invalid");
			grasp::Assert::valid(drainerItem.length() > 0, ac, "drainerItem: invalid");
			drainerScanPose.assertValid(grasp::Assert::Context(ac, "drainerScanPose."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(Demo, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Data bundle default name */
	std::string dataName;

	/** Drainer pose estimation camera */
	grasp::Camera* drainerCamera;
	/** Drainer data handler */
	grasp::data::Handler* drainerHandler;
	/** Drainer data item */
	std::string drainerItem;
	/** Drainer scan pose */
	grasp::ConfigMat34 drainerScanPose;

	/** Drainer colour solid */
	golem::RGBA drainerColourSolid;
	/** Drainer colour wireframe */
	golem::RGBA drainerColourWire;
	/** Drainer renderer */
	golem::DebugRenderer drainerRenderer;

	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	Demo(golem::Scene &scene);
	~Demo();
};

//------------------------------------------------------------------------------

};

#endif // _PACMAN_BHAM_DEMO_DEMO_H_