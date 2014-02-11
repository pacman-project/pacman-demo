#include <pacman/Bham/Control/ControlImpl.h>
#include <Golem/Tools/Data.h>
#include <Golem/Tools/XMLData.h>

using namespace pacman;
using namespace golem;

//-----------------------------------------------------------------------------

BhamControlImpl::BhamControlImpl(golem::Controller& controller) : controller(controller), context(controller.getContext()), info(controller.getStateInfo()) {}

pacman::float_t BhamControlImpl::time() const {
	return context.getTimer().elapsed();
}

pacman::float_t BhamControlImpl::cycleDuration() const {
	return controller.getCycleDuration();
}

void BhamControlImpl::lookupState(pacman::float_t t, RobotState& state) const {
	Controller::State inp = controller.createState();
	controller.lookupState((SecTmReal)t, inp);
	convert(inp, state);
}

void BhamControlImpl::lookupCommand(pacman::float_t t, RobotCommand& command) const {
	Controller::State inp = controller.createState();
	controller.lookupCommand((SecTmReal)t, inp);
	convert(inp, command);
}

void BhamControlImpl::send(const RobotCommand* command, std::uintptr_t size) {
	if (size <= 0)
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::send(): invalid number of commands %u", size);
	
	Controller::State::Seq seq(size, controller.createState());
	for (Controller::State::Seq::iterator i = seq.begin(); i != seq.end(); ++i) {
		controller.setToDefault(*i);
		convert((const RobotCommand&)(*command)(i - seq.begin()), *i);
	}

	controller.send(seq.data(), seq.data() + size);
}

bool BhamControlImpl::waitForCycleBegin(double timewait) {
	return controller.waitForBegin(golem::SecToMSec(timewait));
}

bool BhamControlImpl::waitForTrajectoryEnd(double timewait) {
	return controller.waitForEnd(golem::SecToMSec(timewait));
}

//-----------------------------------------------------------------------------

void BhamControlImpl::assertRobotUIBK() const {
	assertConfig<RobotUIBK::Config, 7 + 3 + 3 + 2>(controller.getStateInfo());
}

//-----------------------------------------------------------------------------

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotState& dst) const {
	switch (dst.getRobotType()) {
	case RobotType::ROBOT_UIBK:
		BhamControlImpl::assertRobotUIBK(); // throws
		convert(src, (RobotUIBK::State&)dst);
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::convertState(): unknown robot type %u", dst.getRobotType());
	}
}

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotCommand& dst) const {
	switch (dst.getRobotType()) {
	case RobotType::ROBOT_UIBK:
		BhamControlImpl::assertRobotUIBK(); // throws
		convert(src, (RobotUIBK::Command&)dst);
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::convertCommand(): unknown robot type %u", dst.getRobotType());
	}
}

void BhamControlImpl::convert(const RobotCommand& src, ::golem::Controller::State& dst) const {
	switch (src.getRobotType()) {
	case RobotType::ROBOT_UIBK:
		BhamControlImpl::assertRobotUIBK(); // throws
		convert((const RobotUIBK::Command&)src, dst);
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::convertCommand(): unknown robot type %u", src.getRobotType());
	}
}

//-----------------------------------------------------------------------------

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotUIBK::State& dst) const {
	// time
	dst.t = (pacman::float_t)src.t;
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 0).begin()], dst.pos.arm);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 1).begin()], dst.pos.hand);
}

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotUIBK::Command& dst) const {
	// time
	dst.t = (pacman::float_t)src.t;
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 0).begin()], dst.pos.arm);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 0).begin()], dst.vel.arm);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 0).begin()], dst.acc.arm);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 1).begin()], dst.pos.hand);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 1).begin()], dst.vel.hand);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 1).begin()], dst.acc.hand);
}

void BhamControlImpl::convert(const RobotUIBK::Command& src, ::golem::Controller::State& dst) const {
	// time
	dst.t = (SecTmReal)src.t;
	// arm
	configToGolem(src.pos.arm, &dst.cpos[info.getJoints(info.getChains().begin() + 0).begin()]);
	configToGolem(src.vel.arm, &dst.cvel[info.getJoints(info.getChains().begin() + 0).begin()]);
	configToGolem(src.acc.arm, &dst.cacc[info.getJoints(info.getChains().begin() + 0).begin()]);
	// hand
	configToGolem(src.pos.hand, &dst.cpos[info.getJoints(info.getChains().begin() + 1).begin()]);
	configToGolem(src.vel.hand, &dst.cvel[info.getJoints(info.getChains().begin() + 1).begin()]);
	configToGolem(src.acc.hand, &dst.cacc[info.getJoints(info.getChains().begin() + 1).begin()]);
}

//-----------------------------------------------------------------------------

Context::Ptr context;
Controller::Desc::Ptr controllerDesc;
Controller::Ptr controller;
BhamControlImpl* pBhamControl = nullptr;

BhamControl::Ptr BhamControl::create(const std::string& path) {
	// Create XML parser and load configuration file
	XMLParser::Desc parserDesc;
	XMLParser::Ptr pParser = parserDesc.create();
	FileReadStream fs(path.c_str());
	pParser->load(fs);

	// Find program XML root context
	XMLContext* pXMLContext = pParser->getContextRoot()->getContextFirst("golem");

	// Create program context
	golem::Context::Desc contextDesc;
	XMLData(contextDesc, pXMLContext);
	context = contextDesc.create(); // throws

	// Load driver
	controllerDesc = Controller::Desc::load(context.get(), pXMLContext->getContextFirst("controller"));

	// Create controller
	context->info("Initialising controller...\n");
	controller = controllerDesc->create(*context);

	pBhamControl = new BhamControlImpl(*controller); //do not throw!

	return BhamControl::Ptr(pBhamControl, [&] (BhamControl*) {
		delete pBhamControl;
		controller.release();
		controllerDesc.release();
	});
}

//-----------------------------------------------------------------------------
