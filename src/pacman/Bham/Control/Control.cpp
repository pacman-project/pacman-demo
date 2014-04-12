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

void BhamControlImpl::lookupState(pacman::float_t t, Robot::State& state) const {
	Controller::State inp = controller.createState();
	controller.lookupState((SecTmReal)t, inp);
	convert(inp, state);
}

void BhamControlImpl::lookupCommand(pacman::float_t t, Robot::Command& command) const {
	Controller::State inp = controller.createState();
	controller.lookupCommand((SecTmReal)t, inp);
	convert(inp, command);
}

void BhamControlImpl::send(const Robot::Command* command, std::uintptr_t size) {
	if (size <= 0)
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::send(): invalid number of commands %u", size);
	
	Controller::State::Seq seq(size, controller.createState());
	for (Controller::State::Seq::iterator i = seq.begin(); i != seq.end(); ++i) {
		controller.setToDefault(*i);
		convert((const Robot::Command&)(*command)(i - seq.begin()), *i);
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
	assertConfig<RobotUIBK::Config::CHAINS, 7 + 3 + 3 + 2>(controller.getStateInfo());
}

void BhamControlImpl::assertRobotEddie() const {
	assertConfig<RobotEddie::Config::CHAINS, 2*(7 + 3 + 3 + 2) + 7>(controller.getStateInfo());
}

//-----------------------------------------------------------------------------

void BhamControlImpl::convert(const ::golem::Controller::State& src, Robot::State& dst) const {
	switch (dst.getType()) {
	case Robot::Type::ROBOT_UIBK:
		BhamControlImpl::assertRobotUIBK(); // throws
		convert(src, (RobotUIBK::State&)dst);
		break;
	case Robot::Type::ROBOT_EDDIE:
		BhamControlImpl::assertRobotEddie(); // throws
		convert(src, (RobotEddie::State&)dst);
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::convertState(): unknown robot type %u", dst.getType());
	}
}

void BhamControlImpl::convert(const ::golem::Controller::State& src, Robot::Command& dst) const {
	switch (dst.getType()) {
	case Robot::Type::ROBOT_UIBK:
		BhamControlImpl::assertRobotUIBK(); // throws
		convert(src, (RobotUIBK::Command&)dst);
		break;
	case Robot::Type::ROBOT_EDDIE:
		BhamControlImpl::assertRobotEddie(); // throws
		convert(src, (RobotEddie::Command&)dst);
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::convertCommand(): unknown robot type %u", dst.getType());
	}
}

void BhamControlImpl::convert(const Robot::Command& src, ::golem::Controller::State& dst) const {
	switch (src.getType()) {
	case Robot::Type::ROBOT_UIBK:
		BhamControlImpl::assertRobotUIBK(); // throws
		convert((const RobotUIBK::Command&)src, dst);
		break;
	case Robot::Type::ROBOT_EDDIE:
		BhamControlImpl::assertRobotEddie(); // throws
		convert((const RobotEddie::Command&)src, dst);
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::convertCommand(): unknown robot type %u", src.getType());
	}
}

//-----------------------------------------------------------------------------

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotUIBK::State& dst) const {
	// time
	dst.t = (pacman::float_t)src.t;
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 0).begin()], dst.arm.pos);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 1).begin()], dst.hand.pos);
}

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotUIBK::Command& dst) const {
	// time
	dst.t = (pacman::float_t)src.t;
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 0).begin()], dst.arm.pos);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 0).begin()], dst.arm.vel);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 0).begin()], dst.arm.acc);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 1).begin()], dst.hand.pos);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 1).begin()], dst.hand.vel);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 1).begin()], dst.hand.acc);
}

void BhamControlImpl::convert(const RobotUIBK::Command& src, ::golem::Controller::State& dst) const {
	// time
	dst.t = (SecTmReal)src.t;
	// arm
	configToGolem(src.arm.pos, &dst.cpos[info.getJoints(info.getChains().begin() + 0).begin()]);
	configToGolem(src.arm.vel, &dst.cvel[info.getJoints(info.getChains().begin() + 0).begin()]);
	configToGolem(src.arm.acc, &dst.cacc[info.getJoints(info.getChains().begin() + 0).begin()]);
	// hand
	configToGolem(src.hand.pos, &dst.cpos[info.getJoints(info.getChains().begin() + 1).begin()]);
	configToGolem(src.hand.vel, &dst.cvel[info.getJoints(info.getChains().begin() + 1).begin()]);
	configToGolem(src.hand.acc, &dst.cacc[info.getJoints(info.getChains().begin() + 1).begin()]);
}

//-----------------------------------------------------------------------------

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotEddie::State& dst) const {
	// time
	dst.t = (pacman::float_t)src.t;
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 0).begin()], dst.armLeft.pos);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 1).begin()], dst.handLeft.pos);
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 4).begin()], dst.armRight.pos);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 5).begin()], dst.handRight.pos);
	// head
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 8).begin()], dst.head.pos);
}

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotEddie::Command& dst) const {
	// time
	dst.t = (pacman::float_t)src.t;
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 0).begin()], dst.armLeft.pos);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 0).begin()], dst.armLeft.vel);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 0).begin()], dst.armLeft.acc);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 1).begin()], dst.handLeft.pos);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 1).begin()], dst.handLeft.vel);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 1).begin()], dst.handLeft.acc);
	// arm
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 4).begin()], dst.armRight.pos);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 4).begin()], dst.armRight.vel);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 4).begin()], dst.armRight.acc);
	// hand
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 5).begin()], dst.handRight.pos);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 5).begin()], dst.handRight.vel);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 5).begin()], dst.handRight.acc);
	// head
	configToPacman(&src.cpos[info.getJoints(info.getChains().begin() + 8).begin()], dst.head.pos);
	configToPacman(&src.cvel[info.getJoints(info.getChains().begin() + 8).begin()], dst.head.vel);
	configToPacman(&src.cacc[info.getJoints(info.getChains().begin() + 8).begin()], dst.head.acc);
}

void BhamControlImpl::convert(const RobotEddie::Command& src, ::golem::Controller::State& dst) const {
	// time
	dst.t = (SecTmReal)src.t;
	// arm
	configToGolem(src.armLeft.pos, &dst.cpos[info.getJoints(info.getChains().begin() + 0).begin()]);
	configToGolem(src.armLeft.vel, &dst.cvel[info.getJoints(info.getChains().begin() + 0).begin()]);
	configToGolem(src.armLeft.acc, &dst.cacc[info.getJoints(info.getChains().begin() + 0).begin()]);
	// hand
	configToGolem(src.handLeft.pos, &dst.cpos[info.getJoints(info.getChains().begin() + 1).begin()]);
	configToGolem(src.handLeft.vel, &dst.cvel[info.getJoints(info.getChains().begin() + 1).begin()]);
	configToGolem(src.handLeft.acc, &dst.cacc[info.getJoints(info.getChains().begin() + 1).begin()]);
	// arm
	configToGolem(src.armRight.pos, &dst.cpos[info.getJoints(info.getChains().begin() + 4).begin()]);
	configToGolem(src.armRight.vel, &dst.cvel[info.getJoints(info.getChains().begin() + 4).begin()]);
	configToGolem(src.armRight.acc, &dst.cacc[info.getJoints(info.getChains().begin() + 4).begin()]);
	// hand
	configToGolem(src.handRight.pos, &dst.cpos[info.getJoints(info.getChains().begin() + 5).begin()]);
	configToGolem(src.handRight.vel, &dst.cvel[info.getJoints(info.getChains().begin() + 5).begin()]);
	configToGolem(src.handRight.acc, &dst.cacc[info.getJoints(info.getChains().begin() + 5).begin()]);
	// head
	configToGolem(src.head.pos, &dst.cpos[info.getJoints(info.getChains().begin() + 8).begin()]);
	configToGolem(src.head.vel, &dst.cvel[info.getJoints(info.getChains().begin() + 8).begin()]);
	configToGolem(src.head.acc, &dst.cacc[info.getJoints(info.getChains().begin() + 8).begin()]);
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
