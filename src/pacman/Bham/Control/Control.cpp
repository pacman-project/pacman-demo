#include <pacman/Bham/Control/ControlImpl.h>
#include <Golem/Tools/Data.h>
#include <Golem/Tools/XMLData.h>

using namespace pacman;
using namespace golem;

//-----------------------------------------------------------------------------

BhamControlImpl::BhamControlImpl(RobotType type, golem::Controller& controller) : type(type), controller(controller), context(controller.getContext()), info(controller.getStateInfo()) {}

pacman::float_t BhamControlImpl::time() const {
	return context.getTimer().elapsed();
}

pacman::float_t BhamControlImpl::cycleDuration() const {
	return controller.getCycleDuration();
}

void BhamControlImpl::lookupState(pacman::float_t t, void* state) const {
	Controller::State inp = controller.createState();
	controller.lookupState((SecTmReal)t, inp);
	convertState(inp, state);
}

void BhamControlImpl::lookupCommand(pacman::float_t t, void* command) const {
	Controller::State inp = controller.createState();
	controller.lookupCommand((SecTmReal)t, inp);
	convertCommand(inp, command);
}

void BhamControlImpl::send(const void* command, size_t size) {
	if (size <= 0)
		throw Message(Message::LEVEL_ERROR, "BhamControlImpl::send(): invalid number of commands %u", size);
	
	Controller::State::Seq seq(size, controller.createState());
	for (Controller::State::Seq::iterator i = seq.begin(); i != seq.end(); ++i) {
		controller.setToDefault(*i);
		convertCommand(getCommand(command, i - seq.begin()), *i);
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

const void* BhamControlImpl::getCommand(const void* ptr, size_t n) const {
	switch (type) {
	case ROBOT_UIBK:
		return (const RobotUIBK::Command*)ptr + n;
		break;
	default:
		return ptr;
	}
}

void BhamControlImpl::convertState(const ::golem::Controller::State& src, void* dst) const {
	switch (type) {
	case ROBOT_UIBK:
		convert(src, *(RobotUIBK::State*)dst);
		break;
	}
}

void BhamControlImpl::convertCommand(const ::golem::Controller::State& src, void* dst) const {
	switch (type) {
	case ROBOT_UIBK:
		convert(src, *(RobotUIBK::Command*)dst);
		break;
	}
}

void BhamControlImpl::convertCommand(const void* src, ::golem::Controller::State& dst) const {
	switch (type) {
	case ROBOT_UIBK:
		convert(*(const RobotUIBK::Command*)src, dst);
		break;
	}
}

//-----------------------------------------------------------------------------

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotUIBK::State& dst) const {
	// arm

	// hand
}

void BhamControlImpl::convert(const ::golem::Controller::State& src, RobotUIBK::Command& dst) const {
}

void BhamControlImpl::convert(const RobotUIBK::Command& src, ::golem::Controller::State& dst) const {
}

//-----------------------------------------------------------------------------

void BhamControlImpl::assertRobotUIBK(const golem::Controller& controller) {
	assertConfig<RobotUIBK::Config, 7 + 3 + 3 + 2>(controller.getStateInfo());
}

//-----------------------------------------------------------------------------

Context::Ptr context;
Controller::Desc::Ptr pControllerDesc;
Controller::Ptr controller;
BhamControlImpl* pBhamControl = nullptr;

BhamControl::Ptr BhamControl::create(RobotType type, const std::string& path) {
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
	golem::Context::Ptr pContext = contextDesc.create(); // throws

	// Load driver
	Controller::Desc::Ptr pControllerDesc = Controller::Desc::load(pContext.get(), pXMLContext->getContextFirst("controller"));

	// Create controller
	pContext->info("Initialising controller...\n");
	Controller::Ptr pController = pControllerDesc->create(*pContext);

	// Assert correct robot type
	switch (type) {
	case ROBOT_UIBK:
		BhamControlImpl::assertRobotUIBK(*pController); // throws
		break;
	default:
		throw Message(Message::LEVEL_ERROR, "BhamControl::create(): unknown robot type %u", type);
	}

	pBhamControl = new BhamControlImpl(type, *pController); //do not throw!

	return BhamControl::Ptr(pBhamControl, [&] (BhamControl*) {
		delete pBhamControl;
		controller.release();
		pControllerDesc.release();
		context.release();
	});
}

//-----------------------------------------------------------------------------
