#include <pacman/RobotKuka/RobotKuka.h>
#include <memory>
#include <stdexcept>

using namespace pacman;

/** Robot implementation can be private, but also public if required */
class RobotKuka : public Robot {
public:
	/** Pointer */
	typedef std::auto_ptr<RobotKuka> Ptr;

	/** DOF */
	static const int DOF = 7;

	/** Construction */
	RobotKuka(const std::string& cfgfile);

	/** Name of the robot */
	virtual const std::string& getName() const;

	/** Returns the current robot configuration */
	virtual void getConfiguration(Configuration& configuration) const;

	/** Moves robot to the specified configuration */
	virtual void moveConfiguration(const Configuration& configuration);

protected:
	/** Name */
	const std::string mName;
	/** Current configuration */
	Configuration mConfiguration;
};

/** A single instance of Kuka robot */
RobotKuka::Ptr robot;

RobotKuka::RobotKuka(const std::string& cfgfile) : mName("Kuka Robot") {
	// load config file

	// initialise robot configuration
	mConfiguration.assign(DOF, 0.);
}

const std::string& RobotKuka::getName() const {
	return mName;
}

void RobotKuka::getConfiguration(Configuration& configuration) const {
	configuration = mConfiguration;
}

void RobotKuka::moveConfiguration(const Configuration& configuration) {
	if (configuration.size() != DOF)
		throw std::runtime_error("RobotKuka::moveConfiguration: Invalid DOF");
	mConfiguration = configuration;
}

pacman::Robot* pacman::createRobotKuka(const std::string& cfgfile) {
	robot.reset(new RobotKuka(cfgfile));
	return robot.get();
}
