#include <pacman/Bham/ActiveSens/ActiveSens.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Phys/Data.h>

using namespace pacman;
using namespace golem;
using namespace grasp;


//------------------------------------------------------------------------------

void pacman::HypothesisSensor::Appearance::load(const golem::XMLContext* xmlcontext) {

	/*
	<appearance>
      <frame show="1" v1="0.05" v2="0.05" v3="0.1"/>
      <shape show="1" R="127" G="255" B="10" A="255">
        <bounds type="box" group="1">
          <dimensions v1="0.025" v2="0.025" v3="0.025"/>
          <pose v1="0.0" v2="0.0" v3="-0.025" roll="0.0" pitch="0.0" yaw="0.0"/>
        </bounds>
      </shape>
    </appearance>	
	*/
	if (!xmlcontext)
	{
		
		frameSize.set(0.05, 0.05, 0.1);
		frameShow = true;
		shapeColour.set(255, 0, 0, 255);
		shapeShow = true;
	}
	else
	{
		golem::XMLData(frameSize, xmlcontext->getContextFirst("frame"), false);
		golem::XMLData("show", frameShow, xmlcontext->getContextFirst("frame"), false);
		golem::XMLData(shapeColour, xmlcontext->getContextFirst("shape"), false);
		golem::XMLData("show", shapeShow, xmlcontext->getContextFirst("shape"), false);
	}
	
}

//------------------------------------------------------------------------------

void pacman::HypothesisSensor::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {

	if (!xmlcontext)
	{
		appearance.load(xmlcontext);

		golem::BoundingBox::Desc desc;
		desc.dimensions.set(0.025, 0.025, 0.025);
		desc.pose.setId();
		desc.pose.p.z -= 0.025;
		golem::BoundingBox::Desc::Ptr descBox(new golem::BoundingBox::Desc(desc));

		shapeDesc.push_back(descBox);
		this->configJoint = 0;
	}
	else
	{
		appearance.load(xmlcontext->getContextFirst("appearance"));
		try {
		golem::XMLData(shapeDesc, shapeDesc.max_size(), xmlcontext->getContextFirst("appearance shape"), "bounds", false);
		}
		catch (const MsgXMLParserNameNotFound&) {
		}

		golem::XMLData("config_joint", configJoint, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	
	
}

//------------------------------------------------------------------------------
pacman::HypothesisSensor::HypothesisSensor(golem::Context& context, Mat34 pose, ConfigQuery configQuery, golem::U32 configJoint, golem::RGBA shapeColour) : configJoint(0) {


	pacman::HypothesisSensor::Desc desc;
	desc.load(context, nullptr);
	desc.appearance.load(nullptr);
	desc.appearance.shapeColour = shapeColour;

	desc.configQuery = configQuery;
	desc.configJoint = configJoint;

	this->viewFrame.setId();
	this->viewFrame.p.z = 0.05;
	this->pose = pose;
	this->create(desc);
	

}
pacman::HypothesisSensor::HypothesisSensor(golem::Context& context) : configJoint(0), configQuery(nullptr), viewFrame(Mat34::identity()) {
	
}

pacman::HypothesisSensor::HypothesisSensor(const pacman::HypothesisSensor::Desc& desc)
{
	pose.setId();
	viewFrame.setId();
	this->create(desc);
}

void pacman::HypothesisSensor::create(const pacman::HypothesisSensor::Desc& desc) {
	


	this->configJoint = desc.configJoint;
	if (hasVariableMounting() && !desc.configQuery)
		throw golem::Message(golem::Message::LEVEL_CRIT, "Sensor::create(): Null config callback interface");
	if (hasVariableMounting())
		configQuery = desc.configQuery;

	appearance = desc.appearance;
	shape.clear();
	shapeFrame.clear();
	for (golem::Bounds::Desc::Seq::const_iterator i = desc.shapeDesc.begin(); i != desc.shapeDesc.end(); ++i) {
		shape.push_back((*i)->create());
		shapeFrame.push_back(shape.back()->getPose());
	}
}

//------------------------------------------------------------------------------

void pacman::HypothesisSensor::getConfig(Config& config) const {
	if (configQuery != nullptr) {
	
		configQuery(configJoint, config);
	}
	else {
		config.setToDefault();
	}
}

//------------------------------------------------------------------------------

void pacman::HypothesisSensor::draw(const Appearance& appearance, golem::DebugRenderer& renderer) const {

	
	const Mat34 frame = this->getFrame();
	if (appearance.shapeShow){
		for (size_t i = 0; i < shape.size(); ++i) {
			shape[i]->setPose(frame*shapeFrame[i]);
			renderer.setColour(appearance.shapeColour);
			renderer.addSolid(*shape[i]);
		}
	}
	if (appearance.frameShow) {
		renderer.addAxes3D(frame, appearance.frameSize);
	}
}

void pacman::HypothesisSensor::setGLView(golem::Scene& scene)
{
	
	this->setGLView(scene,this->getFrame());
}

void pacman::HypothesisSensor::setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame)
{
	golem::CriticalSectionWrapper csw(scene.getCSOpenGL());
	OpenGL::Seq openGL = scene.getOpenGL();

	const golem::Mat34 frame = sensorFrame;
	OpenGL opengl = openGL[0];

	frame.p.get(opengl.viewPoint.v);
	frame.R.getColumn(2, opengl.viewDir);
	frame.R.getColumn(0, opengl.viewUp); //Up right sensor
	//frame.R.getColumn(0, -opengl.viewUp); //Upside down sensor

	opengl.viewDir.normalise();
	opengl.viewUp.normalise();


	scene.setOpenGL(opengl);
}