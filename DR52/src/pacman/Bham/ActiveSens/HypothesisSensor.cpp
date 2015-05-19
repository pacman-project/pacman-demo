#include <pacman/Bham/ActiveSens/HypothesisSensor.h>

#include <Golem/Tools/XMLData.h>

void pacman::HypothesisSensor::Appearance::load(const golem::XMLContext* xmlcontext) {


	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame"), false);
	golem::XMLData("show", frameShow, xmlcontext->getContextFirst("frame"), false);
	golem::XMLData(shapeColour, xmlcontext->getContextFirst("shape"), false);
	golem::XMLData("show", shapeShow, xmlcontext->getContextFirst("shape"), false);


}

//------------------------------------------------------------------------------

void pacman::HypothesisSensor::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {


	appearance.load(xmlcontext->getContextFirst("appearance"));
	try {
		golem::XMLData(shapeDesc, shapeDesc.max_size(), xmlcontext->getContextFirst("appearance shape"), "bounds", false);
	}
	catch (const golem::MsgXMLParserNameNotFound&) {
	}



}

//------------------------------------------------------------------------------
pacman::HypothesisSensor::HypothesisSensor(HypothesisSensor::Config config, golem::RGBA shapeColour) : config(config.c, config.w), visited(false) {


	pacman::HypothesisSensor::Desc desc;

	//Appearance of the Hypothesis Sensor
	desc.appearance.frameSize.set(0.05, 0.05, 0.05);
	desc.appearance.frameShow = true;
	desc.appearance.shapeShow = true;
	desc.appearance.shapeColour = shapeColour;

	//Bounds description
	golem::BoundingBox::Desc boundDesc;
	boundDesc.dimensions.set(0.025, 0.025, 0.025);
	boundDesc.pose.setId();
	boundDesc.pose.p.z -= 0.025;
	golem::BoundingBox::Desc::Ptr descBox(new golem::BoundingBox::Desc(boundDesc));

	desc.shapeDesc.push_back(descBox);

	//Local view frame 
	//Frame is attached on the frontal face of the cube representing this Hypothesis Sensor
	desc.viewFrame.setId();
	desc.viewFrame.p.z = 0.05;

	//Pose with respect to base frame
	//this->config = config;
	this->create(desc);


}
pacman::HypothesisSensor::HypothesisSensor(golem::Context& context) : config(golem::Mat34::identity()), viewFrame(golem::Mat34::identity()), visited(false) {

}

pacman::HypothesisSensor::HypothesisSensor(const pacman::HypothesisSensor::Desc& desc) : config(golem::Mat34::identity()), viewFrame(golem::Mat34::identity()), visited(false)
{
	this->create(desc);
}

void pacman::HypothesisSensor::create(const pacman::HypothesisSensor::Desc& desc) {

	this->viewFrame = desc.viewFrame;
	
	appearance = desc.appearance;
	shape.clear();
	shapeFrame.clear();
	for (golem::Bounds::Desc::Seq::const_iterator i = desc.shapeDesc.begin(); i != desc.shapeDesc.end(); ++i) {
		shape.push_back((*i)->create());
		shapeFrame.push_back(shape.back()->getPose());
	}
}


//------------------------------------------------------------------------------

void pacman::HypothesisSensor::draw(const Appearance& appearance, golem::DebugRenderer& renderer) const {


	const golem::Mat34 frame = this->getFrame();
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

	this->setGLView(scene, this->getFrame());
}

void pacman::HypothesisSensor::setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame)
{
	golem::CriticalSectionWrapper csw(scene.getUniverse().getCS());
	golem::OpenGL::Seq openGL = scene.getOpenGL();

	const golem::Mat34 frame = sensorFrame;
	golem::OpenGL opengl = openGL[0];

	frame.p.get(opengl.viewPoint.v);
	frame.R.getColumn(2, opengl.viewDir);
	//frame.R.getColumn(0, opengl.viewUp); //Up right sensor
	frame.R.getColumn(0, -opengl.viewUp); //Upside down sensor

	opengl.viewDir.normalise();
	opengl.viewUp.normalise();


	scene.setOpenGL(opengl);
}

