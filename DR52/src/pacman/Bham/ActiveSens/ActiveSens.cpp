#include <pacman/Bham/ActiveSens/ActiveSens.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Phys/Data.h>

using namespace pacman;
using namespace golem;
using namespace grasp;


//------------------------------------------------------------------------------

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
	catch (const MsgXMLParserNameNotFound&) {
	}



}

//------------------------------------------------------------------------------
pacman::HypothesisSensor::HypothesisSensor(Mat34 pose, golem::RGBA shapeColour) {


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
	this->pose = pose;
	this->create(desc);


}
pacman::HypothesisSensor::HypothesisSensor(golem::Context& context) : pose(Mat34::identity()), viewFrame(Mat34::identity()) {

}

pacman::HypothesisSensor::HypothesisSensor(const pacman::HypothesisSensor::Desc& desc) : pose(Mat34::identity()), viewFrame(Mat34::identity())
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

	this->setGLView(scene, this->getFrame());
}

void pacman::HypothesisSensor::setGLView(golem::Scene& scene, const golem::Mat34& sensorFrame)
{
	golem::CriticalSectionWrapper csw(scene.getCSOpenGL());
	OpenGL::Seq openGL = scene.getOpenGL();

	const golem::Mat34 frame = sensorFrame;
	OpenGL opengl = openGL[0];

	frame.p.get(opengl.viewPoint.v);
	frame.R.getColumn(2, opengl.viewDir);
	//frame.R.getColumn(0, opengl.viewUp); //Up right sensor
	frame.R.getColumn(0, -opengl.viewUp); //Upside down sensor

	opengl.viewDir.normalise();
	opengl.viewUp.normalise();


	scene.setOpenGL(opengl);
}



//ActiveSense------------------------------------------------------------------------------
ActiveSense::ActiveSense(golem::Context& context)
{
	rand.setRandSeed(context.getRandSeed());
}

void ActiveSense::generateRandomViews(pacman::HypothesisSensor::Seq& viewHypotheses, const golem::Vec3& centroid, const golem::I32& nsamples, const golem::Real& radius)
{
	Mat34 sensorPose;


	golem::Vec3 randomVec;
	golem::Real x, y, z, theta;
	golem::U8 r, g, b, a;

	for (int i = 0; i < nsamples; i++)
	{	
		//Generate random color
		r = (U8)rand.nextUniform<float>(50, 255), g = (U8)rand.nextUniform<float>(50, 255), b = (U8)rand.nextUniform<float>(50, 255), a = 255;
		
		
		//Generate random orientation vector
		randomVec.x = rand.nextUniform<float>(-1, 1);
		randomVec.y = rand.nextUniform<float>(-1, 1);
		randomVec.z = rand.nextUniform<float>(-1, 1);
		randomVec.setMagnitude(radius);

		//Generate a new pose
		sensorPose = golem::Mat34(golem::Mat34::identity());
		sensorPose.p = centroid + randomVec;

		//Generate sensor orientation
		golem::Vec3 f = -randomVec;
		golem::Vec3 up(0.0, 0.0, 1.0);
		golem::Vec3 n = f.cross(up);
		up = n.cross(f);

		//Make sure this is an orthonormal basis
		f.normalise(); up.normalise(); n.normalise();

		//Up right sensor
		/*sensorPose.R.setColumn(0, up);
		sensorPose.R.setColumn(1, n);
		sensorPose.R.setColumn(2, f);
		*/

		//Upside down sensor (The sensor is sticked upside down on Boris' wrist)
		sensorPose.R.setColumn(0, n);
		sensorPose.R.setColumn(1, -up);
		sensorPose.R.setColumn(2, f);

		//Creating uniformly generated random hypothesis sensor
		pacman::HypothesisSensor::Ptr s(new HypothesisSensor(sensorPose, golem::RGBA(r, g, b, a)));
		this->viewHypotheses.push_back(s);

	}
}