/** @file HypothesisSensor.cpp
*
* Bham Active sensing library
*
* @author	Ermano Arruda
*
*/

#include "pacman/Bham/ActiveSenseGrasp/Core/HypothesisSensor.h"

#include <Golem/Tools/XMLData.h>
#include <Golem/Math/Vec3.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>



int pacman::HypothesisSensor::next_id = 0;

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
pacman::HypothesisSensor::HypothesisSensor(HypothesisSensor::Config config, golem::RGBA shapeColour, int plannerIdx) : config(config.c, config.w), visited(false) {


    pacman::HypothesisSensor::Desc desc;
    // value
    this->value = 0;
	this->plannerIdx = plannerIdx;

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

    this->id = next_id++;
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
            golem::RGBA color = appearance.shapeColour;
            if( this->visited )
                color._rgba.a = 200;

            renderer.setColour(color);
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
    golem::Vec3 test, out;

    opengl.viewPoint = golem::Vec3(frame.p);
    frame.R.getColumn(2, opengl.viewDir);
    //frame.R.getColumn(0, opengl.viewUp); //Up right sensor
    frame.R.getColumn(0, opengl.viewUp); opengl.viewUp = -opengl.viewUp;//Upside down sensor

    opengl.viewDir.normalise();
    opengl.viewUp.normalise();


    scene.setOpenGL(opengl);
}
bool pacman::HypothesisSensor::compareHypothesisSensor(pacman::HypothesisSensor::Ptr h1, pacman::HypothesisSensor::Ptr h2){

    // decreasing order
    return h1->value > h2->value;

}

//------------------------------------------------------------------------------

pacman::PinholeCamera::PinholeCamera(){

    this->w = 640;
    this->h = 480;
    this->x = this->y = 0;

    this->R = cv::Mat::eye(3,3,CV_32FC1);
    this->T = cv::Mat(cv::Vec3f(0,0,0));

    this->r = -1;
    this->l = 1;
    this->b = -1;
    this->t = 1;

    this->n = 1;
    this->f = 2;

    this->setFrustum(l,r,b,t,n,f);
    //cv::namedWindow("View");

}

pacman::PinholeCamera::PinholeCamera(cv::Vec3f rvector, cv::Vec3f translation)
{
    this->w = 640;
    this->h = 480;
    this->x = this->y = 0;

    cv::Rodrigues(rvector,this->R);
    this->T = cv::Mat(translation);

    this->r = -1;
    this->l = 1;
    this->b = -1;
    this->t = 1;

    this->n = 1;
    this->f = 2;

    this->setFrustum(l,r,b,t,n,f);

    //cout << "olha1\n " << this->R << endl;
    //cout << this->T << endl;
}

void setGolemMatToCVMat(const golem::Mat34& gmat_in, cv::Mat cv_mat_out){

    //    golem::Vec3 p = sensorPose.p;
    //    golem::Vec3 col1, col2, col3;

    //    sensorPose.R.getColumn(0,col1);
    //    sensorPose.R.getColumn(1,col2);
    //    sensorPose.R.getColumn(2,col3);

    //    currSensorPose(0,0) = col1.v1; currSensorPose(0,1) = col2.v1; currSensorPose(0,2) = col3.v1; currSensorPose(0,3) = p.v1;
    //    currSensorPose(1,0) = col1.v2; currSensorPose(1,1) = col2.v2; currSensorPose(1,2) = col3.v2; currSensorPose(1,3) = p.v2;
    //    currSensorPose(2,0) = col1.v3; currSensorPose(2,1) = col2.v3; currSensorPose(2,2) = col3.v3; currSensorPose(2,3) = p.v3;
    //    currSensorPose(3,0) = 0;       currSensorPose(3,1) = 0;       currSensorPose(3,2) = 0;       currSensorPose(3,3) = 1;

}
pacman::PinholeCamera::~PinholeCamera(void)
{
}


void pacman::PinholeCamera::loadRotationMatrix(cv::Vec3f rvector)
{
    Rodrigues(rvector, this->R);
}

void pacman::PinholeCamera::loadRotationMatrix(cv::Mat& R)
{
    this->R = R;
}

void pacman::PinholeCamera::loadRotationMatrix(golem::Mat33& R)
{
    this->R = cv::Mat(3,3, CV_32FC1);
    this->R.at<float>(0,0) = R.m11; this->R.at<float>(0,1) = R.m12; this->R.at<float>(0,2) = R.m13;
    this->R.at<float>(1,0) = R.m21; this->R.at<float>(1,1) = R.m22; this->R.at<float>(1,2) = R.m23;
    this->R.at<float>(2,0) = R.m31; this->R.at<float>(2,1) = R.m32; this->R.at<float>(2,2) = R.m33;

}

void pacman::PinholeCamera::loadTranslation(cv::Vec3f translation)
{
    this->T = cv::Mat(translation);
}
void pacman::PinholeCamera::loadTranslation(cv::Mat& translation)
{
    this->T = translation;
}

void pacman::PinholeCamera::loadTranslation(golem::Vec3& translation)
{
    this->loadTranslation(cv::Vec3f(translation.x, translation.y, translation.z));
}

void pacman::PinholeCamera::setFrustum(float l, float r, float b, float t, float n, float f)
{
    float A, B, X, Y, C, D;

    X = 2*n/(r-l);
    Y = 2*n/(t-b);
    C = (-r-l)/(r-l);
    D = (-t-b)/(t-b);
    A = (-f-n)/(n-f);
    B = 2*f*n/(n-f);

    float k[] = { X, 0, C, 0,
                  0, Y, D, 0,
                  0, 0, A, B,
                  0, 0, 1, 0};
    cv::Mat temp = cv::Mat(4,4, CV_32FC1, k);

    this->K = cv::Mat::eye(4,4, CV_32FC1);
    temp.copyTo(this->K);

}
// Primesense ranges:
// FOV: 58° H, 45° V, 70° D (Horizontal, Vertical, Diagonal)
// Distances: Between 0.8m and 3.5m
void pacman::PinholeCamera::setFrustum(float fov_y, float aspect_ratio, float n, float f){

    const float DEG2RAD = 3.14159265 / 180;

    float tangent = tan(fov_y/2 * DEG2RAD);   // tangent of half fov_y
    float height = n * tangent;          // half height of near plane
    float width = height * aspect_ratio;      // half width of near plane

    // params: left, right, bottom, top, near, far
	setFrustum(-width, width, -height, height, n, f);

}



void pacman::PinholeCamera::transform(active_sense::Model::Voxel::Seq& model, std::vector<cv::Mat> & transformedPoints, std::vector<bool>& clip_mask, bool negate_mask )
{
    cv::Mat RT = this->getRT();

    cv::Mat P = K*RT;

    transformedPoints.resize(model.size());
    clip_mask.resize(model.size(),negate_mask);

    std::vector<int> z_buffer(this->w*this->h,active_sense::Model::Voxel::NONE);
    std::vector< std::pair<cv::Point3f,int> > ndc_points;

    cv::Mat point;
    float xc, yc, zc, wc;
    float xn, yn, zn;

    // min heap compare function
    auto pointComp = [](const std::pair<cv::Point3f,int>& p1, const std::pair<cv::Point3f,int>& p2){
        return p1.first.z >  p2.first.z;
    };

    for(int i = 0; i < model.size(); i++)
    {
        Eigen::Vector3f p = model[i].point;
        point = cv::Mat(cv::Vec3f(p.x(),p.y(),p.z()));
        point.resize(4,1); // eye_x eye_y eye_z 1


        transformedPoints[i] = P*point;

        xc = transformedPoints[i].at<float>(0);
        yc = transformedPoints[i].at<float>(1);
        zc = transformedPoints[i].at<float>(2);
        wc = transformedPoints[i].at<float>(3);

        // if outside frustum, then it should be cliped (not visible)
        if( (fabs(xc) > fabs(wc) || fabs(yc) > fabs(wc) || fabs(zc) > fabs(wc)) )
        {
            clip_mask[i] = !negate_mask;

        }

        model[i].is_visible = clip_mask[i];

        if(model[i].is_visible){

            xn = xc/wc;
            yn = yc/wc;
            zn = zc/wc;

            ndc_points.push_back( std::make_pair(cv::Point3f( xn, yn, zn ), i) );
        }


    }

    std::make_heap(ndc_points.begin(), ndc_points.end(), pointComp);

    int px,py;
    int idx = 0;
    int extra_free_count = 0;
    while( !ndc_points.empty() )
    {

        cv::Point3f p = this->viewportTransform( ndc_points.front().first );
        idx = ndc_points.front().second;

        std::pop_heap(ndc_points.begin(), ndc_points.end(), pointComp);

        ndc_points.pop_back();

        px = static_cast<int>(p.x);
        py = static_cast<int>(p.y);

        // Checking the current state of our 'z_buffer',
        // free voxels can be projected on top of the other, NONE can also be projected on top
        // Unknowns are allowed to be projected on top too

        if( (z_buffer[this->w*py + px] == active_sense::Model::Voxel::NONE
                || z_buffer[this->w*py + px] == active_sense::Model::Voxel::FREE
                /*|| z_buffer[this->w*py + px] == active_sense::Model::Voxel::UNKNOWN*/) )/*if( (model[idx].isFree() || model[idx].isOccupied()) && (z_buffer[this->w*py + px] == active_sense::Model::Voxel::NONE
                || z_buffer[this->w*py + px] == active_sense::Model::Voxel::FREE))*/{

            extra_free_count += z_buffer[this->w*py + px] == active_sense::Model::Voxel::FREE;
            z_buffer[this->w*py + px] = model[idx].state;

        }
        // otherwise, position (x,y) has a occupied voxel projected there
        // this voxel should occlude all others after, so we cannot say model[idx] is visible
        else{
            clip_mask[idx] = !negate_mask;
            z_buffer[this->w*py + px] = model[idx].state;

        }




    }

//    cv::Mat buf(480, 640, CV_8UC3);
//    int hist[4] = {0,0,0,0};
//    for(int i = 0; i < 480; i++){
//        for(int j = 0; j < 640; j++){
//            cv::Scalar color;
//            if(z_buffer[this->w*i + j]== active_sense::Model::Voxel::UNKNOWN){

//                color = cv::Scalar(255,0,0);
//                hist[0]++;
//            }
//            else if(z_buffer[this->w*i + j]== active_sense::Model::Voxel::OCC){
//                color = cv::Scalar(0,255,0);
//                hist[1]++;
//            }
//            else if(z_buffer[this->w*i + j]== active_sense::Model::Voxel::FREE){
//                color = cv::Scalar(0,125,255);
//                hist[2]++;
//            }
//            else{ //None
//                color = cv::Scalar(0,0,0);
//                hist[3]++;
//            }
//            cv::circle(buf, cv::Point(j,i), 5,color,2);
//            //buf.at<unsigned char>(i,j) = static_cast<unsigned char>((z_buffer[this->w*i + j]== active_sense::Model::Voxel::UNKNOWN)*255.0 / 4.0);
//    }
//    }
//    //        cv::Mat buf_out;

//    //applyColorMap(buf, buf_out, cv::COLORMAP_AUTUMN);

//    cv::imshow("View",buf);
//    hist[1] = extra_free_count;
//    printf("Count unk: %d occ %d free %d total_known %d\n",hist[0],hist[1],hist[2],hist[1]+hist[2]);
//    cv::waitKey(0);





}

cv::Point3f pacman::PinholeCamera::viewportTransform( const cv::Point3f& point )
{
    cv::Point3f viewportPoint;

    viewportPoint.x = point.x*this->w/2 + (this->x + this->w/2);
    viewportPoint.y = point.y*this->h/2 + (this->y + this->h/2);
    viewportPoint.z = point.z*(this->f - this->n)/2 + (this->f + this->n)/2;


    return viewportPoint;
}

cv::Mat pacman::PinholeCamera::getRT()
{

    cv::Mat RT = cv::Mat::eye(4,4, CV_32FC1); //Extrinsic matrix

    cv::Mat RT_r = RT(cv::Range(0,3),cv::Range(0,3)); //Sub matrix: rotation part
    cv::Mat RT_t = RT(cv::Range(0,3), cv::Range(3,4)); //Sub matrix: translation part

    //Copying data to RT
    this->R.copyTo(RT_r);

    cv::Mat t = this->T;//-this->R*this->T;

    t.copyTo(RT_t);

    return RT;
}

