/** @file Collision.h
 *
 * Collision model
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _ACTIVE_SENSE_GRASP_CONTACT_COLLISION_H_
#define _ACTIVE_SENSE_GRASP_CONTACT_COLLISION_H_

//------------------------------------------------------------------------------

#include <Grasp/Contact/Manipulator.h>
#include <Grasp/App/Recorder/Data.h>
#include <Grasp/Core/Search.h>

#include "ActiveSense/Core/Model.h"

//------------------------------------------------------------------------------



namespace flann {
template <typename T> struct L2_Simple;
};

namespace pcl {
struct PointXYZ;
template <typename T, typename Dist> class KdTreeFLANN;
struct PolygonMesh;
};
//------------------------------------------------------------------------------

namespace pacman {

class ActiveSenseDemo;

//------------------------------------------------------------------------------

/** Collision model */
class Collision {
public:
    typedef golem::shared_ptr<Collision> Ptr;
    typedef std::vector<Collision::Ptr> Seq;
    typedef std::vector<grasp::NNSearch::Ptr> NNSearchPtrSeq;

    class Result {
    public:

        active_sense::Model::Voxel::Seq voxels;
        golem::Real entropy, eval;
        size_t collisions, free, unknown;
        size_t total_eval;
        bool changed;
        golem::Real landmark;
        std::vector<double> collisionProfile;
        typedef golem::shared_ptr<Result> Ptr;

        Result(const Result& other) : voxels(other.voxels), entropy(other.entropy), eval(other.eval), collisions(other.collisions), free(other.free), unknown(other.unknown), total_eval(other.total_eval) {

        }

        Result() : landmark(0.0), entropy(0.0), eval(0.0), collisions(0), free(0), unknown(0), total_eval(0), changed(false) {}

        Result::Ptr makeShared(){
            return Result::Ptr(new Result(*this));
        }

        void setToDefault(){
            changed = true;
            entropy = 0.0;
            collisions = 0;
            free = 0;
            unknown = 0;
            eval = 0.0;
            total_eval = 0;
            landmark = 0;
            voxels.clear();
            collisionProfile.clear();


        }

        size_t getEntropyCount(){
            return collisions+free+unknown;
        }

    };

    class Feature {
    public:
        typedef std::vector<Feature> Seq;

        /** As real vector: two surface normals, distance between points, rgb colour */
        static const size_t N = 3;

        /** Appearance */
        class Appearance {
        public:
            /** Line colour */
            golem::RGBA lineColour;

            /** Show normals */
            bool normalShow;
            /** Points' normals size */
            golem::Real normalSize;
            /** Normal colour */
            golem::RGBA normalColour;

            /** Show frame */
            bool frameShow;
            /** Frame axes size */
            golem::Vec3 frameSize;

            /** Constructs description. */
            Appearance() {
                setToDefault();
            }
            /** Sets the parameters to the default values. */
            void setToDefault() {
                lineColour = golem::RGBA::YELLOW;
                normalShow = true;
                normalSize = golem::Real(0.005);
                normalColour = golem::RGBA::CYAN;
                frameShow = false;
                frameSize.set(golem::Real(0.02));
            }
            /** Checks if the description is valid. */
            bool isValid() const {
                if (normalSize <= golem::REAL_ZERO || !frameSize.isPositive())
                    return false;
                return true;
            }
        };

        /** Feature distance metric */
        class FlannDist {
        public:
            typedef bool is_kdtree_distance;
            typedef golem::Real ElementType;
            typedef golem::Real ResultType;

            //			static const size_t LIN_N = 3;

            /** constructor */
            FlannDist() {
            }

            template <typename _Iter1, typename _Iter2> golem::Real operator() (_Iter1 a, _Iter2 b, size_t size = Feature::N, golem::Real worst_dist = -golem::numeric_const<golem::Real>::ONE) const {
                //printf("a=<%f>, b=<%f>, d=%f\n", a[0], b[0], golem::Math::sqrt(golem::Math::sqr(a[0] - b[0])));
                //return golem::Math::sqrt(golem::Math::sqr(a[0] - b[0]));
                return golem::Math::sqrt(golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2]));
            }
            inline golem::Real accum_dist(const golem::Real& a, const golem::Real& b, int) const {
                return golem::Math::sqr(a - b);
            }
        };
        /** Feature normals: points' normals in a local frame  */
        golem::Vec3 point;
        golem::Vec3 normal;
        golem::Mat34 frame;

        /** No member init by default */
        //Feature() {}

        /** Constructor */
        Feature(const golem::Vec3 &point, const golem::Vec3 &normal = golem::Vec3::zero()) {
            this->point.set(point);
            frame.set(golem::Mat33::identity(), point);
            this->normal.set(normal);
        }
        /** Access to data as a single vector */
        inline golem::Real* data() {
            return (golem::Real*)&point.v;
        }
        /** Access to data as a single vector */
        inline const golem::Real* data() const {
            return (const golem::Real*)&point.v;
        }

        /** Access to data as a 3D point */
        inline golem::Vec3 getPoint() {
            return point;
        }
        /** Access to data as a 3D point */
        inline const golem::Vec3 getPoint() const {
            return (const golem::Vec3)point;
        }

        /** Access to data as a 3D point */
        inline golem::Vec3 getNormal() {
            return normal;
        }
        /** Access to data as a 3D point */
        inline const golem::Vec3 getNormal() const {
            return (const golem::Vec3)normal;
        }


        /** Draw feature */
        void draw(const Appearance& appearance, golem::DebugRenderer& renderer) const;

    };



    /** Bounds */
    template <typename _Real, typename _RealEval> class _Bounds {
    public:
        typedef _Real Real;
        typedef _RealEval RealEval;
        typedef golem::_Vec3<_Real> Vec3;
        typedef golem::_Mat33<_Real> Mat33;
        typedef golem::_Mat34<_Real> Mat34;
        typedef std::vector<Vec3> Vec3Seq;

        //typedef std::vector<_Bounds> Seq;
        typedef golem::ScalarCoord<_Bounds, golem::Configspace> Coord;

        /** Surface */
        struct Surface {
            typedef std::vector<Surface> Seq;
            typedef std::vector<Seq> SeqSeq;
            Vec3 point;
            Vec3 point2, point3;
            Vec3 normal;
        };
        /** Triangle */
        struct Triangle : public Surface {
            typedef std::vector<Triangle> Seq;
            typedef std::vector<Seq> SeqSeq;
            Real distance;
        };

        /** Create bounds from convex meshes */
        inline void create(const golem::Bounds::Seq& bounds) {

            golem::Real max_edge = 0;
            for (size_t i = 0; i < bounds.size(); ++i) {
                const golem::BoundingConvexMesh* mesh = dynamic_cast<const golem::BoundingConvexMesh*>(bounds[i].get());
                if (mesh != nullptr) {
                    surfaces.resize(surfaces.size() + 1);
                    triangles.resize(triangles.size() + 1);
                    surfaces.back().resize(mesh->getTriangles().size());
                    triangles.back().resize(mesh->getTriangles().size());
                    for (size_t j = 0; j < mesh->getTriangles().size(); ++j) {
                        triangles.back()[j].normal = surfaces.back()[j].normal = Vec3(mesh->getNormals()[j]);
                        surfaces.back()[j].point = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t1]); // e.g. first triangle
                        surfaces.back()[j].point2 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t2]); // e.g. second triangle
                        surfaces.back()[j].point3 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t3]); // e.g. third triangle
                        //surfaces.back()[j].point = (mesh->getVertices()[mesh->getTriangles()[j].t1] + mesh->getVertices()[mesh->getTriangles()[j].t2] + mesh->getVertices()[mesh->getTriangles()[j].t3])/Real(3.0); // centroid
                        triangles.back()[j].distance = Real(mesh->getDistances()[j]);

                        // Testing things :)
                        golem::Vec3 p0 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t1]);
                        golem::Vec3 p1 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t2]);
                        golem::Vec3 p2 = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t3]);

                        golem::Real edge_max_size = std::max((p1-p0).magnitude(), (p2-p0).magnitude());
                        edge_max_size = std::max(edge_max_size, (p2-p1).magnitude());
                        max_edge = std::max(max_edge,edge_max_size);
                        // end aditional test code


                    }
                }
            }

            printf("TRIANGLE MAX EDGE IS %lf !!!\n", max_edge);
        }

        /** Pose */
        static inline void setPose(const Mat34& pose, const Surface& surface, Triangle& triangle) {
            pose.multiply(triangle.point, surface.point);
            pose.multiply(triangle.point2, surface.point2);
            pose.multiply(triangle.point3, surface.point3);
            pose.R.multiply(triangle.normal, surface.normal);
            triangle.distance = triangle.normal.dot(triangle.point);
        }
        /** Pose */
        static inline void setPose(const Mat34& pose, const typename Surface::Seq& surfaces, typename Triangle::Seq& triangles) {
            triangles.resize(surfaces.size());
            for (size_t i = 0; i < triangles.size(); ++i)
                setPose(pose, surfaces[i], triangles[i]);
        }
        /** Pose */
        inline void setPose(const Mat34& pose) {
            for (size_t i = 0; i < triangles.size(); ++i)
                setPose(pose, surfaces[i], triangles[i]);
        }


        static inline Real getOccupancy(const Collision& collision, const typename Triangle::Seq& triangles, const active_sense::Model::Ptr& model, active_sense::Model::Voxel::Seq& voxels, golem::Real& entropy, size_t& collisions, size_t& free, size_t& unknown, size_t& total_eval) {
            golem::Real eval = golem::numeric_const<golem::Real>::ZERO, c= golem::numeric_const<golem::Real>::ZERO, c2 = golem::numeric_const<golem::Real>::ZERO;  // if no bounds or collisions, no effect
            //int idx = voxels.size();
            //voxels.resize(voxels.size() + triangles.size()*3);
            //int actual_size = 0;
            active_sense::Model::Voxel v1, v2, v3;
            bool v1_ok, v2_ok, v3_ok;
            v1_ok = v2_ok = v3_ok = false;
            for (typename Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {

                golem::Vec3 p = i->point;

                v1_ok = collision.intersect(p);
                model->test(p.x, p.y, p.z, v1);


                p = i->point2;

                v2_ok = collision.intersect(p);


                model->test(p.x, p.y, p.z, v2);
                p = i->point3;

                v3_ok = collision.intersect(p);

                model->test(p.x, p.y, p.z, v3);

                //actual_size+=3;
                golem::Real occupancy(0.0);
                golem::Real local_entropy(0.0);
                float k = 1;//0.0001;
                if( v1_ok ){
                    collisions += size_t(v1.isOccupied());
                    free += size_t(v1.isFree());
                    unknown += size_t(v1.isUnknown());
                    occupancy = golem::Real(v1.occupancy);

                    local_entropy = -(occupancy*golem::Math::log2(occupancy)+(1.0f-occupancy)*golem::Math::log2(1.0f-occupancy));

                    golem::kahanSum(entropy, c2, local_entropy);


                    if( !v1.isUnknown() && !v1.is_contact ) {
                        golem::Real locc = golem::Math::ln(1.0f - occupancy*k);
                        if(locc==locc){
                            golem::kahanSum(eval, c, locc);
                            total_eval++;
                        }
                    }

                    voxels.push_back(v1);
                }

                golem::Real occupancy2(0.0);
                golem::Real local_entropy2(0.0);
                if( v2_ok ){
                    collisions += size_t(v2.isOccupied());
                    free += size_t(v2.isFree());
                    unknown += size_t(v2.isUnknown());
                    occupancy2 = golem::Real(v2.occupancy);

                    golem::Real local_entropy2 = -(occupancy2*golem::Math::log2(occupancy2)+(1.0f-occupancy2)*golem::Math::log2(1.0f-occupancy2));

                    golem::kahanSum(entropy, c2, local_entropy2);

                    if( !v2.isUnknown() && !v2.is_contact ) {
                        golem::Real locc = golem::Math::ln(1.0f - occupancy2*k);
                        if( locc == locc ){
                            golem::kahanSum(eval, c, locc);
                            total_eval++;
                        }
                    }

                    voxels.push_back(v2);
                }

                golem::Real occupancy3(0.0);
                golem::Real local_entropy3(0.0);
                if( v3_ok ){
                    collisions += size_t(v3.isOccupied());
                    free += size_t(v3.isFree());
                    unknown += size_t(v3.isUnknown());
                    occupancy3 = golem::Real(v3.occupancy);

                    local_entropy3 = -(occupancy3*golem::Math::log2(occupancy3)+(1.0f-occupancy3)*golem::Math::log2(1.0f-occupancy3));

                    golem::kahanSum(entropy, c2, local_entropy3);


                    if( !v3.isUnknown() && !v3.is_contact ) {
                        golem::Real locc = golem::Math::ln(1.0f - occupancy3*k);
                        if( locc == locc){
                            golem::kahanSum(eval, c, locc);
                            total_eval++;
                        }
                    }

                    voxels.push_back(v3);
                }







                //printf("occ: %lf local entropy: %lf cumulative_entropy: %lf unk %d\n",occupancy, local_entropy, entropy,voxels[idx].isUnknown() );
                // summing up log_free = sum(log2(1-occupancy)), exp(log_free) = prod(1-occupancy)

                //if(voxels[idx].isFree() && voxels[idx+1].isFree() && voxels[idx+2].isFree()){


                //printf("loc_prob_free0: %lf\n",golem::Math::exp(eval));

                // }

                //printf("loc_prob_free: %lf \n",golem::Math::exp(eval));
            }

            //voxels.resize(voxels.size()-actual_size);

            return eval;
        }


        /** Collision likelihood model */
        inline _RealEval evaluate(const Collision& collision, const active_sense::Model::Ptr& model, active_sense::Model::Voxel::Seq& voxels, golem::Real& entropy, size_t& collisions,  size_t& free, size_t& unknown, size_t& total_eval) const {
            Real eval = golem::numeric_const<Real>::ZERO; // if no bounds or collisions, no effect
            for (typename Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
                eval += getOccupancy(collision, *i, model, voxels, entropy, collisions, free, unknown, total_eval);
            }
            return eval;
        }

        /** Empty */
        inline bool empty() const {
            return surfaces.empty();
        }

        /** Triangles */
        inline const typename Triangle::SeqSeq& getTriangles() const {
            return triangles;
        }
        /** Surfaces */
        inline const typename Surface::SeqSeq& getSurfaces() const {
            return surfaces;
        }

    private:
        /** Triangles */
        typename Triangle::SeqSeq triangles;
        /** Surfaces */
        typename Surface::SeqSeq surfaces;
    };

    /** Collision waypoint */
    class Waypoint {
    public:
        typedef std::vector<Waypoint> Seq;

        /** Path distance */
        golem::Real pathDist;
        /** Number of points */
        golem::U32 points;
        /** Distance standard deviation */
        golem::Real depthStdDev;
        /** Likelihood multiplier */
        golem::Real likelihood;

        /** Constructs description object */
        Waypoint() {
            Waypoint::setToDefault();
        }
        /** Sets the parameters to the default values */
        void setToDefault() {
            pathDist = golem::Real(0.0);
            points = 1000;
            depthStdDev = golem::Real(1000.0);
            likelihood = golem::Real(1000.0);
        }
        /** grasp::Assert that the description is valid. */
        void assertValid(const grasp::Assert::Context& ac) const {
            grasp::Assert::valid(golem::Math::isFinite(pathDist), ac, "pathDist: invalid");
            grasp::Assert::valid(depthStdDev > golem::REAL_EPS, ac, "depthStdDev: < eps");
            grasp::Assert::valid(likelihood >= golem::REAL_ZERO, ac, "likelihood: < 0");
        }
        /** Load descritpion from xml context. */
        void load(const golem::XMLContext* xmlcontext);
    };

    /** Bounds */
    typedef _Bounds<golem::F32, golem::F32> Bounds;

    /** Flann description */
    class FlannDesc {
    public:
        /** Neighbour points during a query */
        golem::U32 neighbours;
        /** MAx neighbouring points selected for collision detection */
        golem::U32 points;
        /** Distance standard deviation */
        golem::Real depthStdDev;
        /** Likelihood multiplier */
        golem::Real likelihood;

        /** Consider a point as a sphere to increase accuracy */
        golem::Real radius;

        /** Contructor */
        FlannDesc() {
            setToDefault();
        }
        /** Nothing to do here */
        virtual ~FlannDesc() {}
        /** Sets the parameters to the default values */
        virtual void setToDefault() {
            neighbours = 1000;
            points = 250;
            depthStdDev = golem::Real(1000.0);
            likelihood = golem::Real(1000.0);

            radius = golem::REAL_ZERO;
        }
        /** Checks if the description is valid. */
        virtual bool isValid() const {
            if (neighbours <= 0 || points < 0)
                return false;
            if (depthStdDev < golem::REAL_EPS || likelihood < golem::REAL_ZERO)
                return false;
            return true;
        }

    };

    /** Collision description */
    class Desc {
    public:
        typedef golem::shared_ptr<Desc> Ptr;

        /** Collision waypoints */
        Waypoint::Seq waypoints;

        /** flann */
        FlannDesc flannDesc;
        /** KDTree decription*/
        grasp::KDTreeDesc nnSearchDesc;

        /** Capture region in global coordinates */
        golem::Bounds::Desc::Seq regionCaptureDesc;

        /** Constructs description object */
        Desc() {
            Desc::setToDefault();
        }
        /** Nothing to do here */
        virtual ~Desc() {}
        /** Creates the object from the description. */
		virtual Collision::Ptr create(const grasp::Manipulator& manipulator, ActiveSenseDemo* demoOwner) const {
            return Collision::Ptr(new Collision(manipulator, demoOwner, *this));
        }
        /** Sets the parameters to the default values */
        virtual void setToDefault() {
            waypoints.clear();
            waypoints.push_back(Waypoint());

            nnSearchDesc.setToDefault();
            flannDesc.setToDefault();

            regionCaptureDesc.clear();


        }
        /** grasp::Assert that the description is valid. */
        virtual void assertValid(const grasp::Assert::Context& ac) const {
            grasp::Assert::valid(!waypoints.empty(), ac, "waypoints: empty");
            for (Waypoint::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i) {
                i->assertValid(grasp::Assert::Context(ac, "waypoints[]."));
            }

            grasp::Assert::valid(flannDesc.isValid(), "flannDesc is not valid.");

            for (golem::Bounds::Desc::Seq::const_iterator i = regionCaptureDesc.begin(); i != regionCaptureDesc.end(); ++i)
                grasp::Assert::valid((*i)->isValid(), ac, "regionCaptureDesc[]: invalid");

        }
        /** Load descritpion from xml context. */
        virtual void load(const golem::XMLContext* xmlcontext);
    };

    /** Create */
    /** Create */
    //virtual void create(golem::Rand& rand, const grasp::data::Point3D& points);
    //virtual void create(golem::Rand& rand, const active_sense::Model::Ptr& model);


    virtual golem::Real evaluate(const grasp::Manipulator::Config& config, active_sense::Model::Voxel::Seq& voxels, golem::Real& entropy, size_t& collisions, size_t& free, size_t& unknown, size_t& total_eval, bool debug);
    virtual golem::Real evaluateProb(const grasp::Manipulator::Waypoint::Seq &path, int eval_size, Result& result, bool debug = true, float alpha = 0.01f);
    void calculateMetrics(size_t total, size_t free, size_t unknown, size_t collisions, golem::Real& pfree, golem::Real& pocc, golem::Real& punknown, golem::Real& entropy2);
    //broken
    bool intersect(const golem::Vec3& p) const {
        bool ret = false;

        ret = golem::Bounds::intersect(regionCapture.begin(), regionCapture.end(), p);
//        for(int i = 0; i < regionCapture.size() && !ret; i++){
//            //golem::BoundingBox* bb = reinterpret_cast<golem::BoundingBox*>(regionCapture[i]);
//            ret = regionCapture[i]->getSurfaceDistance(p) < 0;//regionCapture[i]->intersect(p);
//        }

        //printf("Is in? %lf %lf %lf %d\n",p.x,p.y,p.z, ret);

        return ret;
    }

    /** Joints bounds */
    inline const Bounds::Coord& getJointBounds() const {
        return jointBounds;
    }
    /** Base bounds */
    inline const Bounds& getBaseBounds() const {
        return baseBounds;
    }

    /** Points */
    inline const Feature::Seq& getPoints() const {
        return points;
    }

    void setModel(const active_sense::Model::Ptr& model){
        this->model = model;
    }

protected:
    /** grasp::Manipulator */
    const grasp::Manipulator& manipulator;

    /** KD tree pointer */
    grasp::NNSearch::Ptr nnSearch;

    /** OcTree model pointer */
    active_sense::Model::Ptr model;

    /** Description */
    const Desc desc;

    /** Joints */
    Bounds::Coord jointBounds;
    /** Base */
    Bounds baseBounds;
    /** Points */
    Feature::Seq points;
    //Bounds::Vec3Seq points;

    /** Region capture */
    golem::Bounds::Seq regionCapture;


	ActiveSenseDemo* demoOwner;

    /** Create */
	Collision(const grasp::Manipulator& manipulator, ActiveSenseDemo* demoOwner, const Desc& desc);
};

void XMLData(Collision::Waypoint& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(Collision::FlannDesc& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}	// namespace pacman

//------------------------------------------------------------------------------

#endif /*_ACTIVE_SENSE_GRASP_CONTACT_COLLISION_H_*/
