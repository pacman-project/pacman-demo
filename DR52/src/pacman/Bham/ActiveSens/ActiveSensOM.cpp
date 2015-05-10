#include <pacman/Bham/ActiveSens/ActiveSensOM.h>

namespace pacman {

	void ActiveSensOnlineModel::setTrainingData(grasp::data::Item::Map::iterator predModel){

		trainingData.clear();

		// collect data (TODO: Receive a list of ItemPredictorModel
		const grasp::data::ItemPredictorModel* model = grasp::is<const grasp::data::ItemPredictorModel>(predModel);
		if (model)
		{
			trainingData.push_back(&model->modelMap);
		}
		else
		{
			printf("ActiveSense: This is not an ItemPredictorModel\n");
		}

	}
	/**
	Sets current point cloud
	*/
	void ActiveSensOnlineModel::updateContacts(const grasp::Contact3D::Seq& graspContacts){
		this->graspContacts.insert(this->graspContacts.end(), graspContacts.begin(), graspContacts.end());
	}

	void ActiveSensOnlineModel::updateContacts(){
		this->graspContacts.clear();

		printf("ActiveSensOnlineModel: Updating contacts...\n");
		for (TrainingData::const_iterator i = trainingData.begin(); i != trainingData.end(); ++i){
			//For each graspType's mapping of joint->contacts do...
			for (grasp::data::ItemPredictorModel::Data::Map::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j) {

				//j->first => GraspType
				//j->second.contacts => Map between joints x contact3D 
				//where contact3D is contains (point,orientation,frame,weight)
				//For each joint's sequence of contact3D regardless of the joint associated with it, do...
				for (grasp::Contact3D::Map::const_iterator k = j->second.contacts.begin(); k != j->second.contacts.end(); k++)
				{
					// k->first => jointToString()
					// k->second => contactSequence
					updateContacts(k->second);
				}

			}
		}
	}

	/**
	Updates viewingAngles for all visitedHypotheses, ordered according to the their order of visit
	Internally, the current set pointCloud is going to be used
	*/
	void ActiveSensOnlineModel::updateViewingAngles(const HypothesisSensor::Seq& visitedHypotheses){

		viewingAngles.clear();
		viewingAngles.resize(this->graspContacts.size(),golem::REAL_MAX);
		printf("ActiveSensOnlineModel: updatingViewingAngles...\n");
		for (HypothesisSensor::Seq::const_iterator it_h = visitedHypotheses.begin(); it_h != visitedHypotheses.end(); it_h++)
		{
			int i = 0;
			for (grasp::Contact3D::Seq::const_iterator it = graspContacts.begin(); it != graspContacts.end(); it++, i++)
			{

				golem::Mat34 frame;
				frame.p = it->point;
				frame.R.fromQuat(it->orientation);


				golem::Vec3 v(0.0, 0.0, 0.0);
				(*it_h)->getFrame().R.getColumn(3, v); //viewDir of hypothesis sensor

				golem::Vec3 n(0.0, 0.0, 0.0);
				frame.R.getColumn(3, n); //Surface Normal vector of contact point
				n.normalise();
				v.normalise();

				golem::Real theta = golem::REAL_PI - golem::Math::acos((n).dot(v));

				viewingAngles[i] = theta < viewingAngles[i] ? theta : viewingAngles[i];
			}

		}

	}
	golem::Real ActiveSensOnlineModel::computeValue(HypothesisSensor::Ptr hypothesis){


		int index = 0;
		golem::Real value(0.0);
		//demoOwner->context.debug("Computing Value\n");
		printf("ActiveSensOnlineModel: Computing value outer...\n");
		for (TrainingData::const_iterator i = trainingData.begin(); i != trainingData.end(); ++i){
			//For each graspType's mapping of joint->contacts do...
			for (grasp::data::ItemPredictorModel::Data::Map::const_iterator j = (*i)->begin(); j != (*i)->end(); ++j) {

				//j->first => GraspType
				//j->second.contacts => Map between joints x contact3D 
				//where contact3D is contains (point,orientation,frame,weight)
				//For each joint's sequence of contact3D regardless of the joint associated with it, do...
				value = golem::Real(0.0);
				for (grasp::Contact3D::Map::const_iterator k = j->second.contacts.begin(); k != j->second.contacts.end(); k++)
				{
					// k->first => jointToString()
					// k->second => contactSequence
					value += computeValue(hypothesis,k->second, index);
					index += static_cast<int>(k->second.size());
				}

				printf("Value %f\n", static_cast<float>(value));

			}
		}

		return value;
	}

	/**
	Computes value for a given hypothesis
	*/
	golem::Real ActiveSensOnlineModel::computeValue(HypothesisSensor::Ptr hypothesis, const grasp::Contact3D::Seq& graspContacts, int index){

		int i = index;

		golem::Real cdf(0.0);
		golem::Real value(0.0);
		printf("ActiveSensOnlineModel: Computing value inner...\n");
		for (grasp::Contact3D::Seq::const_iterator it = graspContacts.begin(); it != graspContacts.end(); it++, i++)
		{

			golem::Mat34 frame;
			frame.p = it->point;
			frame.R.fromQuat(it->orientation);


			golem::Vec3 v(0.0, 0.0, 0.0);
			hypothesis->getFrame().R.getColumn(3, v); //viewDir of hypothesis sensor

			golem::Vec3 n(0.0, 0.0, 0.0);
			frame.R.getColumn(3, n); //Surface Normal vector of contact point
			n.normalise();
			v.normalise();

			golem::Real theta = golem::REAL_PI - golem::Math::acos((n).dot(v));

			golem::Real sigma = theta < viewingAngles[i] ? theta : viewingAngles[i];

			golem::Real weight = it->weight;

			cdf += weight;
			value += weight*sigma;
		}
		value /= cdf;


		return value;

	}


};