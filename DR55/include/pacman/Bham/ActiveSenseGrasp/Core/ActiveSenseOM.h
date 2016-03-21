/** @file ActiveSenseOM.h
*
* Bham Active sensing library
*
* @author	Ermano Arruda
*
*/

#pragma once
#ifndef _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_OM_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_OM_H_


#include <Grasp/Data/ContactModel/ContactModel.h>
#include "pacman/Bham/ActiveSenseGrasp/Core/HypothesisSensor.h"

#include <list>

/** PaCMan name space */
namespace pacman {


	class ActiveSensOnlineModel {

	public:
		grasp::Contact3D::Seq graspContacts;
		std::vector<golem::Real> viewingAngles;
		typedef std::vector<const grasp::data::ItemContactModel::Data::Map*> TrainingData;
		TrainingData trainingData;


		ActiveSensOnlineModel(){}

		/**
		Sets current point cloud
		*/
		void updateContacts(const grasp::Contact3D::Seq& graspContacts);

        void setTrainingData(grasp::data::Item::Map::iterator contactModel);

		void updateContacts();

		

		/**
		Updates viewingAngles for all visitedHypotheses, ordered according to the their order of visit
		Internally, the current set pointCloud is going to be used
		*/
		void updateViewingAngles(const HypothesisSensor::Seq& visitedHypotheses);

		/**
		Computes value for a given hypothesis
		*/
		golem::Real computeValue(HypothesisSensor::Ptr hypothesis, const grasp::Contact3D::Seq& graspContacts, int index = 0);
		golem::Real computeValue(HypothesisSensor::Ptr hypothesis);


	};


};

#endif // _PACMAN_BHAM_ACTIVESENS_ACTIVESENS_OM_H_
