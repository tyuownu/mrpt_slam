/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef TUSEROPTIONSCHECKER_ROS_IMPL_H
#define TUSEROPTIONSCHECKER_ROS_IMPL_H

namespace mrpt { namespace graphslam { namespace apps {

template<class GRAPH_T>
TUserOptionsChecker_ROS<GRAPH_T>::TUserOptionsChecker_ROS() {
}

template<class GRAPH_T>
TUserOptionsChecker_ROS<GRAPH_T>::~TUserOptionsChecker_ROS() {
}

template<class GRAPH_T>
void TUserOptionsChecker_ROS<GRAPH_T>::createDeciderOptimizerMappings() {
	using namespace std;
	using namespace mrpt::graphs;
	using namespace mrpt::graphslam::apps;
	using namespace mrpt::graphslam::deciders;
	parent::createDeciderOptimizerMappings();

	// node registration deciders
	this->node_regs_map["CICPCriteriaNRD_MR"] =
		parent::template createNodeRegistrationDecider<CICPCriteriaNRD_MR<GRAPH_T>>;
	this->node_regs_map["CFixedIntervalsNRD_MR"] =
		parent::template createNodeRegistrationDecider<CFixedIntervalsNRD_MR<GRAPH_T>>;


	// edge registration deciders
	this->edge_regs_map["CLoopCloserERD_MR"] =
		parent::template createEdgeRegistrationDecider<CLoopCloserERD_MR<GRAPH_T>>;

	// optimizers

}

template<class GRAPH_T>
void TUserOptionsChecker_ROS<GRAPH_T>::populateDeciderOptimizerProperties() {
	using namespace mrpt::graphslam::apps;
	using namespace std;

	parent::populateDeciderOptimizerProperties();
	{ // CICPCriteriaNRD_MR
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CICPCriteriaNRD_MR";
		dec->description =
			"Multi-robot SLAM implementation of the CICPCriteriaNRD class based on \"Condensed Measurements\"";
		dec->type = "Node";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
		dec->is_mr_slam_class = "true";

		this->regs_descriptions.push_back(dec);
	}
	{ // CFixedIntervalsNRD_MR
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CFixedIntervalsNRD_MR";
		dec->description =
			"Multi-robot SLAM implementation of the CFixedIntervalsNRD class based on \"Condensed Measurements\"";
		dec->type = "Node";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
		dec->is_mr_slam_class = "true";

		this->regs_descriptions.push_back(dec);
	}

	{ // CLoopCloserERD_MR
		TRegistrationDeciderProps* dec = new TRegistrationDeciderProps;
		dec->name = "CLoopCloserERD_MR";
		dec->description =
			"Multi-robot SLAM implementation of the CLoopCloserERD class based on \"Condensed Measurements\"";
		dec->type = "Edge";
		dec->rawlog_format = "Both";
		dec->observations_used.push_back("CObservation2DRangeScan - Format #1, #2");
		dec->is_mr_slam_class = "true";

		this->regs_descriptions.push_back(dec);
	}

}

} } } //end namespaces

#endif /* end of include guard: TUSEROPTIONSCHECKER_ROS_IMPL_H */
