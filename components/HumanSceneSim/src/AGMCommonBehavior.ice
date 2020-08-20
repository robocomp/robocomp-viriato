//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: AGMCommonBehavior.ice
//  Source: AGMCommonBehavior.idsl
//
//******************************************************************
#ifndef ROBOCOMPAGMCOMMONBEHAVIOR_ICE
#define ROBOCOMPAGMCOMMONBEHAVIOR_ICE
module RoboCompAGMCommonBehavior
{
	enum StateEnum {  Starting, Running, Stopped };
	struct StateStruct
	{
		StateEnum state;
		string info;
	};
	struct Parameter
	{
		bool editable;
		string value;
		string type;
	};
	dictionary <string, Parameter> ParameterMap;
	interface AGMCommonBehavior
	{
		bool activateAgent (ParameterMap prs);
		bool deactivateAgent ();
		ParameterMap getAgentParameters ();
		StateStruct getAgentState ();
		void killAgent ();
		bool reloadConfigAgent ();
		bool setAgentParameters (ParameterMap prs);
		int uptimeAgent ();
	};
};

#endif
