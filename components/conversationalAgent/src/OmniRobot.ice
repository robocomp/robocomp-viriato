//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: OmniRobot.ice
//  Source: OmniRobot.idsl
//
//******************************************************************
#ifndef ROBOCOMPOMNIROBOT_ICE
#define ROBOCOMPOMNIROBOT_ICE
#include <GenericBase.ice>
module RoboCompOmniRobot
{
	struct TMechParams
	{
		float temp;
		string device;
		string handler;
		float maxVelAdv;
		float maxVelRot;
	};
	interface OmniRobot
	{
		void correctOdometer (int x, int z, float alpha) throws RoboCompGenericBase::HardwareFailedException;
		void getBasePose (out int x, out int z, out float alpha) throws RoboCompGenericBase::HardwareFailedException;
		void getBaseState (out RoboCompGenericBase::TBaseState state) throws RoboCompGenericBase::HardwareFailedException;
		void resetOdometer () throws RoboCompGenericBase::HardwareFailedException;
		void setOdometer (RoboCompGenericBase::TBaseState state) throws RoboCompGenericBase::HardwareFailedException;
		void setOdometerPose (int x, int z, float alpha) throws RoboCompGenericBase::HardwareFailedException;
		void setSpeedBase (float advx, float advz, float rot) throws RoboCompGenericBase::HardwareFailedException;
		void stopBase () throws RoboCompGenericBase::HardwareFailedException;
	};
};

#endif
