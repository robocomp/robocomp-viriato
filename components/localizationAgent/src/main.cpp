/*
 *    Copyright (C) 2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */


/** \mainpage RoboComp::localizationAgent
 *
 * \section intro_sec Introduction
 *
 * The localizationAgent component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd localizationAgent
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/localizationAgent --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <agmcommonbehaviorI.h>
#include <agmexecutivetopicI.h>
#include <apriltagsI.h>
#include <fullposeestimationpubI.h>
#include <fullposeestimationpubI.h>

#include <Planning.h>
#include <GenericBase.h>
#include <FullPoseEstimation.h>
#include <AGMWorldModel.h>
#include <JointMotor.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

class localizationAgent : public RoboComp::Application
{
public:
	localizationAgent (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void ::localizationAgent::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::localizationAgent::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);  // NON-GUI application


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	AGMExecutivePrx agmexecutive_proxy;
	OmniRobotPrx omnirobot_proxy;

	string proxy, tmp;
	initialize();


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "AGMExecutiveProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AGMExecutiveProxy\n";
		}
		agmexecutive_proxy = AGMExecutivePrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy AGMExecutive: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("AGMExecutiveProxy initialized Ok!");

	mprx["AGMExecutiveProxy"] = (::IceProxy::Ice::Object*)(&agmexecutive_proxy);//Remote server proxy creation example

	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "OmniRobotProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy OmniRobotProxy\n";
		}
		omnirobot_proxy = OmniRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy OmniRobot: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("OmniRobotProxy initialized Ok!");

	mprx["OmniRobotProxy"] = (::IceProxy::Ice::Object*)(&omnirobot_proxy);//Remote server proxy creation example
	IceStorm::TopicManagerPrx topicManager;
	try
	{
		topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: STORM not running: " << ex << endl;
		return EXIT_FAILURE;
	}

	SpecificWorker *worker = new SpecificWorker(mprx);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "AGMCommonBehavior.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AGMCommonBehavior";
			}
			Ice::ObjectAdapterPtr adapterAGMCommonBehavior = communicator()->createObjectAdapterWithEndpoints("AGMCommonBehavior", tmp);
			AGMCommonBehaviorI *agmcommonbehavior = new AGMCommonBehaviorI(worker);
			adapterAGMCommonBehavior->add(agmcommonbehavior, Ice::stringToIdentity("agmcommonbehavior"));
			adapterAGMCommonBehavior->activate();
			cout << "[" << PROGRAM_NAME << "]: AGMCommonBehavior adapter created in port " << tmp << endl;
			}
			catch (const IceStorm::TopicExists&){
				cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for AGMCommonBehavior\n";
			}



		// Server adapter creation and publication
		IceStorm::TopicPrx agmexecutivetopic_topic;
		Ice::ObjectPrx agmexecutivetopic;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "AGMExecutiveTopicTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AGMExecutiveTopicProxy";
			}
			Ice::ObjectAdapterPtr AGMExecutiveTopic_adapter = communicator()->createObjectAdapterWithEndpoints("agmexecutivetopic", tmp);
			AGMExecutiveTopicPtr agmexecutivetopicI_ =  new AGMExecutiveTopicI(worker);
			Ice::ObjectPrx agmexecutivetopic = AGMExecutiveTopic_adapter->addWithUUID(agmexecutivetopicI_)->ice_oneway();
			if(!agmexecutivetopic_topic)
			{
				try {
					agmexecutivetopic_topic = topicManager->create("AGMExecutiveTopic");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						agmexecutivetopic_topic = topicManager->retrieve("AGMExecutiveTopic");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				IceStorm::QoS qos;
				agmexecutivetopic_topic->subscribeAndGetPublisher(qos, agmexecutivetopic);
			}
			AGMExecutiveTopic_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating AGMExecutiveTopic topic.\n";
			//Error. Topic does not exist
		}

		// Server adapter creation and publication
		IceStorm::TopicPrx apriltags_topic;
		Ice::ObjectPrx apriltags;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "AprilTagsTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AprilTagsProxy";
			}
			Ice::ObjectAdapterPtr AprilTags_adapter = communicator()->createObjectAdapterWithEndpoints("apriltags", tmp);
			AprilTagsPtr apriltagsI_ =  new AprilTagsI(worker);
			Ice::ObjectPrx apriltags = AprilTags_adapter->addWithUUID(apriltagsI_)->ice_oneway();
			if(!apriltags_topic)
			{
				try {
					apriltags_topic = topicManager->create("AprilTags");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						apriltags_topic = topicManager->retrieve("AprilTags");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				IceStorm::QoS qos;
				apriltags_topic->subscribeAndGetPublisher(qos, apriltags);
			}
			AprilTags_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating AprilTags topic.\n";
			//Error. Topic does not exist
		}

		// Server adapter creation and publication
		IceStorm::TopicPrx fullposeestimationpub_topic;
		Ice::ObjectPrx fullposeestimationpub;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimationPubTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimationPubProxy";
			}
			Ice::ObjectAdapterPtr FullPoseEstimationPub_adapter = communicator()->createObjectAdapterWithEndpoints("fullposeestimationpub", tmp);
			FullPoseEstimationPubPtr fullposeestimationpubI_ =  new FullPoseEstimationPubI(worker);
			Ice::ObjectPrx fullposeestimationpub = FullPoseEstimationPub_adapter->addWithUUID(fullposeestimationpubI_)->ice_oneway();
			if(!fullposeestimationpub_topic)
			{
				try {
					fullposeestimationpub_topic = topicManager->create("FullPoseEstimationPub");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						fullposeestimationpub_topic = topicManager->retrieve("FullPoseEstimationPub");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				IceStorm::QoS qos;
				fullposeestimationpub_topic->subscribeAndGetPublisher(qos, fullposeestimationpub);
			}
			FullPoseEstimationPub_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating FullPoseEstimationPub topic.\n";
			//Error. Topic does not exist
		}

		// Server adapter creation and publication
		IceStorm::TopicPrx fullposeestimationpub_topic;
		Ice::ObjectPrx fullposeestimationpub1;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "FullPoseEstimationPubTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy FullPoseEstimationPubProxy";
			}
			Ice::ObjectAdapterPtr FullPoseEstimationPub_adapter = communicator()->createObjectAdapterWithEndpoints("fullposeestimationpub", tmp);
			FullPoseEstimationPubPtr fullposeestimationpubI_ =  new FullPoseEstimationPubI(worker);
			Ice::ObjectPrx fullposeestimationpub1 = FullPoseEstimationPub_adapter->addWithUUID(fullposeestimationpubI_)->ice_oneway();
			if(!fullposeestimationpub_topic)
			{
				try {
					fullposeestimationpub_topic = topicManager->create("FullPoseEstimationPub");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						fullposeestimationpub_topic = topicManager->retrieve("FullPoseEstimationPub");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				IceStorm::QoS qos;
				fullposeestimationpub_topic->subscribeAndGetPublisher(qos, fullposeestimationpub1);
			}
			FullPoseEstimationPub_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating FullPoseEstimationPub topic.\n";
			//Error. Topic does not exist
		}

		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();

		try
		{
			std::cout << "Unsubscribing topic: agmexecutivetopic " <<std::endl;
			agmexecutivetopic_topic->unsubscribe( agmexecutivetopic );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: agmexecutivetopic " <<std::endl;
		}
		try
		{
			std::cout << "Unsubscribing topic: apriltags " <<std::endl;
			apriltags_topic->unsubscribe( apriltags );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: apriltags " <<std::endl;
		}
		try
		{
			std::cout << "Unsubscribing topic: fullposeestimationpub " <<std::endl;
			fullposeestimationpub_topic->unsubscribe( fullposeestimationpub );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: fullposeestimationpub " <<std::endl;
		}
		try
		{
			std::cout << "Unsubscribing topic: fullposeestimationpub " <<std::endl;
			fullposeestimationpub_topic->unsubscribe( fullposeestimationpub );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: fullposeestimationpub " <<std::endl;
		}

		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	std::string configFile = "config";
	if (argc > 1)
	{
		std::string initIC("--Ice.Config=");
		size_t pos = std::string(argv[1]).find(initIC);
		if (pos == 0)
		{
			configFile = std::string(argv[1]+initIC.size());
		}
		else
		{
			configFile = std::string(argv[1]);
		}
	}

	// Search in argument list for --prefix= argument (if exist)
	QString prefix("");
	QString prfx = QString("--prefix=");
	for (int i = 2; i < argc; ++i)
	{
		arg = argv[i];
		if (arg.find(prfx.toStdString(), 0) == 0)
		{
			prefix = QString::fromStdString(arg).remove(0, prfx.size());
			if (prefix.size()>0)
				prefix += QString(".");
			printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
		}
	}
	::localizationAgent app(prefix);

	return app.main(argc, argv, configFile.c_str());
}
