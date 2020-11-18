//
// Created by robolab on 22/07/20.
//


#include "actionExecution.h"

void ActionExecution::initialize(AGMModel::SPtr worldModel_)
{
    worldModel = worldModel_;
}

void ActionExecution::updateWordModel(AGMModel::SPtr worldModel_)
{
    worldModel = worldModel_;

}
void ActionExecution::update(std::string action_,  RoboCompAGMCommonBehavior::ParameterMap params_)
{
    qDebug()<< "ActionExecution" << __FUNCTION__;
    action = action_;
    params = params_;
    newActionReceived = true;


}



//////////////////////////////////missions//////////////////////////////////////////

ActionExecution::retActions ActionExecution::runActions(std::string action_,  RoboCompAGMCommonBehavior::ParameterMap params_, bool testing)
{
//    qDebug() << "---------------------------------------------------";
//    qDebug() <<__FUNCTION__ <<"Checking ACTION: " << QString::fromStdString(action_);

    retActions ret;


    if (action_ == "changeroom")
        ret = action_ChangeRoom(params_,testing);
    else
        prevRoomTarget = "";

    if (action_ == "gotoperson")
        ret = action_GoToPerson(params_);

    if (action_ == "gotoaffordance")
        ret = action_GoToAffordance(params_);

    if (action_ == "gotogroupofpeople")
        ret = action_GoToGroupOfPeople(params_);


    return ret;

}


ActionExecution::retActions ActionExecution::action_ChangeRoom(RoboCompAGMCommonBehavior::ParameterMap params_,bool testing)
{

//    qDebug()<<"-------------------------------"<< __FUNCTION__<< ----------------";
    AGMModelSymbol::SPtr roomSymbol;
    AGMModelSymbol::SPtr robotSymbol;


    try
    {
        roomSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["room2"].value));
        robotSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["robot"].value));
    }
    catch( const Ice::Exception& ex)
    {
        std::cout << "Exception:: Error reading room and robot symbols" << ex << endl;
    }


    int32_t destRoomID = roomSymbol->identifier;
    std::string roomName = roomSymbol->getAttribute("imName");
//    printf("imName: <%s>\n", imName.c_str());

    int32_t currentRoom = -1;

    try
    {

        for (AGMModelSymbol::iterator edge = robotSymbol->edgesBegin(worldModel);
             edge!=robotSymbol->edgesEnd(worldModel);
             edge++) {
            if (edge->getLabel()=="in") {
                const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
                auto second = worldModel->getSymbol(symbolPair.second);
                if (second->typeString() == "room")
                {
                    currentRoom = second->identifier;
                }
            }
        }

    }

    catch( const Ice::Exception& ex)
    {
        std::cout << "Exception:: Cant get edge between robot and room:" << ex << endl;
    }

    if (currentRoom == -1)
    {
        qDebug()<< "shit";
        exit(-1);
    }

    bool needsReplanning = false;

    auto roomPolygon = getRoomPolyline(roomSymbol);
    QPointF newTarget = roomPolygon.boundingRect().center();
//    QPointF newTarget = getRandomPointInRoom(roomPolygon);

    if (currentRoom == destRoomID)
    {
        qDebug()<< "Robot already at target room";
        needsReplanning = false;
    }

    else if (newActionReceived)
    {
        qDebug()<< QString::fromStdString(prevRoomTarget) << QString::fromStdString(roomName);

        needsReplanning = true;
        newActionReceived = false;
    }

//    qDebug()<<  __FUNCTION__ << "Returning "<< needsReplanning << newTarget;
    return std::make_tuple(needsReplanning, newTarget);
};



ActionExecution::retActions ActionExecution::action_GoToPerson(RoboCompAGMCommonBehavior::ParameterMap params_)
{
    qDebug()<<"-------------------------------"<< __FUNCTION__<< "-------------------------------";
    AGMModelSymbol::SPtr personSymbol;
    AGMModelSymbol::SPtr robotSymbol;

    try
    {
        personSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["p"].value));
        robotSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["robot"].value));
    }
    catch( const Ice::Exception& ex)
    {
        std::cout << "Exception:: Error reading room and robot symbols" << ex << endl;
    }


    bool needsReplanning = false;
    QPointF newTarget = getPointInSocialSpace(personSymbol,robotSymbol);

    if (newActionReceived or previousPerson != personSymbol->identifier)
    {

        newActionReceived = false;
        needsReplanning = true;

    }

    previousPerson = personSymbol->identifier;

    qDebug()<<  __FUNCTION__ << "Returning "<< needsReplanning << newTarget;


    return std::make_tuple(needsReplanning, newTarget);


}



ActionExecution::retActions ActionExecution::action_GoToAffordance(RoboCompAGMCommonBehavior::ParameterMap params_)
{
    qDebug()<<"-------------------------------"<< __FUNCTION__<< "-------------------------------";
    AGMModelSymbol::SPtr personSymbol;
    AGMModelSymbol::SPtr robotSymbol;
    AGMModelSymbol::SPtr objectSymbol;

    try
    {
        personSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["p"].value));
        robotSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["robot"].value));
        objectSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["o"].value));
    }
    catch( const Ice::Exception& ex)
    {
        std::cout << "Exception:: Error reading room and robot symbols" << ex << endl;
    }


    bool needsReplanning = false;
    QPointF newTarget = getPointNearAffordance(personSymbol, objectSymbol, robotSymbol);

    if (newActionReceived or previousPerson != personSymbol->identifier)
    {
        newActionReceived = false;
        needsReplanning = true;

    }

    previousPerson = personSymbol->identifier;

    qDebug()<<  __FUNCTION__ << "Returning "<< needsReplanning << newTarget;

    return std::make_tuple(needsReplanning, newTarget);


}

ActionExecution::retActions ActionExecution::action_GoToGroupOfPeople(RoboCompAGMCommonBehavior::ParameterMap params_)
{
    qDebug()<<"-------------------------------"<< __FUNCTION__<< "-------------------------------";
    AGMModelSymbol::SPtr robotSymbol;
    bool needsReplanning = false;

    try
    {
        robotSymbol = worldModel->getSymbolByIdentifier(std::stoi(params_["robot"].value));
    }
    catch( const Ice::Exception& ex)
    {
        std::cout << "Exception:: Error reading robot symbol" << ex << endl;
    }

    vector<AGMModelSymbol::SPtr> totalPerson;

    try
    {

        for (AGMModelSymbol::iterator edge = robotSymbol->edgesBegin(worldModel);
             edge != robotSymbol->edgesEnd(worldModel);
             edge++)
        {
            if (edge->getLabel()=="is_blocking") {
                const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
                totalPerson.push_back(worldModel->getSymbol(symbolPair.first));

            }
        }

    }

    catch( const Ice::Exception& ex)
    {
        std::cout << "Exception:: Cant get edge between robot and person:" << ex << endl;
    }

    auto nearestPerson = getNearestPerson(totalPerson,robotSymbol);
    QPointF newTarget = getPointInSocialSpace(nearestPerson,robotSymbol);

    qDebug()<<"Nearest person is "<< nearestPerson->identifier;

    if (newActionReceived or previousPerson_group != nearestPerson->identifier )
    {
        newActionReceived = false;
        needsReplanning = true;
    }

    previousPerson_group = nearestPerson->identifier;

    return std::make_tuple(needsReplanning, newTarget);

}

AGMModelSymbol::SPtr ActionExecution::getNearestPerson(vector<AGMModelSymbol::SPtr> totalPersons,AGMModelSymbol::SPtr robotSymbol)
{
    float minDist  = std::numeric_limits<int>::max();
    AGMModelSymbol::SPtr nearestPerson;

    localPerson robot;
    auto id_r = robotSymbol->identifier;
    AGMModelSymbol::SPtr robotParent = worldModel->getParentByLink(id_r, "RT");
    AGMModelEdge& edgeRT_r = worldModel->getEdgeByIdentifiers(robotParent->identifier, id_r, "RT");

    robot.id = id_r;
    robot.x = str2float(edgeRT_r.attributes["tx"]);
    robot.z = str2float(edgeRT_r.attributes["tz"]);


    for (auto personSymbol : totalPersons)
    {
        localPerson person;

        auto id = personSymbol->identifier;
        AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
        AGMModelEdge& edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

        person.id = id;
        person.x = str2float(edgeRT.attributes["tx"]);
        person.z = str2float(edgeRT.attributes["tz"]);

        auto dist = sqrt((robot.x - person.x) * (robot.x - person.x) + (robot.z - person.z) * (robot.z - person.z));

        if (dist< minDist)
        {
            nearestPerson = personSymbol;
            minDist = dist;
        }

    }

    return nearestPerson;
}

QPointF ActionExecution::getPointInSocialSpace(AGMModelSymbol::SPtr personSymbol,AGMModelSymbol::SPtr robotSymbol)
{
    qDebug()<<__FUNCTION__;
    localPerson person;

    auto id = personSymbol->identifier;
    AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
    AGMModelEdge& edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

    person.id = id;
    person.x = str2float(edgeRT.attributes["tx"]);
    person.z = str2float(edgeRT.attributes["tz"]);

    localPerson robot;

    auto id_r = robotSymbol->identifier;
    AGMModelSymbol::SPtr robotParent = worldModel->getParentByLink(id_r, "RT");
    AGMModelEdge& edgeRT_r = worldModel->getEdgeByIdentifiers(robotParent->identifier, id_r, "RT");

    robot.id = id_r;
    robot.x = str2float(edgeRT_r.attributes["tx"]);
    robot.z = str2float(edgeRT_r.attributes["tz"]);

    qDebug()<<__FUNCTION__<<"ROBOT POSE "<< robot.x << robot.z ;

    QString personal = QString::fromStdString(personSymbol->getAttribute("polyline_personal"));

    vector<QPolygonF> polygonSeq;
    QPolygonF personalPolygon;

    for(auto pol: personal.split(";;"))
    {
        if(pol.size() == 0)
            continue;

        personalPolygon = QPolygonF();

        for (auto pxz : pol.split(";"))
        {
            auto p = pxz.split(" ");

            if (p.size() != 2)
                continue;

            auto x = std::stof(p[0].toStdString());
            auto z = std::stof(p[1].toStdString());

            personalPolygon << QPointF(x,z);
        }

        //Puede haber varios espacios sociales, se comprueba que la persona esta contenida en el espacio social
        if (personalPolygon.containsPoint(QPointF(person.x,person.z),Qt::OddEvenFill))
            break;
    }

    QLineF line(QPointF(person.x,person.z),QPointF(robot.x,robot.z));

    for (int i = 0; i < 10; i++)
    {
        float step = i/10.0f;

        QPointF point = line.pointAt(step);

        //El punto más cercano a la persona dentro del espacio social será el primer punto no contenido en el personal
        if (!personalPolygon.containsPoint(point,Qt::OddEvenFill)){
            return point;
        }
    }

}


QPointF ActionExecution::getPointNearAffordance(AGMModelSymbol::SPtr personSymbol,AGMModelSymbol::SPtr objectSymbol,AGMModelSymbol::SPtr robotSymbol)
{
    qDebug()<<__FUNCTION__;
    localPerson person;

    auto id = personSymbol->identifier;
    AGMModelSymbol::SPtr personParent = worldModel->getParentByLink(id, "RT");
    AGMModelEdge& edgeRT = worldModel->getEdgeByIdentifiers(personParent->identifier, id, "RT");

    person.id = id;
    person.x = str2float(edgeRT.attributes["tx"]);
    person.z = str2float(edgeRT.attributes["tz"]);

    localPerson robot;

    auto id_r = robotSymbol->identifier;
    AGMModelSymbol::SPtr robotParent = worldModel->getParentByLink(id_r, "RT");
    AGMModelEdge& edgeRT_r = worldModel->getEdgeByIdentifiers(robotParent->identifier, id_r, "RT");

    robot.id = id_r;
    robot.x = str2float(edgeRT_r.attributes["tx"]);
    robot.z = str2float(edgeRT_r.attributes["tz"]);

    qDebug()<<__FUNCTION__<<"ROBOT POSE "<< robot.x << robot.z ;

    QString affordance = QString::fromStdString(objectSymbol->getAttribute("polyline_affordance"));

    vector<QPolygonF> polygonSeq;
    QPolygonF affPolygon;

    for(auto pol: affordance.split(";;"))
    {
        if(pol.size() == 0)
            continue;

        for (auto pxz : pol.split(";"))
        {
            auto p = pxz.split(" ");

            if (p.size() != 2)
                continue;

            auto x = std::stof(p[0].toStdString());
            auto z = std::stof(p[1].toStdString());

            affPolygon<< QPointF(x,z);
        }

    }

    QLineF line(QPointF(person.x,person.z),QPointF(robot.x,robot.z));

    for (int i = 0; i < 10; i++)
    {
        float step = i/10.0f;

        QPointF point = line.pointAt(step);

        if (!affPolygon.containsPoint(point,Qt::OddEvenFill)){
            return point;
        }
    }

}

QPolygonF ActionExecution::getRoomPolyline(AGMModelSymbol::SPtr roomSymbol)
{

    QPolygonF polygon;

    try {
        auto polyline = QString::fromStdString(roomSymbol->getAttribute("polyline"));

        for (auto pxz : polyline.split(";"))
        {
            auto p = pxz.split(" ");

            if (p.size() != 2)
                continue;

            auto x = std::stof(p[0].toStdString());
            auto z = std::stof(p[1].toStdString());

            polygon << QPointF(x,z);
        }

        return polygon;

    }

    catch (std::exception& e) {
        std::cout << "Exception reading room "<< roomSymbol->identifier << " polyline: " << e.what() << std::endl;
    }

}

QPointF ActionExecution::getRandomPointInRoom(QPolygonF room) {

    float hmin, hmax, vmin, vmax;
    auto rect = room.boundingRect();
    
    if (rect.top() < rect.bottom())
    {
        vmin = rect.top() + 200;
        vmax = rect.bottom() - 200;
    }
    else
    {
        vmin = rect.bottom() + 200;
        vmax = rect.top() - 200;
    }

    if (rect.right()<rect.left() )
    {
        hmin = rect.right() + 200;
        hmax = rect.left() - 200;
    }
    else
    {
        hmin = rect.left()+ 200;
        hmax = rect.right() - 200;

    }


    auto x = hmin + (double)rand() * (hmax - hmin)/ (double)RAND_MAX;
    auto z = vmin + (double)rand() * (vmax - vmin)/ (double)RAND_MAX;

    return QPointF(x,z);
}
