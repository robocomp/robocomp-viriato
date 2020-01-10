//
// Created by robolab on 9/01/20.
//

#include "trajectory.h"

void Trajectory::initialize(const std::shared_ptr<InnerModel> &innerModel_,
        const std::shared_ptr<InnerViewer> &viewer_,
        const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &configparams_,
        LaserPrx laser_prx,
        OmniRobotPrx omnirobot_proxy)

{
    qDebug()<<"Initializing Trajectory";

    innerModel = innerModel_;
    viewer = viewer_;
    configparams = configparams_;

    checkInitialCollisions();
    initGrid();

}

void Trajectory::update(const std::shared_ptr<InnerModel> &innerModel_)
{
    innerModel = innerModel_;

}

void Trajectory::updatePolylines(const std::shared_ptr<InnerModel> &innerModel_, SNGPersonSeq persons_, SNGPolylineSeq intimate_seq,SNGPolylineSeq personal_seq,SNGPolylineSeq social_seq,SNGPolylineSeq object_seq,SNGPolylineSeq objectsblocking_seq)
{
    innerModel = innerModel_;

    polylines_intimate.clear();
    polylines_personal.clear();
    polylines_social.clear();
    polylines_objects_total.clear();
    polylines_objects_blocked.clear();


    for (auto intimate : intimate_seq)
    {
        QPolygonF polygon;
        for (auto i : intimate)
            polygon << QPointF(i.x, i.z);
        polylines_intimate.push_back(polygon);
    }

    for (auto personal : personal_seq)
    {
        QPolygonF polygon;
        for (auto p : personal)
            polygon << QPointF(p.x, p.z);
        polylines_personal.push_back(polygon);
    }

   for (auto social : social_seq)
    {
        QPolygonF polygon;
        for (auto s : social)
            polygon << QPointF(s.x, s.z);
        polylines_social.push_back(polygon);
    }

   for (auto object : object_seq)
    {
        QPolygonF polygon;
        for (auto o : object)
            polygon << QPointF(o.x, o.z);
        polylines_objects_total.push_back(polygon);
    }

   for (auto object_block : objectsblocking_seq)
    {
        QPolygonF polygon;
        for (auto ob : object_block)
            polygon << QPointF(ob.x, ob.z);
        polylines_objects_blocked.push_back(polygon);
    }


    updateFreeSpaceMap();


}



void Trajectory::checkInitialCollisions()
{
    QStringList ls = QString::fromStdString(configparams->at("ExcludedObjectsInCollisionCheck").value).replace(" ", "" ).split(',');
    qDebug() << __FILE__ << __FUNCTION__ << ls.size() << "objects read for exclusion list";

            foreach(const QString &s, ls)
            excludedNodes.insert(s);

    // Compute the list of meshes that correspond to robot, world and possibly some additionally excluded ones
    robotNodes.clear(); restNodes.clear();
    recursiveIncludeMeshes(innerModel->getRoot(), "robot", false, robotNodes, restNodes, excludedNodes);
};

void Trajectory::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out, std::set<QString> &excluded)
{

    if (node->id == robotId)
    {
        inside = true;
    }

    InnerModelMesh *mesh;
    InnerModelPlane *plane;
    InnerModelTransform *transformation;

    if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
    {
        for (int i=0; i<node->children.size(); i++)
        {
            recursiveIncludeMeshes(node->children[i], robotId, inside, in, out, excluded);

        }

    }

    else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
    {
        if( std::find(excluded.begin(), excluded.end(), node->id) == excluded.end() )
        {
            if (inside)
            {
                in.push_back(node->id);
            }
            else
            if(mesh or plane)
                out.push_back(node->id);
        }
    }

}

bool Trajectory::checkRobotValidStateAtTargetFast(const QVec &targetPos, const QVec &targetRot) const
{
    //First we move the robot in our copy of innermodel to its current coordinates
    innerModel->updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());

    ///////////////////////
    //// Check if the robot at the target collides with any know object
    ///////////////////////
    for ( auto &in : robotNodes )
    {
        for ( auto &out : restNodes )
        {
            if ( innerModel->collide( in, out))
            {
                //qDebug() << __FUNCTION__ << "collision de " << in << " con " << out;
                return false;
            }
        }
    }
    return true;
}


///ADMIN GRID
void Trajectory::initGrid()
{
    try
    {
        dimensions.TILE_SIZE = int(TILE_SIZE_);
        dimensions.HMIN = std::min(std::stof(configparams->at("OuterRegionLeft").value), std::stof(configparams->at("OuterRegionRight").value));
        dimensions.WIDTH = std::max(std::stof(configparams->at("OuterRegionLeft").value), std::stof(configparams->at("OuterRegionRight").value)) - dimensions.HMIN;
        dimensions.VMIN = std::min(std::stof(configparams->at("OuterRegionTop").value), std::stof(configparams->at("OuterRegionBottom").value));
        dimensions.HEIGHT = std::max(std::stof(configparams->at("OuterRegionTop").value), std::stof(configparams->at("OuterRegionBottom").value)) - dimensions.VMIN;

    }
    catch(const std::exception &e)
    {
        std::cout << "Exception " << e.what() << " Trajectory::initialize(). OuterRegion parameters not found in config file" << std::endl;
        //robocomp::exception ex("OuterRegion parameters not found in config file");
        throw e;
    }

    grid.initialize(dimensions, TCell{0, true, false, 1.f});
    grid.draw(viewer.get());

}


void Trajectory::resetGrid()
{
    toSetFree.clear();
    toResetCost.clear();

    toSetFree.insert(toSetFree.end(), prev_polylines_intimate.begin(),prev_polylines_intimate.end());
    toSetFree.insert(toSetFree.end(), prev_polylines_objects_blocked.begin(),prev_polylines_objects_blocked.end());

    toResetCost.insert(toResetCost.end(), prev_polylines_personal.begin(),prev_polylines_personal.end());
    toResetCost.insert(toResetCost.end(), prev_polylines_social.begin(),prev_polylines_social.end());
    toResetCost.insert(toResetCost.end(), prev_polylines_objects_total.begin(),prev_polylines_objects_total.end());


    for (auto &&prev : toSetFree)
    {
        grid.markAreaInGridAs(prev, true);
    }

    for (auto &&prev_cost : toResetCost)
    {
        grid.modifyCostInGrid(prev_cost, 1.0);
    }




}

void Trajectory::updateFreeSpaceMap()
{
    // First remove polygons from last iteration respecting cell occupied by furniture
    resetGrid();

    //To set occupied
    for (auto &&poly_intimate : polylines_intimate)
    {
        grid.markAreaInGridAs(poly_intimate, false);
    }

    for (auto &&poly_object_block : polylines_objects_blocked)
    {
        grid.markAreaInGridAs(poly_object_block, false);
    }

    //To modify cost
    for (auto &&poly_object : polylines_objects_total)
    {
        grid.modifyCostInGrid(poly_object, 1.5);
    }

    for (auto &&poly_soc : polylines_social)
    {
        grid.modifyCostInGrid(poly_soc, 2.0);
    }

    for (auto &&poly_per : polylines_personal)
    {
        grid.modifyCostInGrid(poly_per, 4.0);
    }


    prev_polylines_intimate = polylines_intimate;
    prev_polylines_personal = polylines_personal;
    prev_polylines_social = polylines_social;
    prev_polylines_objects_total = polylines_objects_total;
    prev_polylines_objects_blocked = polylines_objects_blocked;

    grid.draw(viewer.get());

}




