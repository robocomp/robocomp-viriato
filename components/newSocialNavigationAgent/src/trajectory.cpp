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

    init_grid();


}

void Trajectory::init_grid()
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

    
    grid.initialize(dimensions, TCell{0, true, false, nullptr, 1.f});
    grid.draw(viewer.get());

}

void Trajectory::update()
{

}

void Trajectory::updatePolylines()
{

}

