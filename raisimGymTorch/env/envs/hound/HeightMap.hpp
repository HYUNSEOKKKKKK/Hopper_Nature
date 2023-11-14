//
// Created by gijeong on 23. 11. 14.
//

#ifndef _RAISIM_GYM_TORCH_HEIGHTMAP_HPP
#define _RAISIM_GYM_TORCH_HEIGHTMAP_HPP

#include "raisim/RaisimServer.hpp"

raisim::HeightMap* HeightMapSample(raisim::World* world, int heightMapType, double curriculum, std::mt19937& gen, std::uniform_real_distribution<double>& uniDist){
    /// heightMapType:
    /// 0 -> plain with roughness
    /// 1 -> slope
    /// 2 -> stair
    /// 3 -> big stair
    /// 4 -> slope
    /// curriculum : 0 ~ 3.0, max curriculum : 3.0

    /// 경사로
    if (heightMapType == 0){
        /// plain
        double hardness = (uniDist(gen) < 0.0) ? curriculum : curriculum * abs(uniDist(gen)); // 50% : curriculum, 50% : 0~curriculum
        double roughness = hardness/3.0;
        int xSampleNum = 50;
        int ySampleNum = 50;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        for (int i=0; i<xSampleNum; i++){
            for (int j=0; j<ySampleNum; j++){
                heightVec[j*xSampleNum + i] = (0.15 * roughness) * uniDist(gen) + 0.1;
            }
        }

        double y_size = 20;
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, y_size, 0., 0., heightVec);
    }


    else if (heightMapType == 1){
        /// slope
        double hardness = (uniDist(gen) < 0.5) ? curriculum : curriculum * abs(uniDist(gen)); // 75% : curriculum, 25% : 0~curriculum
        double roughness = abs(uniDist(gen));
        double heightMax = 0.21 * hardness * 20;
        int xSampleNum = 50;
        int ySampleNum = 50;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        for (int i=0; i<xSampleNum; i++){
            for (int j=0; j<ySampleNum; j++){
                heightVec[j*xSampleNum + i] = j * (heightMax/(double)ySampleNum) + (0.06 * roughness) * uniDist(gen);
            }
        }

        double y_size = 20;
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, y_size, 0., 0., heightVec);
    }


    else if (heightMapType == 2){
        /// stair
        double hardness = (uniDist(gen) < 0.0) ? curriculum : curriculum * abs(uniDist(gen)); // 50% : curriculum, 50% : 0~curriculum
        double stepHeight = 0.07 * hardness;
        int xSampleNum = 2;
        int ySampleNum = 2000;
        int stairNum = 50;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        int step = 0;
        for (int j=0; j<ySampleNum; j++){
            heightVec[j*2] = ceil((double)step/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 50 계단
            heightVec[j*2+1] = heightVec[j*2];
            step++;
        }

        double y_size = (0.275 + 0.075 * uniDist(gen)) * (double)stairNum; // 폭 [20,35]
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, y_size, 0., 0., heightVec);
    }


    else if(heightMapType == 3){
        /// big stair
        double hardness = (uniDist(gen) < 0.0) ? curriculum : curriculum * abs(uniDist(gen)); // 50% : curriculum, 50% : 0~curriculum
        double stepHeight = 0.115 * hardness;
        int xSampleNum = 2;
        int ySampleNum = 1800;
        int stairNum = 12;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        int step = 0;
        for (int j=0; j<ySampleNum; j++){
            heightVec[j*2] = ceil((double)step/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 12 계단
            heightVec[j*2+1] = heightVec[j*2];
            step++;
        }

        double y_size = (1.7 + 0.3 * uniDist(gen)) * (double)stairNum; // 폭 [140,200]
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, y_size, 0., 0., heightVec);
    }



    else{
        /// sqaure
        double hardness = (uniDist(gen) < 0.0) ? curriculum : curriculum * abs(uniDist(gen)); // 50% : curriculum, 50% : 0~curriculum
        double roughness = hardness/3.0;
        int xSampleNum = 480;
        int ySampleNum = 480;

        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        Eigen::Matrix<double,10,10> squareHeight;
        for (int i=0;i<10;i++){
            for (int j=0;j<10;j++){
                squareHeight(i,j) = 0.2 + (0.1 * roughness) * uniDist(gen);
            }
        }

        for (int i=0; i<xSampleNum; i++){
            for (int j=0; j<ySampleNum; j++){
                heightVec[j*xSampleNum + i] = squareHeight((i/12)%10,(j/12)%10);
            }
        }

        double y_size = 20;
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, y_size, 0., 0., heightVec);
        }
}

#endif //_RAISIM_GYM_TORCH_HEIGHTMAP_HPP
