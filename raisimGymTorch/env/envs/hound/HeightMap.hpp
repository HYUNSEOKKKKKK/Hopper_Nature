//
// Created by gijeong on 23. 11. 14.
//

#ifndef _RAISIM_GYM_TORCH_HEIGHTMAP_HPP
#define _RAISIM_GYM_TORCH_HEIGHTMAP_HPP

#include "raisim/RaisimServer.hpp"

raisim::HeightMap* HeightMapSample(raisim::World* world, int heightMapType, double curriculum, std::mt19937& gen, std::uniform_real_distribution<double>& uniDist){
    /// heightMapType:
    /// 0 -> plain with roughness (or square)
    /// 1 -> slope
    /// 2 -> stair
    /// 3 -> big stair
    /// curriculum : 0 ~ 3.0, max curriculum : 3.0
    double hardness = (uniDist(gen) < 0.0) ? curriculum : curriculum * abs(uniDist(gen)); // 50% : curriculum, 50% : 0~curriculum
//        double hardness = curriculum; // 50% : curriculum, 50% : 0~curriculum

    if (heightMapType == 0){
        double roughness = hardness/3.0;
        std::vector<double> heightVec;

        if (uniDist(gen) < 0.0){
            /// plain
            int xSampleNum = 50;
            int ySampleNum = 50;

            heightVec.resize(xSampleNum*ySampleNum);

            for (int i=0; i<xSampleNum; i++){
                for (int j=0; j<ySampleNum; j++){
                    heightVec[j*xSampleNum + i] = (0.10 * roughness) * uniDist(gen) + 0.2;
                }
            }
            double ySize = 20;
            return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
        }else{
            /// sqaure
            int xSampleNum = 480;
            int ySampleNum = 480;

            heightVec.resize(xSampleNum*ySampleNum);

            Eigen::Matrix<double,10,10> squareHeight;
            for (int i=0;i<10;i++){
                for (int j=0;j<10;j++){
                    squareHeight(i,j) = 0.2 + (0.08 * roughness) * uniDist(gen);
                }
            }
            for (int i=0; i<xSampleNum; i++){
                for (int j=0; j<ySampleNum; j++){
                    heightVec[j*xSampleNum + i] = squareHeight((i/12)%10,(j/12)%10);
                }
            }
            double ySize = 20;
            return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
        }
    }


    else if (heightMapType == 1){
        /// slope
        double roughness = abs(uniDist(gen));
        double heightMax = 0.17 * hardness * 20;
        int xSampleNum = 50;
        int ySampleNum = 50;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        for (int i=0; i<xSampleNum; i++){
            for (int j=0; j<ySampleNum; j++){
                heightVec[j*xSampleNum + i] = j * (heightMax/(double)ySampleNum) + (0.02 * roughness) * uniDist(gen) + 0.2;
            }
        }

        double ySize = 20;
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
    }


    else if (heightMapType == 2){
        /// stair
        double stepHeight = 0.065 * hardness;
        int xSampleNum = 6;
        int ySampleNum = 3600; //3600;
        int stairNum = 60;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        int step = 0;
        for (int j=0; j<ySampleNum; j++){
            heightVec[j*xSampleNum] = ceil((double)step/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 50 계단
            for (int i=1; i<xSampleNum; i++){
                heightVec[j*xSampleNum+i] = heightVec[j*xSampleNum];
            }
            step++;
        }

        double ySize = (0.325 + 0.075 * uniDist(gen)) * (double)stairNum; // 폭 [25,40]
//        double ySize = 0.25 * (double)stairNum; // 폭 [20,40]
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
    }


    else if(heightMapType == 3){
        /// big stair
        double stepHeight = 0.115 * hardness;
        int xSampleNum = 6;
        int ySampleNum = 3600;
        int stairNum = 12;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        int step = 0;
        for (int j=0; j<ySampleNum; j++){
            heightVec[j*xSampleNum] = ceil((double)step/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 12 계단
            for (int i=1; i<xSampleNum; i++){
                heightVec[j*xSampleNum+i] = heightVec[j*xSampleNum];
            }
            step++;
        }

        double ySize = (1.7 + 0.3 * uniDist(gen)) * (double)stairNum; // 폭 [140,200]
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
    }
}

#endif //_RAISIM_GYM_TORCH_HEIGHTMAP_HPP
