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
//                    heightVec[j*xSampleNum + i] = (0.10 * roughness) * uniDist(gen) + 0.2;
                    heightVec[j*xSampleNum + i] = (0.12 * roughness) * uniDist(gen) + 0.2;
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
//                    squareHeight(i,j) = 0.2 + (0.08 * roughness) * uniDist(gen);
                    squareHeight(i,j) = 0.2 + (0.12 * roughness) * uniDist(gen);
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
        double heightMax = 0.13 * hardness * 20;
        int xSampleNum = 50;
        int ySampleNum = 50;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        for (int i=0; i<xSampleNum; i++){
            for (int j=0; j<ySampleNum; j++){
                heightVec[j*xSampleNum + i] = j * (heightMax/(double)ySampleNum) + (0.02 * roughness) * uniDist(gen) + 0.2;
            }
        }

        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, 20.0, 0., 0., heightVec);
    }


    else if (heightMapType == 2){
        /// stair
        double stepHeight = 0.065 * hardness;
        int xSampleNum = 60;
        int ySampleNum = 1200; //3600;
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

        double ySize = (0.30 + 0.10 * uniDist(gen)) * (double)stairNum; // 폭 [20,40]
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
    }


    else if(heightMapType == 3){
        /// big stair
        double stepHeight = 0.115 * hardness;
        int xSampleNum = 60;
        int ySampleNum = 1200;
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

        double ySize = (1.35 + 0.15 * uniDist(gen)) * (double)stairNum; // 폭 [120,150]
//        double ySize = (1.2 ) * (double)stairNum; // 폭 [140,200]
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
    }

    else if(heightMapType == 4){
        /// mixed
        int xSampleNum = 50;
        int ySampleNum = 50;
        std::vector<double> heightVec;
        heightVec.resize(25*xSampleNum*ySampleNum);
        for (int x_cnt = 0; x_cnt < 5; x_cnt++){
            for (int y_cnt = 0; y_cnt < 5; y_cnt++){
                if ((x_cnt+y_cnt)%5 == 0) {
                /// square
                    Eigen::Matrix<double,10,10> squareHeight;
                    for (int i=0;i<10;i++){
                        for (int j=0;j<10;j++){
                            squareHeight(i,j) = 0.2 + 0.12 * uniDist(gen);
                        }
                    }
                    for (int i=0; i<xSampleNum; i++){
                        for (int j=0; j<ySampleNum; j++){
                            heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum + i + x_cnt*xSampleNum] = squareHeight((i/8)%10,(j/8)%10);
                        }
                    }
                }
                else if ((x_cnt+y_cnt)%5 == 1) {
                /// rough terrain
                    double roughness = hardness/15.0;
                    for (int i=0; i<xSampleNum/2; i++){
                        for (int j=0; j<ySampleNum/2; j++){
                            double temp_height =  (0.24 * roughness) * uniDist(gen) + 0.2;
                            heightVec[(2*j + y_cnt*ySampleNum)*5*xSampleNum + 2*i + x_cnt*xSampleNum] = temp_height;
                            heightVec[(2*j + 1 + y_cnt*ySampleNum)*5*xSampleNum + 2*i + x_cnt*xSampleNum] = temp_height;
                            heightVec[(2*j + y_cnt*ySampleNum)*5*xSampleNum + 2*i + 1 + x_cnt*xSampleNum] = temp_height;
                            heightVec[(2*j + 1  + y_cnt*ySampleNum)*5*xSampleNum + 2*i + 1 + x_cnt*xSampleNum] = temp_height;
                        }
                    }
                }
                else if ((x_cnt+y_cnt)%5 == 2) {
                /// stair
                    double stepHeight = 0.042 * hardness;
                    int stairNum = 10;
                    int step = 0;
                    for (int j=0; j<ySampleNum; j++){
                        if (j < ySampleNum/2) {
                            heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum] = ceil((double)step/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 50 계단
                            for (int i=1; i<xSampleNum; i++){
                                heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum + i + x_cnt*xSampleNum] = heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum];
                            }
                        }
                        else {
                            heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum] = ceil((double)(ySampleNum - step)/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 50 계단
                            for (int i=1; i<xSampleNum; i++){
                                heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum + i + x_cnt*xSampleNum] = heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum];
                            }
                        }
                        step++;
                    }
                }
                else if ((x_cnt+y_cnt)%5 == 3) {
                /// slope
                    double roughness = abs(uniDist(gen));
                    double heightMax = 0.213 * hardness;
                    for (int i=0; i<xSampleNum; i++){
                        for (int j=0; j<ySampleNum; j++){
                            if (j < ySampleNum/2) {
                                heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum + i + x_cnt*xSampleNum] = j * (heightMax/(double)(ySampleNum/2)) + 0.2;
                            }
                            else {
                                heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum + i + x_cnt*xSampleNum] = (ySampleNum - j) * (heightMax/(double)(ySampleNum/2)) + 0.2;
                            }
                        }
                    }
                }
                else if ((x_cnt+y_cnt)%5 == 4) {
                /// big stairs
                    double stepHeight = 0.07 * hardness;
                    int stairNum = 3;
                    int step = 0;
                    for (int j=0; j<ySampleNum; j++){
                        if (j < ySampleNum/2) {
                            heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum] = ceil((double)step/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 50 계단
                            for (int i=1; i<xSampleNum; i++){
                                heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum + i + x_cnt*xSampleNum] = heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum];
                            }
                        }
                        else {
                            heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum] = ceil((double)(ySampleNum - step)/(double)(ySampleNum/stairNum)) * stepHeight + 0.2; // 50 계단
                            for (int i=1; i<xSampleNum; i++){
                                heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum + i + x_cnt*xSampleNum] = heightVec[(j + y_cnt*ySampleNum)*5*xSampleNum];
                            }
                        }
                        step++;
                    }
                }
            }
        }
        return world->addHeightMap(5*xSampleNum, 5*ySampleNum, 20.0, 20.0, 0., 0., heightVec);
    }

    else if (heightMapType == 5) {
        /// big stair 2
        double stepHeight = 0.115 * hardness;
        int xSampleNum = 6;
        int ySampleNum = 3600;
        int stairNum = 12;
        std::vector<double> heightVec;
        heightVec.resize(xSampleNum*ySampleNum);

        int step = 0;
        for (int j=0; j<ySampleNum; j++){
            heightVec[j*xSampleNum] = (int((double)step/(double)(ySampleNum/stairNum))%2) * stepHeight + 0.2; // 12 계단
            for (int i=1; i<xSampleNum; i++){
                heightVec[j*xSampleNum+i] = heightVec[j*xSampleNum];
            }
            step++;
        }

        double ySize = (2.0) * (double)stairNum; // 폭 [120,150]
        return world->addHeightMap(xSampleNum, ySampleNum, 20.0, ySize, 0., 0., heightVec);
    }
            return nullptr;
}

#endif //_RAISIM_GYM_TORCH_HEIGHTMAP_HPP
