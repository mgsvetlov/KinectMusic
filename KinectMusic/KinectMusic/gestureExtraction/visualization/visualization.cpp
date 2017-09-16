//
//  visualization.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include "visualization.h"
#include "../processframedata.h"

Visualization* Visualization::p_vis = nullptr;
cv::Mat Visualization::matImage;
bool Visualization::isNeedRedraw = false;

Visualization::Visualization() {
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::moveWindow("Display window", 20, -1000);
    //cv::resizeWindow("Display window", 640, 480);
    p_vis = this;
}

bool Visualization::showImage() {
    pthread_mutex_lock(&ProcessFrameData::visualisation_mutex);
    if(!getIsNeedRedraw()){
        pthread_mutex_unlock(&ProcessFrameData::visualisation_mutex);
        usleep(10);
        return true;
    }
    isNeedRedraw = false;
    if(p_vis == nullptr){
        Visualization();
    }
    //cv::Mat matImageRes;
    //cv::resize(matImage, matImageRes, cv::Size(matImage.cols << 1, matImage.rows << 1));
    cv::imshow( "Display window", matImage);
    pthread_mutex_unlock(&ProcessFrameData::visualisation_mutex);
    
    if(cv::waitKey(1) == 27) {
        return false;
    }
    return true;
}


void Visualization::mat2img(cv::Mat mat, cv::Mat& matImg) {
    
    int w = mat.cols;
    int h = mat.rows;
    cv::Mat r  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    cv::Mat g  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    cv::Mat b  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    uint16_t* p_mat16 = (uint16_t*)(mat.data);
    unsigned char* p_r = (unsigned char*)(r.data);
    unsigned char* p_g = (unsigned char*)(g.data);
    unsigned char* p_b = (unsigned char*)(b.data);
    for(int i = 0; i < w*h; i++) {
        uint16_t d16 = *p_mat16;
        
        if(d16 && d16 < Params::getMaxKinectValue()) {
            *p_b = *p_g =*p_r = 255 - d16 * 255. / Params::getMaxKinectValue();
            //*p_b = 0;
        }
        /*else {
         *p_r = 0;
         *p_b = 255;
         }*/
        p_r++, p_g++, p_b++, p_mat16++;
    }
    
    std::vector<cv::Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);
    
    cv::merge(channels, matImg);
}

void Visualization::keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size) {
    int x = keyPoint.x;
    int y = keyPoint.y;
    if(x < 0 || y < 0)
        return;
    cv::circle(matImg, cv::Point(x, y), size,  color, -1);
}

void Visualization::drawText(cv::Mat& mat, std::string text, double fontScale, int thickness, cv::Scalar color, cv::Point2f textCenter)
{
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace,
                                        fontScale, thickness, &baseline);
    baseline += thickness;
    
    // center the text
    cv::Point textOrg((mat.cols - textSize.width) * textCenter.x,
                      (mat.rows + textSize.height) * textCenter.y);
    
    cv::putText(mat, text, textOrg, fontFace, fontScale,
                color, thickness, 8);
}

void Visualization::blobs2img(const std::list<BlobFinal>& lBlobs, cv::Mat& matImg, bool drawKeyPoints ){
    if(lBlobs.empty())
        return;
    
    //int i(0);
    for(const auto& blob : lBlobs){
        //int rem = i % 3;
        //cv::Scalar color = rem == 0 ? cv::Scalar(0,1.0,0) : rem == 1 ?cv::Scalar(1.0,0,0) : cv::Scalar(1.0,1.0,0) ;
        cv::Scalar color (1,1,1);
        blob2img(blob, matImg, color, true);
        if(drawKeyPoints) {
            int x = blob.cells.MinValCell()->x;
            int y = blob.cells.MinValCell()->y;
            int z = blob.cells.MinValCell()->val;
            keyPoint2img(cv::Point3i(x, y, z), matImg, cv::Scalar(127, 0, 0), 3);
        }
        //++i;
    }
    
}

void Visualization::blob2img(const BlobFinal& blob, cv::Mat& matImg, const cv::Scalar& color, bool colorFromNormal){
    /*int minVal = blob.cells.MinValCell()->val;
    for(auto& cell : blob.cells.AllConst()){
        int col = 255 - (cell.val - minVal);
        if(col < 0)
            col = 0;
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (color[0] * col, color[1] * col, color[2] * col), -1);
    }*/
    //const auto& contour = blob.borderPtr->contour;
    /*for(const auto& cell : contour){
        cv::Scalar c = (cell.flags & FLAGS::ADJACENT_BODY) ? cv::Scalar (255.0f, 0.0f, 0) : cv::Scalar (0.0f, 0.0f, 255);
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, c, -1);
    }*/
    /*auto fingerInds = blob.borderPtr->fingerInds;
    for(auto ind : fingerInds){
        auto& cell = contour[ind];
        cv::Scalar c = (cell.flags & FLAGS::ADJACENT_BODY) ? cv::Scalar (255.0f, 0.0f, 0) : cv::Scalar (0.0f, 0.0f, 255);
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, c, -1);
    }*/
    
    int fingerCount(0);
    for(const auto& blob : blob.blobsFingers){
        int rem = fingerCount % 6;
        cv::Scalar color = rem == 0 ? cv::Scalar (255, 0, 0) :
        rem == 1 ? cv::Scalar (0, 255, 0) :
        rem == 2 ? cv::Scalar (0, 0, 255) :
        rem == 3 ? cv::Scalar (255, 255, 0) :
        rem == 4 ? cv::Scalar (255, 0, 255) :
        cv::Scalar (33, 67, 255);
        bool isFirstCell(true);
        for(const auto& cell : blob.cells.AllConst()){
            int size = 1;//isFirstCell? 3 : 1;
            cv::circle(matImg, cv::Point(cell.x, cell.y), size, color, -1);
            isFirstCell = false;
        }
        ++fingerCount;
    }
    
    for(const auto& p : blob.pointsCHull){
        cv::Scalar color = cv::Scalar (0, 0, 255);
        cv::circle(matImg, cv::Point(p.x, p.y), 1, color, -1);
    }
    
    /*for(auto& poly : blob.meshInds){
        //Set Vector U to (Triangle.p2 minus Triangle.p1)
        //Set Vector V to (Triangle.p3 minus Triangle.p1)
        
        //Set Normal.x to (multiply U.y by V.z) minus (multiply U.z by V.y)
        //Set Normal.y to (multiply U.z by V.x) minus (multiply U.x by V.z)
        //Set Normal.z to (multiply U.x by V.y) minus (multiply U.y by V.x)
        cv::Vec3i U = blob.pointsCHull[poly[1]] - blob.pointsCHull[poly[0]];
        cv::Vec3i V = blob.pointsCHull[poly[2]] - blob.pointsCHull[poly[0]];
        int normal_z = U[0] * V[1] - U[1] * V[0];
        //if(normal_z > 0)
            //continue;
        cv::Scalar color = (normal_z < 0) ? cv::Scalar (255, 127, 64) : cv::Scalar (64, 127, 255);
        for(int i = 0; i < poly.size(); ++i){
            auto& p1 = blob.pointsCHull[poly[i]];
            auto& p2 = blob.pointsCHull[poly[(i+1) % poly.size()]];
            cv::line(matImg, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), color);
        }
    }*/
    
    for(const auto& p : blob.pointsFingers){
        cv::Scalar color = cv::Scalar (0, 0, 255 - (p.z -1) * 16);
        cv::circle(matImg, cv::Point(p.x, p.y), 1, color, -1);
    }

    if(blob.angles3dPtr){
        const auto& anglesData = blob.angles3dPtr->getDataConst();
        for(auto& d : anglesData){
            auto& p = std::get<1>(d);
            cv::circle(matImg, cv::Point(p.x, p.y), 5, cv::Scalar (0.0f, 127, 196), -1);
            auto& pl = std::get<0>(d);
            cv::Point p1 (p.x /*+ pl.x * 40*/, p.y + (1.0 - std::abs(pl.z)) * 40.);
            cv::circle(matImg, cv::Point(p1.x, p1.y), 5, cv::Scalar (255, 0, 255), -1);
            cv::line(matImg, cv::Point(p.x, p.y), p1, cv::Scalar (255, 127, 64), 3);
        }
    }
    /*auto& borderClusts = blob.borderPtr->borderClusts;
    for(auto& row : borderClusts){
        for(auto& clust : row.clusts){
            int y = clust.y;
            for(auto x = clust.first; x <= clust.last; ++ x)
                cv::circle(matImg, cv::Point(x, y), 1, cv::Scalar (0.0f, 0.0f, 255), -1);
        }
    }*/
    /*for(auto& cell : blob.borderCells1.AllConst()){
        auto& parentCell = blob.cells.AllConst()[cell.parentInd];
        cv::circle(matImg, cv::Point(parentCell.x, parentCell.y), 1, cv::Scalar (0.0f, 255.0f, 0), -1);
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (0.0f, 255.0f, 255), -1);
    }
    for(auto& cell : blob.borderCells2.AllConst()){
        auto& parentCell = blob.cells.AllConst()[cell.parentInd];
        cv::circle(matImg, cv::Point(parentCell.x, parentCell.y), 1, cv::Scalar (255, 255.0f, 0), -1);
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (255, 0.0f, 0), -1);
    }*/
}

void Visualization::tracks2img(const std::vector<Track>& tracks, cv::Mat& matImg){
    for(int i = 0; i < tracks.size(); ++i){
        cv::Scalar color = i == 0 ? cv::Scalar (255.0f, 0.0f, 0) : cv::Scalar (0.0f, 255.0f, 0);
        const auto& track = tracks[i];
        if(track.handHistory.empty())
            return;
        auto& keyPoint = track.handHistory.back().keyPoint;
        cv::circle(matImg, cv::Point(keyPoint.x, keyPoint.y), 5, color, -1);
    }
}

void Visualization::vecs2img(cv::Mat& matVec, cv::Mat& matImg){
    static const int resizePow = log2(matImg.cols/matVec.cols);
    cv::Vec2f* p_matVec = (cv::Vec2f*)(matVec.data);
    int ind(0);
    for( ; p_matVec < (cv::Vec2f*)(matVec.data) + matVec.total(); ++p_matVec, ++ind){
        auto x = ind % matVec.cols;
        auto y = (ind - x) / matVec.cols;
        x <<= resizePow;
        y <<= resizePow;
        const cv::Vec2f& vec = *p_matVec;
        double length = cv::norm(vec);
        unsigned char col = length > 255 ? 255 : 0;//static_cast<unsigned char>(length);
        if(!col)
            continue;
        unsigned char col1 = vec[0] + vec[1] > 0? col : 0;
        unsigned char col2 = vec[0] + vec[1] < 0? col : 0;
        //auto ind = y * matImg.cols + x;
        //*((unsigned char*)(matImg.data) + ind * 3) = col1;
        //*((unsigned char*)(matImg.data) + ind * 3 + 1) = col2;
        //*((unsigned char*)(matImg.data) + ind * 3 + 2) = 0;
        cv::circle(matImg, cv::Point(x, y), 3, cv::Scalar(col1,col2,0), -1);
    }
    
}
/*void Visualization::gesture2img(const std::shared_ptr<Gesture>& gesture, cv::Mat& matImg, size_t length){
 int pointSize(5);
 cv::Scalar color = gesture->handInd == 0 ? cv::Scalar(0,255,255) : cv::Scalar(255,255, 0);
 auto rit = gesture->handsData.crbegin();
 if(length == 0 || gesture->handsData.size() < length)
 length = gesture->handsData.size();
 for(int i = 0; i < length; ++i, ++rit){
 const auto& handData = *rit;
 const auto& point = handData.point;
 if(point.x != NO_DATA_VALUE && handData.phase != NO_DATA_VALUE){
 keyPoint2img(point, matImg, color, pointSize);
 }
 }
 }*/

/*void Visualization::gestures2img(const std::vector<std::shared_ptr<Gesture>>& gestures, cv::Mat& matImg, size_t length){
 for(auto& gesture : gestures) {
 gesture2img(gesture, matImg, length);
 }
 }*/
