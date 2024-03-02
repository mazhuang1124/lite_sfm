
#ifndef SFM_MAP_H
#define SFM_MAP_H

#include "MapPoint.h"
#include "Frame.h"


class Map {
public:
    typedef shared_ptr<Map> Ptr;
    list<MapPoint::Ptr> mapPoints;        // all landmarks
    list<Frame::Ptr> frames;         // all key-frames

    Map() {}

    void addFrame(Frame::Ptr frame);

    void addMapPoint(MapPoint::Ptr mapPoint);

    void savePoints();

    void visInCloudViewer();
};

#endif 

