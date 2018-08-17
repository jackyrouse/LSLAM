//
// Created by jacky on 18-8-16.
//

#ifndef LSLAM_MARKERARRAY_H
#define LSLAM_MARKERARRAY_H


namespace MyMarkerArray
{
//Point
template <class ContainerAllocator>
struct Point_
{
    typedef Point_<ContainerAllocator> Type;

    Point_()
        : x(0.0)
        , y(0.0)
        , z(0.0)  {
    }
    Point_(const ContainerAllocator& _alloc)
        : x(0.0)
        , y(0.0)
        , z(0.0)  {
        (void)_alloc;
    }



    typedef double _x_type;
    _x_type x;

    typedef double _y_type;
    _y_type y;

    typedef double _z_type;
    _z_type z;


    typedef boost::shared_ptr<Point_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr<Point_<ContainerAllocator> const> ConstPtr;

}; // struct Point_

typedef Point_<std::allocator<void> > Point;

typedef boost::shared_ptr<Point > PointPtr;
typedef boost::shared_ptr<Point const> PointConstPtr;
//end of point-----

//Quaternion
template <class ContainerAllocator>
struct Quaternion_
{
    typedef Quaternion_<ContainerAllocator> Type;

    Quaternion_()
        : x(0.0)
        , y(0.0)
        , z(0.0)
        , w(0.0)  {
    }
    Quaternion_(const ContainerAllocator& _alloc)
        : x(0.0)
        , y(0.0)
        , z(0.0)
        , w(0.0)  {
        (void)_alloc;
    }

    typedef double _x_type;
    _x_type x;

    typedef double _y_type;
    _y_type y;

    typedef double _z_type;
    _z_type z;

    typedef double _w_type;
    _w_type w;

    typedef boost::shared_ptr<Quaternion_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr<Quaternion_<ContainerAllocator> const> ConstPtr;

}; // struct Quaternion_

typedef Quaternion_<std::allocator<void> > Quaternion;

typedef boost::shared_ptr<Quaternion > QuaternionPtr;
typedef boost::shared_ptr<Quaternion const> QuaternionConstPtr;
//end of Quaternion----------

//Pose
template <class ContainerAllocator>
struct Pose_
{
    typedef Pose_<ContainerAllocator> Type;

    Pose_()
        : position()
        , orientation()  {
    }
    Pose_(const ContainerAllocator& _alloc)
        : position(_alloc)
        , orientation(_alloc)  {
        (void)_alloc;
    }



    typedef Point_<ContainerAllocator>  _position_type;
    _position_type position;

    typedef Quaternion_<ContainerAllocator>  _orientation_type;
    _orientation_type orientation;

    typedef boost::shared_ptr<Pose_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr<Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr<Pose > PosePtr;
typedef boost::shared_ptr<Pose const> PoseConstPtr;
//end of pose ----

struct Markerheaderinfo
{
//    uint32_t seq;
    timeval stamp;
    std::string frame_id;
};
typedef Markerheaderinfo MarkerHeaderInfo;

template <typename T>
using My_List = std::list<T>;

template <typename T>
using My_Vector = std::vector<T>;

struct Marker
{
    MarkerHeaderInfo header;
    std::string ns;
    int id;
    int type;
    int action;
    Pose pose;
    My_List<Point> points;
};
typedef Marker Marker;

struct MarkerArray
{
    My_Vector<Marker> markers;
};
typedef MarkerArray MarkerArray;

}
#endif //LSLAM_MARKERARRAY_H
