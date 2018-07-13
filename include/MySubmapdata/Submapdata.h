//
// Created by jacky on 18-7-11.
//

#ifndef TESTLASER_SUBMAPDATA_H
#define TESTLASER_SUBMAPDATA_H

namespace MySubmapdata
{
namespace
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


struct Submapheaderinfo
{
//    uint32_t seq;
    timeval stamp;
    std::string frame_id;
};
typedef Submapheaderinfo SubmapHeaderInfo;

struct My_SubmapEntry
{
    int my_trajectory_id;
    int my_submap_index;
    int my_submap_version;
    Pose my_pose;
};
typedef My_SubmapEntry MySubmapEntry;

template <typename T>
using My_List = std::list<T>;

struct My_SubmapList
{
    SubmapHeaderInfo header;
    My_List<MySubmapEntry> submap;
};
typedef  My_SubmapList MySubmapList;


template <class ContainerAllocator>
struct SubmapTexture_
{
    typedef SubmapTexture_<ContainerAllocator> Type;

    SubmapTexture_()
        : cells(), width(0), height(0), resolution(0.0), slice_pose()
    {
    }
    SubmapTexture_(const ContainerAllocator &_alloc)
        : cells(_alloc), width(0), height(0), resolution(0.0), slice_pose(_alloc)
    {
        (void)_alloc;
    }

    typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> _cells_type;
    _cells_type cells;

    typedef int32_t _width_type;
    _width_type width;

    typedef int32_t _height_type;
    _height_type height;

    typedef double _resolution_type;
    _resolution_type resolution;

    typedef Pose_<ContainerAllocator> _slice_pose_type;
    _slice_pose_type slice_pose;

    typedef boost::shared_ptr<SubmapTexture_<ContainerAllocator>> Ptr;
    typedef boost::shared_ptr<SubmapTexture_<ContainerAllocator> const> ConstPtr;

}; // struct SubmapTexture_

typedef SubmapTexture_<std::allocator<void>> SubmapTexture;

typedef boost::shared_ptr<SubmapTexture> SubmapTexturePtr;
typedef boost::shared_ptr<SubmapTexture const> SubmapTextureConstPtr;

//typedef std::vector<SubmapTexture_<ContainerAllocator> , typename ContainerAllocator::template rebind<SubmapTexture_<ContainerAllocator> >::other >  _textures_type;


struct My_SubmapResponse
{
    bool status;
    std::string errstr;
    int my_response_version;
    std::vector<SubmapTexture> textures;
};
typedef My_SubmapResponse MySubmapResponse;

}
}
#endif //TESTLASER_SUBMAPDATA_H
