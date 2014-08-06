#ifndef MRPT_BRIDGE_POSE_H
#define MRPT_BRIDGE_POSE_H

#include <cstring> // size_t

namespace std{
    template <class T> class allocator;
}

namespace tf{
    class Transform;
    class Matrix3x3;
}

namespace geometry_msgs{
    template <class ContainerAllocator> struct Pose_;
    typedef Pose_<std::allocator<void> > Pose;
    template <class ContainerAllocator> struct PoseWithCovariance_;
    typedef PoseWithCovariance_<std::allocator<void> > PoseWithCovariance;
    template <class ContainerAllocator> struct Quaternion_;
    typedef Quaternion_<std::allocator<void> > Quaternion;
}

namespace mrpt{
namespace math{
    template<class T> class CQuaternion;
    template <typename T,size_t NROWS,size_t NCOLS> class CMatrixFixedNumeric;
    typedef CMatrixFixedNumeric<double,3,3> CMatrixDouble33;
}
namespace poses{
        class CPose2D;
        class CPose3D;
        class CPosePDFGaussian;
        class CPose3DPDFGaussian;
        typedef math::CQuaternion<double> CQuaternionDouble;
    }
}

namespace mrpt_bridge
{
    /** Convert: ROS's Matrix -> MRPT's Matrix */
    mrpt::math::CMatrixDouble33& convert( const tf::Matrix3x3& _src, mrpt::math::CMatrixDouble33& _des);

    /** Convert: MRPT's Matrix -> Matrix */
    tf::Matrix3x3& convert( const mrpt::math::CMatrixDouble33& _src, tf::Matrix3x3& _des);

    /** Convert: MRPT's CPose3D -> ROS's Transform */
    tf::Transform& convert( const mrpt::poses::CPose3D& _src, tf::Transform&  _des);
    /** Convert: ROS's CPose3D -> MRPT's Transform */
    mrpt::poses::CPose3D& convert( const tf::Transform& _src, mrpt::poses::CPose3D& _des);

    /** Convert: MRPT's CPose3D -> ROS's Pose */
    geometry_msgs::Pose&  convert( const mrpt::poses::CPose3D& _src, geometry_msgs::Pose& _des);

	/** Convert: MRPT's CPose2D (x,y,yaw) -> ROS's Pose */
    geometry_msgs::Pose& convert( const mrpt::poses::CPose2D& _src, geometry_msgs::Pose& _des);

	/** Convert: MRPT's CPose3DPDFGaussian -> ROS's PoseWithCovariance */
    geometry_msgs::PoseWithCovariance& convert( const mrpt::poses::CPose3DPDFGaussian& _src, geometry_msgs::PoseWithCovariance& _des);

    /** Convert: MRPT's CPose3DPDFGaussian -> ROS's Transform */
    tf::Transform& convert( const mrpt::poses::CPose3DPDFGaussian& _src, tf::Transform&);

	/** Convert: MRPT's CPosePDFGaussian (x,y,yaw) -> ROS's PoseWithCovariance */
    geometry_msgs::PoseWithCovariance& convert( const mrpt::poses::CPosePDFGaussian& _src, geometry_msgs::PoseWithCovariance& _des);

	/** Convert: MRPT's CQuaternionDouble -> ROS's Quaternion  */
    geometry_msgs::Quaternion& convert(  const mrpt::poses::CQuaternionDouble& _src, geometry_msgs::Quaternion& _des);

	/** Convert: ROS's Pose -> MRPT's CPose2D  */
    mrpt::poses::CPose2D& convert(const geometry_msgs::Pose& _src, mrpt::poses::CPose2D& _des);

	/** Convert: ROS's Pose -> MRPT's CPose3D  */
    mrpt::poses::CPose3D& convert( const geometry_msgs::Pose& _src, mrpt::poses::CPose3D& _des);

	/** Convert: ROS's PoseWithCovariance -> MRPT's CPose3DPDFGaussian */
    mrpt::poses::CPose3DPDFGaussian& convert( const geometry_msgs::PoseWithCovariance& _src, mrpt::poses::CPose3DPDFGaussian& _des);

	/** Convert: ROS's PoseWithCovariance -> MRPT's CPosePDFGaussian (x,y,yaw) */
    mrpt::poses::CPosePDFGaussian& convert( const geometry_msgs::PoseWithCovariance& _src, mrpt::poses::CPosePDFGaussian& _des);

	/** Convert: ROS's Quaternion -> MRPT's CQuaternionDouble  */
    mrpt::poses::CQuaternionDouble& convert( const geometry_msgs::Quaternion& _src, mrpt::poses::CQuaternionDouble& _des);

}

#endif /* MRPT_BRIDGE_POSE_H */
