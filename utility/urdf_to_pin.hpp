#ifndef __robots_robot_wrapper_hpp__
#define __robots_robot_wrapper_hpp__

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>

#include <string>
#include <vector>


class RobotWrapper{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef double Scalar;
        typedef pinocchio::Model Model;
        typedef pinocchio::Data Data;
        typedef pinocchio::Motion Motion;
        typedef pinocchio::Frame Frame;
        typedef pinocchio::SE3 SE3;
        typedef Eigen::VectorXd  Vector;
        typedef Eigen::Matrix<Scalar,3,1>  Vector3;
        typedef Eigen::Matrix<Scalar,6,1> Vector6;
        typedef Eigen::MatrixXd Matrix;
        typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic> Matrix3x;
        typedef Eigen::Matrix<Scalar,6,6> Matrix6d;
        typedef Eigen::Ref<Vector> RefVector;
        typedef const Eigen::Ref<const Vector> ConstRefVector;
        
        RobotWrapper(const std::string & filename, const std::vector<std::string> & package_dirs, bool verbose=false);
        ~RobotWrapper(){};

        virtual int nq() const;
        virtual int nv() const;
        virtual int na() const;

        const Model & model() const;
        Model & model();

        void computeAllTerms(Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v);
        const SE3 & position(const Data & data, const Model::JointIndex index) const;
        const Motion & velocity(const Data & data, const Model::JointIndex index) const;
        void jacobianWorld(const Data & data, const Model::JointIndex index, Data::Matrix6x & J);
        SE3 framePosition(const Data & data, const Model::FrameIndex index) const;
        void framePosition(const Data & data, const Model::FrameIndex index, SE3 & framePosition) const;
        Motion frameVelocity(const Data & data, const Model::FrameIndex index) const;
        void frameVelocity(const Data & data, const Model::FrameIndex index, Motion & frameVelocity) const;
        Motion frameAcceleration(const Data & data, const Model::FrameIndex index) const;
        void frameAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;
        Motion frameClassicAcceleration(const Data & data, const Model::FrameIndex index) const;
        void frameClassicAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;
        void frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J) ;

    protected:
        Model m_model;
        std::string m_model_filename;
        bool m_verbose;
        int m_na;
        Eigen::VectorXd m_q;
};

#endif
        