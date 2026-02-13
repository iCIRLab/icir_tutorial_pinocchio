
//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

static Eigen::Matrix3d skm(Eigen::Vector3d x)
  {
    Eigen::Matrix3d Skew_temp1(3, 3);
    Skew_temp1.setZero();
    Skew_temp1(0, 1) = -x(2);
    Skew_temp1(0, 2) = x(1);
    Skew_temp1(1, 0) = x(2);
    Skew_temp1(1, 2) = -x(0);
    Skew_temp1(2, 0) = -x(1);
    Skew_temp1(2, 1) = x(0);
    return Skew_temp1;
  }

void SE3ToVector(const pinocchio::SE3 & M, Eigen::VectorXd & vec)
  {
    assert(vec.size()==12);
    vec.head<3>() = M.translation();
    typedef Eigen::Matrix<double,9,1> Vector9;
    vec.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
  }  

void vectorToSE3(Eigen::VectorXd & vec, pinocchio::SE3 & M)
  {
    assert(vec.size()==12);
    M.translation( vec.head<3>() );
    typedef Eigen::Matrix<double,3,3> Matrix3;
    M.rotation( Eigen::Map<const Matrix3>(&vec(3), 3, 3) );
  }

void errorInSE3 (const pinocchio::SE3 & M,
                  const pinocchio::SE3 & Mdes,
                  pinocchio::Motion & error)
  {
    // error = pinocchio::log6(Mdes.inverse() * M);
    // pinocchio::SE3 M_err = Mdes.inverse() * M;
    pinocchio::SE3 M_err = M.inverse() * Mdes;
    error.linear() = M_err.translation();
    error.angular() = pinocchio::log3(M_err.rotation());
  }

static double cubic(double time,    ///< Current time
                      double time_0,  ///< Start time
                      double time_f,  ///< End time
                      double x_0,     ///< Start state
                      double x_f,     ///< End state
                      double x_dot_0, ///< Start state dot
                      double x_dot_f  ///< End state dot
                      )
  {
    double x_t;

    if (time < time_0)
    {
      x_t = x_0;
    }
    else if (time > time_f)
    {
      x_t = x_f;
    }
    else
    {
      double elapsed_time = time - time_0;
      double total_time = time_f - time_0;
      double total_time2 = total_time * total_time;  // pow(t,2)
      double total_time3 = total_time2 * total_time; // pow(t,3)
      double total_x = x_f - x_0;

      x_t = x_0 + x_dot_0 * elapsed_time
            + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time
            + (-2 * total_x / total_time3 +
               (x_dot_0 + x_dot_f) / total_time2) *
                  elapsed_time * elapsed_time * elapsed_time;      
    }

    return x_t;
  }

static Eigen::VectorXd SE3Cubic(double m_time,     //< Current time
                      double m_stime,              //< Start time
                      double m_duration,           //< duration
                      pinocchio::SE3 m_init,       //< Start state
                      pinocchio::SE3 m_goal        //< End state                      
                      )
  {
    Eigen::VectorXd pos_sample(12);
    pos_sample.setZero();

    pinocchio::SE3 m_cubic;
    Eigen::Vector3d cubic_tra;

    Eigen::Matrix3d rot_diff = (m_init.rotation().transpose() * m_goal.rotation()).log();

    typedef Eigen::Matrix<double, 9, 1> Vector9;
    if (m_time < m_stime) {
      pos_sample.head<3>() = m_init.translation();
      pos_sample.tail<9>() = Eigen::Map<const Vector9>(&m_init.rotation()(0), 9);

      return pos_sample;
    }
    else if (m_time > m_stime + m_duration) {
      pos_sample.head<3>() = m_goal.translation();
      pos_sample.tail<9>() = Eigen::Map<const Vector9>(&m_goal.rotation()(0), 9);

      return pos_sample;
    }
    else {
        double a0, a1, a2, a3;
        double tau = cubic(m_time, m_stime, m_stime + m_duration, 0, 1, 0, 0);

        for (int i = 0; i < 3; i++) {
          a0 = m_init.translation()(i);
          a1 = 0.0; //m_init.vel(i);
          a2 = 3.0 / pow(m_duration, 2) * (m_goal.translation()(i) - m_init.translation()(i));
          a3 = -1.0 * 2.0 / pow(m_duration, 3) * (m_goal.translation()(i) - m_init.translation()(i));

          cubic_tra(i) = a0 + a1 * (m_time - m_stime) + a2 * pow(m_time - m_stime, 2) + a3 * pow(m_time - m_stime, 3);
        }

        m_cubic.rotation() = m_init.rotation() * (rot_diff * tau).exp();
        m_cubic.translation() = cubic_tra;
      
        pos_sample.head<3>() = m_cubic.translation();
        pos_sample.tail<9>() = Eigen::Map<const Vector9>(&m_cubic.rotation()(0), 9);      

        return pos_sample;
    }
  }