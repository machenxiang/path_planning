#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
    private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;
    public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);
            
        Eigen::MatrixXd getQ(int n_seg,int p_num1d,const Eigen::VectorXd &Time);
        Eigen::MatrixXd getM(int n_seg,int p_num1d,const Eigen::VectorXd &Time);
        Eigen::MatrixXd getCt(int n_seg,int d_order);
        
        Eigen::VectorXd closedFormCalCoeff1D(const Eigen::VectorXd &WayPoints1D,
                                             const int n_seg,
                                             const int d_order,
                                             const Eigen::VectorXd &Time);
        
        int Factorial(int x);
};
        

#endif

