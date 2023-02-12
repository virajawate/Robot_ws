#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/costmap_model.h>
#include <tf/transform_datatypes.h>

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>


namespace ob = ompl::base;
namespace oc = ompl::control;

namespace rrt_planner{

    class RRTPlanner : public nav_core::BaseGlobalPlanner{

        public:
            RRTPlanner();

            virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

            ~RRTPlanner(){}

            void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result);

            bool isStateValid(const oc::SpaceInformation *si, const ob::State *state);

            double calc_cost(const ob::State *state);

            double motion_cost(const ob::State *s1, const ob::State* s2);

            void get_xy_theta_v(const ob::State *s, double& x, double& y, double& theta, double& velocity);
            
            void set_xy_theta_v(const ob::State *s, double& x, double& y, double& theta, double& velocity);

        private:
            costmap_2d::Costmap2DROS* _costmap_ros;
            std::string _frame_id;
            ros::Publisher _plan_pub;
            bool _initialized;
            bool _allow_unknown;

            std::string tf_prefix_;
            boost::mutex _mutex;
            base_local_planner::CostmapModel* _costmap_model;

            ob::StateSpacePtr _se2_space;
            ob::StateSpacePtr _velocity_space;
            ob::StateSpacePtr _space;
    };

    class CostMapObjective : public ob::StateCostIntegralObjective{

        public:
            CostMapObjective(RRTPlanner& op, const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true), _rrt_planner(op){}

            virtual ob::Cost stateCost(const ob::State* s) const{
                return ob::Cost(_rrt_planner.calc_cost(s));
            }

        private:
            RRTPlanner& _rrt_planner;
    };

    class CostMapWorkObjective : public ob::MechanicalWorkOptimizationObjective{

        public:
            CostMapWorkObjective(RRTPlanner& op, const ob::SpaceInformationPtr& si) : ob::MechanicalWorkOptimizationObjective(si), _rrt_planner(op){}
            ob::Cost stateCost(const ob::State* s) const{
                return ob::Cost(_rrt_planner.calc_cost(s));
            }

        private:
            RRTPlanner& _rrt_planner;
    };
}
#endif