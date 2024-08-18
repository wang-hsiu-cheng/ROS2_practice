#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/JointState.hpp"
#include "aip_test/upper_computer.h"

TurtleController::TurtleController() : Node("turtle_controller")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(500ms, std::bind(&TurtleController::move_turtle, this));
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(500),
    //     std::bind(&TurtleController::move_turtle, this)
    // );
}
void TurtleController::pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "callback: speed = %f, angular = %f, theta = %f",
    //     msg->linear_velocity, msg->angular_velocity, msg->theta);
    rVx = msg->linear_velocity * cos(msg->theta);
    rVy = msg->linear_velocity * sin(msg->theta);
    rW = msg->angular_velocity;
}
void TurtleController::initialize_path()
{
    // setup path
    path_.push_back(Eigen::Vector3d(4.0, 0.0, 0));
    path_.push_back(Eigen::Vector3d(4.0, 4.0, 0));
    // path_.push_back(Eigen::Vector3d(0.0, 4.0, 0));
    // path_.push_back(Eigen::Vector3d(0.0, 4.0, 3.1415926536 / 2));
    // path_.push_back(Eigen::Vector3d(0.0, 0.0, 3.1415926536 / 2));

	obsticals.push_back(Eigen::Vector3d(2.0, 0.0, 0.2));
    obsticals.push_back(Eigen::Vector3d(4.0, 2.0, 0.2));
    // obsticals.push_back(Eigen::Vector3d(0.0, 2.0, 0.2));
    // obsticals.push_back(Eigen::Vector3d(2.0, 2.0, 0.2));
}

void TurtleController::move_turtle()
{
    initialize_path();

    while (rclcpp::ok())
    {
        // Example logic to move turtle towards the first point in the path
        if (!path_.empty()) {
            // RCLCPP_INFO(this->get_logger(), "point: x=%f, y=%f, theta=%f",
            //     path_.front().x(), path_.front().y(), path_.front().z());
            moveTo(path_.front().x(), path_.front().y(), path_.front().z());
            path_.erase(path_.begin());  // Remove the first point after moving towards it
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);// 初始化ROS
    // rclcpp::spin(std::make_shared<TurtleController>());
    auto node = std::make_shared<TurtleController>();
    node->move_turtle();
    // 關閉ROS
    rclcpp::shutdown();

    return 0;
}
void TurtleController::updateUnitVector(double moved)
{
    if (moved < d1)
    {
        x_vec = x_vec;
        y_vec = y_vec;
    }
    else if (moved >= d1 && goalDistance - moved > d2)
    {
        double angVel = VelocityNow / radius;
        double temp;
        // rotation matrix to update unit vector velocity
        if (obsOnRoad[0].second)
        {
            temp = cos(angVel * deltaTime) * x_vec + sin(angVel * deltaTime) * y_vec;
            y_vec = -sin(angVel * deltaTime) * x_vec + cos(angVel * deltaTime) * y_vec;
        }
        else
        {
            temp = cos(angVel * deltaTime) * x_vec - sin(angVel * deltaTime) * y_vec;
            y_vec = sin(angVel * deltaTime) * x_vec + cos(angVel * deltaTime) * y_vec;
        }
        x_vec = temp;
    }
    else
    {
        x_vec = x_vec;
        y_vec = y_vec;
    }
}
void TurtleController::planNewPath(std::vector<pair<int, bool>> obsOnRoad, double xGoal, double yGoal)
{
    double theta, D1, D2;
    pair<double, double> p1, p2;
    radius = R + obsticals[obsOnRoad[0].first].z();
    D1 = hypot((obsticals[obsOnRoad[0].first].x() - botPositionX), (obsticals[obsOnRoad[0].first].y() - botPositionY)); // distance between start point and obstical
    d1 = sqrt(pow(D1, 2) - pow(radius, 2));                                                                         // first straight line path
    D2 = hypot((obsticals[obsOnRoad[0].first].x() - xGoal), (obsticals[obsOnRoad[0].first].y() - yGoal));               // distance between obstical and end point
    d2 = sqrt(pow(D2, 2) - pow(radius, 2));                                                                         // second straight line path

    if (obsOnRoad[0].second) // use rotation matrix to turn clockwise
    {
        x_vec = ((obsticals[obsOnRoad[0].first].x() - botPositionX) * d1 / D1 + (obsticals[obsOnRoad[0].first].y() - botPositionY) * -radius / D1) / D1;
        y_vec = ((obsticals[obsOnRoad[0].first].x() - botPositionX) * radius / D1 + (obsticals[obsOnRoad[0].first].y() - botPositionY) * d1 / D1) / D1;
    }
    else // use rotation matrix to turn counterclockwise
    {
        x_vec = ((obsticals[obsOnRoad[0].first].x() - botPositionX) * d1 / D1 + (obsticals[obsOnRoad[0].first].y() - botPositionY) * radius / D1) / D1;
        y_vec = ((obsticals[obsOnRoad[0].first].x() - botPositionX) * -radius / D1 + (obsticals[obsOnRoad[0].first].y() - botPositionY) * d1 / D1) / D1;
    }
    p1.first = x_vec * d1 + botPositionX;
    p1.second = y_vec * d1 + botPositionY;
    p2.first = ((obsticals[obsOnRoad[0].first].x() - xGoal) * d2 / D2 + (obsticals[obsOnRoad[0].first].y() - yGoal) * radius / D2) / D2 * d2 + xGoal;
    p2.second = ((obsticals[obsOnRoad[0].first].x() - xGoal) * -radius / D1 + (obsticals[obsOnRoad[0].first].y() - yGoal) * d2 / D2) / D2 * d2 + yGoal;
    cout << p2.first << " " << p2.second << endl;
    // calculate the angle of the curve to avoid the obstical
    theta = acos(((p1.first - obsticals[obsOnRoad[0].first].x()) * (p2.first - obsticals[obsOnRoad[0].first].x()) + (p1.second - obsticals[obsOnRoad[0].first].y()) * (p2.second - obsticals[obsOnRoad[0].first].y())) / pow(radius, 2));
    theta = (theta > PI) ? 2 * PI - theta : theta;
    goalDistance = d1 + d2 + radius * theta; // total path length to avoid the obstical between two goal points
    cout << "theta: " << theta << endl;
    cout << "goalDistance: " << goalDistance << endl;
}

void TurtleController::pointToDist(double xGoal, double yGoal, double wGoal)
{
    cout << "(botPositionX, botPositionY): " << "(" << botPositionX << ", " << botPositionY << ")\n";
    goalDistance = hypot((xGoal - botPositionX), (yGoal - botPositionY));
    x_vec = (xGoal - botPositionX) / goalDistance;
    y_vec = (yGoal - botPositionY) / goalDistance;
    Goal_w = wGoal;

    obsOnRoad.clear();
    Eigen::Vector3d pt;
    cout << "obsticals.size: " << (int)obsticals.size() << endl;
    for (int i = 0; i < (int)obsticals.size(); i++)
    {
        std::vector<Eigen::Vector3d> pts;
        // record three points of the obstical(top, bottom, center)
        pts.push_back(pt = {(obsticals[i].x() + y_vec * obsticals[i].z()), (obsticals[i].y() - x_vec * obsticals[i].z()), 0.0});
        pts.push_back(pt = {(obsticals[i].x() - y_vec * obsticals[i].z()), (obsticals[i].y() + x_vec * obsticals[i].z()), 0.0});
        pts.push_back(pt = {obsticals[i].x(), obsticals[i].y(), 0.0});
        for (int j = 0; j < 3; j++)
        {
            float x = pts[j].x();
            float y = pts[j].y();
            // use the four formula to check if the points are in the road area
            if (y_vec * y_vec / x_vec * (x - botPositionX) - (y - botPositionY) - R * hypot(x_vec, y_vec) / abs(x_vec) < 0 &&
                y_vec * y_vec / x_vec * (x - botPositionX) - (y - botPositionY) + R * hypot(x_vec, y_vec) / abs(x_vec) > 0 &&
                x_vec * x_vec / -y_vec * (x - botPositionX) - (y - botPositionY) < 0 &&
                x_vec * x_vec / -y_vec * (x - xGoal) - (y - yGoal) > 0)
            {
                pair<int, bool> temp(i, true);
                if ((y_vec / x_vec) * y_vec / x_vec * (x - botPositionX) - (y - botPositionY) < 0) // should turn counterclockwise
                    temp.second = false;
                obsOnRoad.push_back(temp); // record the obsticals that really on the road
                break;
            }
        }
    }
    if (!obsOnRoad.empty())
    {
        hasObs = true;
        cout << "ya" << endl;
        planNewPath(obsOnRoad, xGoal, yGoal);
    }
    else 
        hasObs = false;

    return;
}

void TurtleController::initParam()
{
    // cmd_vel_pub(0, 0, 0);
    pub.linear.x = 0;
    pub.linear.y = 0;
    pub.angular.z = 0;
    publisher_->publish(pub);
    xMoved = 0.0, yMoved = 0.0, wMoved = 0.0;
    remain = goalDistance;
    remain_w = Goal_w - wMoved;
    // modify params about velocity control
    maxVelocity = min(goalDistance / 0.5 * 0.325, 0.325);
    vel_0 = 0.05;
    vel_1 = maxVelocity - 0.05;
    vel_2 = maxVelocity;
    dist_0 = 0.05;
    dist_1 = (maxVelocity + 0.05) / 1.5  - 0.05;
    dist_2 = (maxVelocity + 0.05) / 1.5;
    // modify params about angular velocity control
    tempt_constant = (Goal_w >= 0.06) ? 0.02 : ((Goal_w >= 0.04) ? 0.01 : 0.005);
    maxYawVel = min((Goal_w / 2 - tempt_constant) * 1.5, maxAngularVelocity);
    yaw_0 = tempt_constant;
    yaw_1 = maxYawVel - tempt_constant;
    yaw_2 = maxYawVel;
    angle_0 = tempt_constant;
    angle_1 = (maxYawVel + tempt_constant) / 1.5 - tempt_constant;
    angle_2 = (maxYawVel + tempt_constant) / 1.5;
}

// Transfer the world coordinate into robot coordinate
Eigen::Vector3d TF_World_to_Robot(float World_x, float World_y, double Robot_theta)
{
    Eigen::Vector3d Robot;
    Robot.x() = World_x * cos(Robot_theta) + World_y * sin(Robot_theta);
    Robot.y() = -World_x * sin(Robot_theta) + World_y * cos(Robot_theta);
    return Robot;
}

// Transfer the robot coordinate into world coordinate
Eigen::Vector3d TF_Robot_to_World(float Robot_x, float Robot_y, double Robot_theta)
{
    Eigen::Vector3d World;
    World.x() = Robot_x * cos(Robot_theta) - Robot_y * sin(Robot_theta);
    World.y() = Robot_x * sin(Robot_theta) + Robot_y * cos(Robot_theta);
    return World;
}

// Return if it's arrived or not
int TurtleController::moveTo(double _x, double _y, double _w)
{
    int is_arrived = 0;
    Eigen::Vector3d world_rVel;
    pointToDist(_x, _y, _w);
    initParam();
    // RCLCPP_INFO(this->get_logger(), "step 1\n");
    rclcpp::Rate loop_rate(1 / deltaTime); // 100 Hz loop rate

    while(!is_arrived)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        
        wMoved += rW * deltaTime;
        remain_w = Goal_w - wMoved;
        // RCLCPP_INFO(this->get_logger(), "wMoved: %f\n", wMoved);
        
        xMoved += rVx * deltaTime;
        yMoved += rVy * deltaTime;
        Moved = hypot(xMoved, yMoved);
        remain = goalDistance - Moved;
        // RCLCPP_INFO(this->get_logger(), "rVx: %f, rVy: %f, Moved: %f\n", rVx, rVy, Moved);

        if (abs(remain) > 0.005)
        {
            if (abs(Moved) <= dist_0)
                VelocityNow = vel_0;
            else if (abs(Moved) <= dist_1)
                VelocityNow = (abs(Moved) - dist_0) * 1.5 + vel_0;
            else if (abs(Moved) <= dist_2)
                VelocityNow = pow(((-abs(Moved) + maxVelocity / 1.5 + 0.1 / 3) / dist_0), 1.5) * -vel_0 + maxVelocity;

            else if (abs(remain) <= dist_0)
                VelocityNow = pow(abs(remain) / dist_0, 1.5) * vel_0;
            else if (abs(remain) <= dist_1)
                VelocityNow = (abs(remain) - dist_0) * 1.5 + vel_0;
            else if (abs(remain) <= dist_2)
                VelocityNow = pow(((-abs(remain) + maxVelocity / 1.5 + 0.1 / 3) / dist_0), 1.5) * -vel_0 + maxVelocity;
            else
                VelocityNow = vel_2;

            if (hasObs)
                updateUnitVector(abs(Moved));
            if (goalDistance < 0)
            {
                VelX = -VelocityNow * x_vec;
                VelY = -VelocityNow * y_vec;
            }
            else
            {
                VelX = VelocityNow * x_vec;
                VelY = VelocityNow * y_vec;
            }
            is_arrived = 0;
        }
        else
        {
            VelX = 0;
            VelY = 0;
        }

        if (abs(remain_w) > 0.01)
        {
            // cout << "\033[2J\033[1;1H";
            // RCLCPP_INFO(this->get_logger(), "step 3\n");

            if (abs(wMoved) <= angle_0)
                AngVelW = yaw_0;
            else if (abs(wMoved) <= angle_1)
                AngVelW = (abs(wMoved) - angle_0) * 1.5 + yaw_0;
            else if (abs(wMoved) <= angle_1)
                AngVelW = pow(((-abs(wMoved) + (maxYawVel + tempt_constant) / 1.5) / angle_0), 1.5) * -yaw_0 + maxYawVel;

            else if (abs(remain_w) <= angle_0)
                AngVelW = pow(abs(remain_w) / angle_0, 1.5) * yaw_0;
            else if (abs(remain_w) <= angle_1)
                AngVelW = (abs(remain_w) - angle_0) * 1.5 + yaw_0;
            else if (abs(remain_w) <= angle_2)
                AngVelW = pow(((-abs(remain_w) + (maxYawVel + tempt_constant) / 1.5) / angle_0), 1.5) * -yaw_0 + maxYawVel;
            else
                AngVelW = yaw_2;
        }
        else
            AngVelW = 0;

        if (VelX == 0 && VelY == 0 && AngVelW == 0)
        {
            botPositionX += xMoved;
            botPositionY += yMoved;
            is_arrived = 1;
        }
        else
            is_arrived = 0;

        // Go through TF
        Eigen::Vector3d TF_vel;
        TF_vel = TF_World_to_Robot(VelX, VelY, wMoved);
        VelX = TF_vel.x();
        VelY = TF_vel.y();

        // RCLCPP_INFO(this->get_logger(), "step 4\n");
        // RCLCPP_INFO(this->get_logger(), "ang vel: %f\n", AngVelW);
        pub.linear.x = VelX;
        pub.linear.y = VelY;
        pub.angular.z = AngVelW;
        publisher_->publish(pub);
        loop_rate.sleep();
    }

    // Return the robot status
    return is_arrived;
}