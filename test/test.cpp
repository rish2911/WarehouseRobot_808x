/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2022 Adithya Singh, Rishabh Singh, Divyansh Agrawal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/
/**
 * @file robot.hpp
 * @author Divyansh Agrawal
 * @author Rishabh Singh
 * @author Adithya Singh
 * @brief Test script for Robot
 * @version 0.1
 * @date 2022-12-13
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <gtest/gtest.h>
#include <vector>
#include <actionlib/client/simple_action_client.h>

#include <memory>
#include <thread>
#include <chrono>

#include "../include/robot.hpp"

class GTestSuite : public ::testing::Test {
    public:
        GTestSuite() {
        }
        ~GTestSuite() {
        }
};

TEST_F(GTestSuite, get_goals) {
    acme::Robot robot;
    std::vector<std::vector<double>> goals;
    goals = robot.get_goals();
    ASSERT_EQ(goals.size(), 5);
}

TEST_F(GTestSuite, rotate) {
    acme::Robot robot;
    ASSERT_NO_THROW(robot.rotate());

}

TEST_F(GTestSuite, stop) {
    acme::Robot robot;
    ASSERT_NO_THROW(robot.stop());
}

TEST_F(GTestSuite, check_fiducial) {
    acme::Robot robot;
    ASSERT_NO_THROW(robot.check_fiducial());
}

TEST_F(GTestSuite, set_fiducial) {
    acme::Robot robot;
    ASSERT_NO_THROW(robot.set_fiducial(true));
}

TEST_F(GTestSuite, listen) {
    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformListener tfListener(tfBuffer);
    acme::Robot robot;
    std::vector<double> goal;
    goal = robot.listener(tfBuffer);
    ASSERT_EQ(goal.size(), 0);

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    testing::InitGoogleTest(&argc, argv);
    std::thread t([]{while(ros::ok()) ros::spin();});
    auto res = RUN_ALL_TESTS();
    ros::shutdown();
    return res;
}