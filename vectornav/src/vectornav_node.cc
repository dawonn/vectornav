/** VectorNav ROS2 node
 *
 * Copyright 2023 Dereck Wonnacott <dereck@gmail.com>
 *
 * Use of this source code is governed by an MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vectornav.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vectornav::Vectornav>(rclcpp::NodeOptions());
  rclcpp::spin(node);  // should never return
  rclcpp::shutdown();  // just in case it does return :)
  return 0;
}
