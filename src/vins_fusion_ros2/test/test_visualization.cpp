#include <gtest/gtest.h>
#include <vins_fusion_ros2/visualization.h>

TEST(FeatureConversion, SixChannelsAreSufficient) {
  sensor_msgs::msg::PointCloud message;
  message.points.resize(1);
  message.points[0].z = 1.0f;
  message.channels.resize(6);
  for (auto& channel : message.channels) channel.values = {0.0f};
  message.channels[0].values[0] = 42.0f;
  message.channels[2].values[0] = 100.0f;
  message.channels[5].values[0] = 2.0f;
  const auto features = fromMsg(message);
  ASSERT_EQ(features.size(), 1u);
  ASSERT_EQ(features.at(42).size(), 1u);
  EXPECT_DOUBLE_EQ(features.at(42)[0].second(3), 100.0);
  EXPECT_DOUBLE_EQ(features.at(42)[0].second(6), 2.0);
}

TEST(FeatureConversion, MissingAndTruncatedChannelsAreRejected) {
  sensor_msgs::msg::PointCloud message;
  message.points.resize(2);
  message.channels.resize(5);
  for (auto& channel : message.channels) channel.values = {0.0f, 0.0f};
  EXPECT_TRUE(fromMsg(message).empty());
  message.channels.resize(6);
  message.channels[5].values = {0.0f};
  EXPECT_TRUE(fromMsg(message).empty());
}

TEST(ImageConversion, Generic8UC1IsAcceptedAsMonochrome) {
  sensor_msgs::msg::Image message;
  message.encoding = "8UC1";
  message.width = 2;
  message.height = 1;
  message.step = 2;
  message.data = {10, 200};
  const auto image = fromMsg(message);
  ASSERT_EQ(image.type(), CV_8UC1);
  EXPECT_EQ(image.at<unsigned char>(0, 1), 200);
}
