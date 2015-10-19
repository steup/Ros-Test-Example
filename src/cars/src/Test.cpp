#include "Car.h"
#include "SimCar.h"

#include <cars/NewCar.h>

#include <ros/ros.h>

#include <gtest/gtest.h>

#include <thread>
#include <atomic>
#include <chrono>

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

class CarTestSuite : public ::testing::Test{
  private:
    ros::ServiceServer mServer;
    string mPlate;
    bool mFailService, mFailUnique;
    bool callback(cars::NewCar::Request& req, cars::NewCar::Response& res){
      mPlate = req.numberPlate;
      res.success = !mFailUnique;
      return !mFailService;
    }
  public:
    CarTestSuite() : mFailService(false), mFailUnique(false){
      mServer = ros::NodeHandle().advertiseService("carRegistry", &CarTestSuite::callback, this);
    }
    ~CarTestSuite() {}
    const string& numberPlate() const { return mPlate; }
    void failUnique()     { mFailUnique = true; }
    void failService()    { mFailService = true; }
};

TEST(InitTestSuite, carCreationTest) {
  ASSERT_THROW(Car car("TestCar");, runtime_error) << "Car did not detect the missing service!";
}

TEST_F(CarTestSuite, creationTest) {
  ASSERT_NO_THROW(Car car("TestCar");) << "Car was not successfully registered!";
}

TEST_F(CarTestSuite, noServiceTest) {
  failService();
  ASSERT_THROW(Car car("TestCar");, runtime_error) << "Car did not detect the missing service!";
}

TEST_F(CarTestSuite, registrationTest) {
  Car car("TestCar");
  ASSERT_EQ(car.numberPlate(), numberPlate()) << "Car registry information is wrong!";
}

TEST_F(CarTestSuite, uniquenessTest) {
  failUnique();
  ASSERT_THROW(Car car("TestCar");, invalid_argument) << "Double creation of cars was not detected correctly";
}

TEST_F(CarTestSuite, localTest) {
  Car car("TestCar");
  ASSERT_EQ(car.steer(), Car::Steering::Straight) << "Default steering straight was not set correctly";
  car.steer(Car::Steering::Left);
  ASSERT_EQ(car.steer(), Car::Steering::Left) << "Steering left was not set correctly";
  car.steer(Car::Steering::Straight);
  ASSERT_EQ(car.steer(), Car::Steering::Straight) << "Steering straight was not set correctly";
  car.steer(Car::Steering::Right);
  ASSERT_EQ(car.steer(), Car::Steering::Right) << "Steering right was not set correctly";
}

TEST_F(CarTestSuite, remoteTest) {
  Car car("TestCar");
  SimCar simCar("TestCar", SimCar::Dir::Up, 1, 1);
  ASSERT_EQ(simCar.steer(), SimCar::Steering::Straight) << "Default steering straight was not set correctly";
  car.steer(Car::Steering::Left);
  sleep_for(milliseconds(10));
  ASSERT_EQ(simCar.steer(), SimCar::Steering::Left) << "Steering left was not transmitted correctly";
  car.steer(Car::Steering::Straight);
  sleep_for(milliseconds(10));
  ASSERT_EQ(simCar.steer(), SimCar::Steering::Straight) << "Steering straight was not transmitted correctly";
  car.steer(Car::Steering::Right);
  sleep_for(milliseconds(10));
  ASSERT_EQ(simCar.steer(), SimCar::Steering::Right) << "Steering right was not transmitted correctly";
}

int main(int argc, char** argv){
  ros::init(argc, argv, "CarTestNode");
  testing::InitGoogleTest(&argc, argv);

  thread t([]{while(ros::ok()) ros::spin();});

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}
