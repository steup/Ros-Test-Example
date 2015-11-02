#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "RoadInterface.h"
#include "Simulation.h"

#include <string>

using namespace std;
using ::testing::_;
using ::testing::AtLeast;
using ::testing::Lt;
using ::testing::Return;
using ::testing::Assign;

class MockRoad : public RoadInterface {
  public:
    MockRoad(unsigned int maxY, unsigned int maxX) : RoadInterface(maxY, maxX) {}
    MOCK_CONST_METHOD2(readCell, Cell(unsigned int y, unsigned int x));
    MOCK_METHOD3(writeCell, void(unsigned int y, unsigned int x, Cell cell));
    MOCK_CONST_METHOD2(isFree, bool(unsigned int y, unsigned x));
    MOCK_CONST_METHOD2(isBlocked, bool(unsigned int y, unsigned x));
    MOCK_CONST_METHOD2(isCar, bool(unsigned int y, unsigned x));
};

TEST(SimulationSuite, CarCreationTest) {
  class MockCar : public SimCarInterface {
    public:
      MockCar(const string& numberPlate, Dir dir, unsigned int y, unsigned int x)
        : SimCarInterface(numberPlate, dir, y, x) {
          EXPECT_CALL(*this, windows(false, false, false)).Times(1);
        }
      MOCK_METHOD3(windows, void(bool left, bool front, bool right));
  };
  MockRoad road(10, 10);
  EXPECT_EQ(road.maxX(), 10) << "X size was not correctly set";
  EXPECT_EQ(road.maxY(), 10) << "Y size was not correctly set";
  EXPECT_CALL(road, isFree(Lt(road.maxY()), Lt(road.maxX())))
    .Times(AtLeast(1))
    .WillOnce(Return(true))
    .WillRepeatedly(Return(false));
  EXPECT_CALL(road, writeCell(1, 1, RoadInterface::Cell::Car))
    .Times(1);

  Simulation<MockCar> sim(road);

  sim.createCar("TestCar");
}

TEST(SimulationSuite, RunTest0) {
  class MockCar : public SimCarInterface {
    public:
      MOCK_METHOD3(windows, void(bool left, bool front, bool right));
      MOCK_CONST_METHOD0(steer, Steering());
      MOCK_METHOD1(dir, void(Dir dir));
      MOCK_METHOD1(x, void(unsigned int x));
      MOCK_METHOD1(y, void(unsigned int y));
      using SimCarInterface::x;
      using SimCarInterface::y;
      using SimCarInterface::dir;

      MockCar(const string& numberPlate, Dir dir, unsigned int y, unsigned int x)
        : SimCarInterface(numberPlate, dir, 1, 1) {
          EXPECT_CALL(*this, windows(true, true, true))
            .Times(AtLeast(1));
          EXPECT_CALL(*this, steer())
            .Times(AtLeast(1))
            .WillRepeatedly(Return(Steering::Straight));
          EXPECT_CALL(*this, x(2))
            .Times(AtLeast(1))
            .WillRepeatedly(Assign(&mX, 2));
          EXPECT_CALL(*this, y(_)).Times(0);
          EXPECT_CALL(*this, dir(_)).Times(0);
        }
  };

  MockRoad road(10, 10);

  EXPECT_CALL(road, isFree(_, _))
    .WillRepeatedly(Return(true));

  EXPECT_CALL(road, writeCell(1, 1, RoadInterface::Cell::Car))
    .Times(AtLeast(1));

  Simulation<MockCar> sim(road);
  sim.createCar("TestCar");
    
  EXPECT_CALL(road, writeCell(1, 1, RoadInterface::Cell::Free))
    .Times(AtLeast(1));

  EXPECT_CALL(road, writeCell(1, 2, RoadInterface::Cell::Car))
    .Times(AtLeast(1));

  ASSERT_NO_THROW(sim["TestCar"]);
  ASSERT_EQ(sim["TestCar"].dir(), SimCarInterface::Dir::Right) << "Original direction of car is wrong";
  ASSERT_EQ(sim["TestCar"].x(), 1)                             << "Original x position of car is wrong";
  ASSERT_EQ(sim["TestCar"].y(), 1)                             << "Original y position of car is wrong";

  sim.run();
}

int main(int argc, char** argv){
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
