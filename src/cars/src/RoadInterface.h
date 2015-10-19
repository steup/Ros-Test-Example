#pragma once

class RoadInterface {
  private:
    const unsigned int mMaxX;
    const unsigned int mMaxY;
  public:
    enum class Cell : unsigned int {
      Free,
      Blocked,
      Car
    };
    RoadInterface(unsigned int maxY, unsigned int maxX) : mMaxX(maxX), mMaxY(maxY) {}
    virtual ~RoadInterface() {}
    virtual Cell readCell(unsigned int y, unsigned int x) const =0;
    virtual void writeCell(unsigned int y, unsigned int x, Cell cell)=0;
    virtual bool isFree(unsigned int y, unsigned x) const = 0;
    virtual bool isBlocked(unsigned int y, unsigned x) const = 0;
    virtual bool isCar(unsigned int y, unsigned x) const = 0;
    unsigned int maxX() const { return mMaxX; }
    unsigned int maxY() const { return mMaxY; }
};
