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
    Roads(unsigned int maxX, unsigned int maxY) : mMaxX(maxX), mMaxY(maxY) {}
    virtual ~Roads() {}
    virtual void writeCell(unsigned int x, unsigned int y, Cell c) =0;
    virtual bool isFree(unsigned int x, unsigned y) const = 0;
    virtual bool isBlocked(unsigned int x, unsigned y) const = 0;
    virtual bool isCar(unsigned int x, unsigned y) const = 0;
    unsigned int maxX() const { return mMaxX; }
    unsigned int maxY() const { return mMaxY; }
};
