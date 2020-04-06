#ifndef ACCMETER_H_
#define ACCMETER_H_

#include <LSM303.h>
#include <RunningAverage.h>

class Accelerometer : public LSM303 {
    typedef struct acc_data_xy  {
      unsigned long timestamp;
      int x;
      int y;
      float dir;
    } acc_data_xy;

  public:
    Accelerometer(int read_size) : ra_x(read_size), ra_y(read_size),is_log_(false) {};
    ~Accelerometer() {};
    void enable(void);
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
  private:
    bool is_log_;
    acc_data_xy last;
    RunningAverage ra_x;
    RunningAverage ra_y;
};

#endif
