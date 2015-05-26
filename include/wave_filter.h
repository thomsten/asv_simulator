#ifndef ASV_WAVE_FILTER_H
#define ASV_WAVE_FILTER_H

#include <Eigen/Dense>

class WaveFilter
{
 public:
  WaveFilter();
  WaveFilter(double sigma, double omega0, double lambda, double gain, double dT);
  ~WaveFilter();

  void initialize(double sigma, double omega0, double lambda, double gain, double dT);

  double updateFilter();
 private:
  bool initialized_;
  double gain_, dT_;

  struct drand48_data randBuffer;

  Eigen::Matrix2d A_;
  Eigen::Vector2d B_;
  Eigen::Vector2d x_;
};

double get_white_noise(struct drand48_data *buf);

#endif
