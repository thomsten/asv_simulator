#include "wave_filter.h"

// Standard libraries
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <ctime>

// Linear algebra math
#include <Eigen/Dense>

unsigned long long rdtsc(){
  unsigned int lo,hi;
  __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
  return ((unsigned long long)hi << 32) | lo;
}

WaveFilter::WaveFilter() : initialized_(false),
                           dT_(0.0) {};

WaveFilter::WaveFilter(double sigma, double omega0, double lambda, double gain, double dT)
{
  this->initialize(sigma, omega0, lambda, gain, dT);
}

WaveFilter::~WaveFilter()
{
}

void WaveFilter::initialize(double sigma, double omega0, double lambda, double gain, double dT)
{
  if (initialized_)
    return;

  A_ << 0, 1, -omega0*omega0, -2*lambda*omega0;
  B_ << 0, 2*lambda*omega0*sigma;
  x_ << 0,0;
  gain_ = gain;
  dT_ = dT;

  srand48_r(rdtsc(), &randBuffer);

  initialized_ = true;
}


double WaveFilter::updateFilter()
{
  if (!initialized_)
    return 0.0;

  x_ += (A_*x_ + B_*get_white_noise(&randBuffer))*dT_;

  return gain_*x_[1];
}


double get_white_noise(struct drand48_data *buf)
{
  double result;
  drand48_r(buf, &result);
  return 2*(result - 0.5);
}

