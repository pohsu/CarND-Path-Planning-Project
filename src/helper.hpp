#ifndef HELPER_H_
#define HELPER_H_

#include <vector>
#include <iostream>
using namespace std;

// For converting back and forth between radians and degrees.
double pi();
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

void global2car(vector<double> &pts_x, vector<double> &pts_y, double ref_x, double ref_y, double ref_yaw);
void car2global(double &pts_x, double &pts_y, double ref_x, double ref_y, double ref_yaw);
template <typename T>
void print_vector(vector<T> elements, string name){
  cout << name << ": ";
  for(auto e: elements){
    cout << e << " ";
  }
  cout << endl;
}
#endif
