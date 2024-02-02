#ifndef PATHLIB_H
#define PATHLIB_H

#include "vector"
#include "pathvector.h"
#include "iostream"
#include "math.h"
#include "eigen3/Eigen/Dense"

#define DegtoRad (M_PI/180.0)
#define RadtoDeg (180.0/M_PI)

// Axis configuration for max_velocity, max_acceleration and jerk_max.
struct motion_config {
    double vm;  // Velocity max.
    double am;  // Acceleration max.
    double jm;  // Jerk max.
};

// Current axis values for velocity, acceleartion and position.
struct motion_current {
    double v;   // Current velocity.
    double a;   // Current acceleration.
    double s;   // Current position.
};

enum segment_types {
    line=0,         // Type is line.
    arc=1,          // Type is arc.
    circle=2        // Type is cicle.
};

// Curve velocity profile plugin type.
enum motion_profiles {
    linear=0,
    scurve=1
};

// Point in 3d space.
struct point {

    double x,y,z;

    void set_xyz(double theX, double theY, double theZ){
        x=theX;
        y=theY;
        z=theZ;
    }
    void print(std::string name){
        std::cout<<name<<" x:"<<x<<" y:"<<y<<" z:"<<z<<std::endl;
    }
    double distance(point p0, point p1){
        return sqrt(pow(p1.x-p0.x,2)+pow(p1.y-p0.y,2)+pow(p1.z-p0.z,2));
    }
    double distance(point p1){
        return sqrt(pow(p1.x-this->x,2)+pow(p1.y-this->y,2)+pow(p1.z-this->z,2));
    }
    void set_x(double value){
        x=value;
    }
    void set_y(double value){
        y=value;
    }
    void set_z(double value){
        z=value;
    }
};

// Splits a path velocity into x,y,z components.
struct velocity_end_vector {
    double vx=0;
    double vy=0;
    double vz=0;
};

// A line or arc described in 3d space.
struct segment {
    segment_types type;
    point p0;       // Start point, Circle circumfence point.
    point pw;       // Arc waypoint.
    point p1;       // End point.
    point pc;       // Arc, circle centerpoint.
    point pn;       // Point on circle normal axis.
    double radius;  // Arc, circle radius.
    double lenght;  // Segment lenght.
    double v;       // Velocity request.
    double vx;      // Velocity vector for x.
    double vy;      // Velocity vector for y.
    double vz;      // Velocity vectorfor z.
    double vox=0;
    double voy=0;
    double voz=0;
    double vex=0;
    double vey=0;
    double vez=0;
    bool cw;

    void print(){
        std::cout<<"segment type:"<<type<<std::endl;;
        if(type==line){
            std::cout<<"p0 x:"<<p0.x<<" y:"<<p0.y<<" z:"<<p0.z<<std::endl;
            std::cout<<"p1 x:"<<p1.x<<" y:"<<p1.y<<" z:"<<p1.z<<std::endl;
        }
        if(type==arc){
            std::cout<<"p0 x:"<<p0.x<<" y:"<<p0.y<<" z:"<<p0.z<<std::endl;
            std::cout<<"pw x:"<<pw.x<<" y:"<<pw.y<<" z:"<<pw.z<<std::endl;
            std::cout<<"p1 x:"<<p1.x<<" y:"<<p1.y<<" z:"<<p1.z<<std::endl;
            std::cout<<"pc x:"<<pc.x<<" y:"<<pc.y<<" z:"<<pc.z<<std::endl;
            std::cout<<"arc is cw: "<< (cw ? "yes" : "no") <<std::endl;
        }
        std::cout<<"pathlenght:"<<lenght<<std::endl;
    }
};

struct plane {
    point p1;
    point p2;
    point p3;

    // Constructor to initialize a plane with three points
    plane(point pa, point pb, point pc)
        : p1(pa), p2(pb), p3(pc) {}

    void set_points(point pa, point pb, point pc){
        p1=pa;
        p2=pb;
        p3=pc;
    }
    void set_xy_plane(){
        p1={0,0,0};
        p2={100,0,0};
        p3={0,100,0};
    }
    void set_xz_plane(){
        p1={0,0,0};
        p2={100,0,0};
        p3={100,0,100};
    }
    void set_yz_plane(){
        p1={0,0,0};
        p2={0,100,0};
        p3={0,100,100};
    }
};

class pathlib
{
public:
    pathlib();

    void set_vm_config(double vm);
    void set_a_config(double am);
    void set_jm_config(double jm);
    void set_debug(bool state);
    bool debug();

    void print(std::string text);
    void print_segment_vec();

    double get_line_lenght(point p0, point p1);
    void get_line_lenght(segment &seg);
    bool is_colinear(point p0, point p1, point p2);
    bool get_arc_lenght_center_radius(point ps,
                                      point pw,
                                      point pe,
                                      point &pc,
                                      double &radius,
                                      double &lenght);
    bool get_arc_lenght_center_radius_normal(point ps,
                                             point pw,
                                             point pe,
                                             point &pc,
                                             double &radius,
                                             double &lenght,
                                             Eigen::Vector3d &arc_normal, double &angle_deg);
    bool get_circle_lenght_radius(point p1,
                                  point pc,
                                  double &radius,
                                  double &lenght);

    void get_arc_waypoint(point p0,    //! Start.
                          point pc,    //! Center.
                          point p1,    //! End.
                          bool cw,     //! Clockwise or counterclockwise.
                          point &pw);  //! Waypoint.
    void get_arc_lenght(point p0, point pw, point p1, double &lenght);
    point rotate_point_around_line(point thePointToRotate,
                                   double theta,            //! Input in radians.
                                   point theLineP1,
                                   point theLineP2);

    bool intersect_plane_plane(plane pl1, plane pl2, point &p0, point &p1);
    std::pair<bool,bool> intersect_ray_sphere(point pl0, point pl1, point p0, point pw, point p1, point &pi0, point &pi1);
    std::pair<bool, bool> intersect_plane_arc(plane pl1, point p0, point pw, point p1, point &pi0, point &pi1);
    bool is_point_on_arc(point p0, point pw, point p1, point p);

    int get_leading_axis(double dx, double dy, double dz);
    double get_velocity_max_leading_axis(int leading_axis, double vm, double vmx, double vmy, double vmz);

    void get_line_vel_vector(segment &seg);
    void get_vel_start_end_vector(segment &seg, segment &next_seg);
    void get_vel_start_end_vector(std::vector<segment> &seg_vec);
    void get_vel_start_end_vector();

    motion_profiles motion_profile;
    std::vector<segment> segment_vec;
    std::vector<velocity_end_vector> vel_end_vec;
    motion_config motion_cfg;
    motion_current motion_cur;

    void add_line(point p0, point p1, double v);
    void add_arc(point p0, point pc, point p1, bool cw);


private:
    bool myDebug=0;
};

#endif // PATHLIB_H
























