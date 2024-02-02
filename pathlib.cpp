#include "pathlib.h"
#include "math.h"
#include "iostream"
#include "eigen3/Eigen/Eigen"
#include <cmath>
#include <gp_Pnt.hxx>
#include <gp_Ax2.hxx>
#include "gce_MakeCirc.hxx"
#include "ElCLib/ElCLib.hxx"
#include "ruckig/ruckig_format.h"
#include "ruckig/ruckig_interface.h"

typedef unsigned int uint;

pathlib::pathlib()
{

}

void pathlib::print(std::string text){
    if(myDebug){
        std::cout<<text<<std::endl;
    }
}

void pathlib::print_segment_vec(){

    double pathlenght=0;
    for(uint i=0; i<segment_vec.size(); i++){
        segment_vec.at(i).print();
        pathlenght+=segment_vec.at(i).lenght;
    }
    std::cout<<"total pathlenght:"<<pathlenght<<std::endl;
}

void pathlib::set_vm_config(double vm){
    motion_cfg.vm=vm;
}

void pathlib::set_a_config(double am){
    motion_cfg.am=am;
}

void pathlib::set_jm_config(double jm){
    motion_cfg.jm=jm;
}

void pathlib::set_debug(bool state){
    myDebug=state;
}

bool pathlib::debug(){
    return myDebug;
}

void pathlib::add_line(point p0, point p1, double v){

    segment seg;
    seg.type=line;
    seg.p0=p0;
    seg.p1=p1;
    seg.v=v;
    get_line_lenght(seg);
    get_line_vel_vector(seg);
    segment_vec.push_back(seg);

    ruckig_result resx;
    resx.curacc=0;
    resx.curpos=0;
    resx.curvel=0;
    resx.taracc=0;
    resx.tarpos=seg.vx;
    resx.tarvel=0;
    resx.period=0.001;
    resx.durationdiscretizationtype=durationdiscretization::Continuous;
    resx.interfacetype=interface::position;
    resx.enable=1;
    resx.maxjerk=motion_cfg.jm;
    resx.maxacc=motion_cfg.am;
    resx.maxvel=motion_cfg.vm;
    resx=ruckig_interface().dofs(resx);
    std::cout<<"duration x:"<<resx.duration<<" sec."<<std::endl;

    ruckig_result resy;
    resy.curacc=0;
    resy.curpos=0;
    resy.curvel=0;
    resy.taracc=0;
    resy.tarpos=seg.vy;
    resy.tarvel=0;
    resy.period=0.001;
    resy.durationdiscretizationtype=durationdiscretization::Continuous;
    resy.interfacetype=interface::position;
    resy.enable=1;
    resy.maxjerk=motion_cfg.jm;
    resy.maxacc=motion_cfg.am;
    resy.maxvel=motion_cfg.vm;
    resy=ruckig_interface().dofs(resy);
    std::cout<<"duration y:"<<resy.duration<<" sec."<<std::endl;

    ruckig_result resz;
    resz.curacc=0;
    resz.curpos=0;
    resz.curvel=0;
    resz.taracc=0;
    resz.tarpos=seg.vz;
    resz.tarvel=0;
    resz.period=0.001;
    resz.durationdiscretizationtype=durationdiscretization::Continuous;
    resz.interfacetype=interface::position;
    resz.enable=1;
    resz.maxjerk=motion_cfg.jm;
    resz.maxacc=motion_cfg.am;
    resz.maxvel=motion_cfg.vm;
    resz=ruckig_interface().dofs(resz);
    std::cout<<"duration z:"<<resz.duration<<" sec."<<std::endl;


}

void pathlib::add_arc(point p0, point pc, point p1, bool cw){

    segment seg;
    seg.type=arc;
    seg.p0=p0;
    seg.pc=pc;
    seg.p1=p1;
    seg.cw=cw;
    get_arc_waypoint(seg.p0,seg.pc,seg.p1,seg.cw,seg.pw);
    get_arc_lenght(seg.p0,seg.pw,seg.p1,seg.lenght);
    segment_vec.push_back(seg);
}

void pathlib::get_arc_waypoint(point p0, //! Start.
                               point pc, //! Center.
                               point p1, //! End.
                               bool cw,  //! Clockwise or counterclockwise.
                               point &pw){

    //! Is it a circle?
    if(p0.x==p1.x && p0.y==p1.y && p0.z==p1.z){
        printf("no arc but circle, processed as 2d circle. \n");

        point p11=pc;
        p11.z+=100;
        pw=rotate_point_around_line(p0,M_PI,pc,p11);
        return;
    }

    //! Arc start to eigen vector 3d.
    Eigen::Vector3d vp0;
    vp0={p0.x,p0.y,p0.z};

    //! Arc start center to eigen vector 3d.
    Eigen::Vector3d vp1;
    vp1={pc.x, pc.y, pc.z};

    //! Arc end center to eigen vector 3d.
    Eigen::Vector3d vp2;
    vp2={p1.x, p1.y, p1.z};

    double radius = (vp1-vp0).norm();
    //! std::cout<<"radius: "<<radius<<std::endl;
    double diameter=radius*2;

    //! Arc angle.
    Eigen::Vector3d va=(vp0-vp1).normalized();
    Eigen::Vector3d vb=(vp2-vp1).normalized();
    //! std::cout<<"va x:"<<va.x()<<" y:"<<va.y()<<" z:"<<va.z()<<std::endl;
    //! std::cout<<"vb x:"<<vb.x()<<" y:"<<vb.y()<<" z:"<<vb.z()<<std::endl;

    //! Arc direction, in arc plane between p1,p3 or v1,v2, doesn't really matter.
    Eigen::Vector3d n=va.cross(vb);
    double nl=n.norm();

    //! When the arc start,center,end are colinear, we assume the plane is xy. The normal is then in z.
    if(nl==0){
        n.x()=0;
        n.y()=0;
        n.z()=-10; //! Gives clockwise output.
        nl=n.norm();
        //! std::cout<<"arc waypoint created, assumming plane is xy, output is clockwise g2."<<std::endl;
    }

    //! Axis to arc's origin.
    Eigen::Vector3d axis=n/sqrt(nl);
    //! std::cout<<"axis trough arc origin x:"<<axis.x()<<" y:"<<axis.y()<<" z:"<<axis.z()<<" l:"<<nl<<std::endl;

    //! Axis to arc's origin.
    Eigen::Vector3d an=axis.normalized();
    //! std::cout<<"axis trough arc origin normalized x:"<<an.x()<<" y:"<<an.y()<<" z:"<<an.z()<<std::endl;

    //! This can be a negative angle if angle > 180 degrrees. Solution is below.
    double angle=acos(va.dot(vb));

    //! https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
    //! Without checking if dot<0, angles > 180 degrees will fail.
    //!
    //!     Determine the sign of the angle
    //!     Find vector V3 = cross product of Va, Vb. (the order is important)
    //!     If (dot product of V3, Vn) is negative, theta is negative. Otherwise, theta is positive.
    //!
    Eigen::Vector3d vab=va.cross(vb);
    double dot=vab.dot(an);
    //! std::cout<<"sign of the angle <0 or >0:"<<dot<<std::endl;

    double arcAngleNegative=false; //! Reset flag.
    if(dot<0){
        double diff=M_PI-angle;
        angle=M_PI+diff;
        arcAngleNegative=true; //! Set flag so user can see there is something going on.
    }

    double arcAngleRad=angle;
    //! std::cout<<"arc angle in radians:"<<angle<<std::endl;
    //! std::cout<<"arc angle in degrees:"<<angle*to_degrees<<std::endl;

    //! Arc, circle circumfence pi*diameter.
    double arcCircumFence=(M_PI*(2*radius));

    //! Arc lenght.
    double arcLenght=(arcAngleRad/(2*M_PI))*arcCircumFence;

    //! Point on arc center line. (Arc center + Axis vector)
    point pointOnArcAxis={vp1.x()+an.x(),vp1.y()+an.y(),vp1.z()+an.z()};

    double i=angle/2;
    if(!cw){
        i=-i;
    }

    //!         Point to rotate.             Arc center             Point on arc center line. (Arc center + Axis vector)
    pw=rotate_point_around_line(p0,i,pc,{vp1.x()+an.x(),vp1.y()+an.y(),vp1.z()+an.z()});
}

void pathlib::get_arc_lenght(point p0, point pw, point p1, double &lenght){
    point pc;
    double radius;
    get_arc_lenght_center_radius(p0,pw,p1,pc,radius,lenght);
}

//! http://paulbourke.net/geometry/rotate/
//!
//!     Rotate a point p by angle theta around an arbitrary line segment p1-p2
//!     Return the rotated point.
//!     Positive angles are anticlockwise looking down the axis
//!     towards the origin.
//!     Assume right hand coordinate system.
point pathlib::rotate_point_around_line(point thePointToRotate,
                                        double theta,            //! Input in radians.
                                        point theLineP1,
                                        point theLineP2){


    point q = {0.0,0.0,0.0};
    double costheta,sintheta;
    point r;

    r.x = theLineP2.x - theLineP1.x;
    r.y = theLineP2.y - theLineP1.y;
    r.z = theLineP2.z - theLineP1.z;
    thePointToRotate.x -= theLineP1.x;
    thePointToRotate.y -= theLineP1.y;
    thePointToRotate.z -= theLineP1.z;

    //! Normalise(&r);
    Eigen::Vector3d v(r.x,r.y,r.z);
    v.norm();
    r.x=v.x();
    r.y=v.y();
    r.z=v.z();

    costheta = cos(theta);
    sintheta = sin(theta);

    q.x += (costheta + (1 - costheta) * r.x * r.x) * thePointToRotate.x;
    q.x += ((1 - costheta) * r.x * r.y - r.z * sintheta) * thePointToRotate.y;
    q.x += ((1 - costheta) * r.x * r.z + r.y * sintheta) * thePointToRotate.z;

    q.y += ((1 - costheta) * r.x * r.y + r.z * sintheta) * thePointToRotate.x;
    q.y += (costheta + (1 - costheta) * r.y * r.y) * thePointToRotate.y;
    q.y += ((1 - costheta) * r.y * r.z - r.x * sintheta) * thePointToRotate.z;

    q.z += ((1 - costheta) * r.x * r.z - r.y * sintheta) * thePointToRotate.x;
    q.z += ((1 - costheta) * r.y * r.z + r.x * sintheta) * thePointToRotate.y;
    q.z += (costheta + (1 - costheta) * r.z * r.z) * thePointToRotate.z;

    q.x += theLineP1.x;
    q.y += theLineP1.y;
    q.z += theLineP1.z;

    return(q);
}

double pathlib::get_line_lenght(point p0, point p1){
    return sqrt(pow(p1.x-p0.x,2)+pow(p1.y-p0.y,2)+pow(p1.z-p0.z,2));
}

void pathlib::get_line_lenght(segment &seg){
    seg.lenght=get_line_lenght(seg.p0,seg.p1);
}

bool pathlib::is_colinear(point p0, point p1, point p2){
    double l0=get_line_lenght(p0,p1);
    double l1=get_line_lenght(p1,p2);
    double l2=get_line_lenght(p0,p2);
    if(l0+l1==l2){
        return 1;
    }
    return 0;
}

bool pathlib::get_arc_lenght_center_radius(point ps, point pw, point pe, point &pc, double &radius, double &lenght){
    Eigen::Vector3d arc_normal;
    double angle_deg;
    return get_arc_lenght_center_radius_normal( ps, pw, pe, pc, radius, lenght, arc_normal,angle_deg);
}

bool pathlib::get_arc_lenght_center_radius_normal(point ps, point pw, point pe, point &pc, double &radius, double &lenght,
                                                  Eigen::Vector3d &arc_normal, double &angle_deg){

    //! Check its a circle.
    if(ps.distance(pe)<1e-6){
        // std::cout<<"error, arc is a closed circle"<<std::endl;
        return 0;
    }

    Eigen::Vector3d p1(ps.x,ps.y,ps.z);
    Eigen::Vector3d p2(pw.x,pw.y,pw.z);
    Eigen::Vector3d p3(pe.x,pe.y,pe.z);

    Eigen::Vector3d v1 = p2-p1;
    Eigen::Vector3d v2 = p3-p1;
    double v1v1, v2v2, v1v2;
    v1v1 = v1.dot(v1);
    v2v2 = v2.dot(v2);
    v1v2 = v1.dot(v2);

    double base = 0.5/(v1v1*v2v2-v1v2*v1v2);
    double k1 = base*v2v2*(v1v1-v1v2);
    double k2 = base*v1v1*(v2v2-v1v2);

    //! Center of arc.
    Eigen::Vector3d center = p1 + v1*k1 + v2*k2;
    pc.set_xyz(center.x(),center.y(),center.z());

    radius = (center-p1).norm();

    if(myDebug){
        // std::cout<<"arc center x:"<<center.x()<<" y:"<<center.y()<<" z:"<<center.z()<<std::endl;
        // std::cout<<"radius:"<<radius<<std::endl;
    }

    //! Arc angle.
    Eigen::Vector3d va=(p1-center).normalized();
    Eigen::Vector3d vb=(p3-center).normalized();
    //! std::cout<<"va x:"<<va.x()<<" y:"<<va.y()<<" z:"<<va.z()<<std::endl;
    //! std::cout<<"vb x:"<<vb.x()<<" y:"<<vb.y()<<" z:"<<vb.z()<<std::endl;

    //! Arc direction, in arc plane between p1,p3 or v1,v2, doesn't really matter.
    Eigen::Vector3d n=v1.cross(v2);
    double nl=n.norm();
    //! Axis to arc's origin.
    Eigen::Vector3d axis=n/sqrt(nl);
    // std::cout<<"axis trough arc origin x:"<<axis.x()<<" y:"<<axis.y()<<" z:"<<axis.z()<<" l:"<<nl<<std::endl;

    //! Axis to arc's origin.
    arc_normal=axis.normalized();
    // std::cout<<"axis trough arc origin normalized x:"<<arc_normal.x()<<" y:"<<arc_normal.y()<<" z:"<<arc_normal.z()<<std::endl;

    //! This can be a negative angle if angle > 180 degrrees. Solution is below.
    double angle=acos(va.dot(vb));

    //! https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
    //! Without checking if dot<0, angles > 180 degrees will fail.
    //!
    //!     Determine the sign of the angle
    //!     Find vector V3 = cross product of Va, Vb. (the order is important)
    //!     If (dot product of V3, Vn) is negative, theta is negative. Otherwise, theta is positive.
    //!
    Eigen::Vector3d vab=va.cross(vb);
    double dot=vab.dot(arc_normal);
    //! std::cout<<"sign of the angle <0 or >0:"<<dot<<std::endl;
    bool arcAngleNegative=false; //! Reset flag.
    if(dot<0){
        double diff=M_PI-angle;
        angle=M_PI+diff;
        arcAngleNegative=true; //! Set flag so user can see there is something going on.
    }

    double arcAngleRad=angle;
    //! std::cout<<"arc angle in radians:"<<angle<<std::endl;

    angle_deg=angle*RadtoDeg;
    // std::cout<<"arc angle in degrees:"<<angle_deg<<std::endl;

    //! Arc, circle circumfence pi*diameter.
    double arcCircumFence=(M_PI*(2*radius));

    //! Arc lenght.
    lenght=(arcAngleRad/(2*M_PI))*arcCircumFence;

    if(myDebug){
        // std::cout<<"arc lenght:"<<lenght<<std::endl;
    }

    return 1;
}

////! Returns arc angle in degrees, from 3 arc points.
double computeArcAngle(const Eigen::Vector3d& p0, const Eigen::Vector3d& pw, const Eigen::Vector3d& p1) {

    Eigen::Vector3d v1 = pw-p0;
    Eigen::Vector3d v2 = p1-p0;
    double v1v1, v2v2, v1v2;
    v1v1 = v1.dot(v1);
    v2v2 = v2.dot(v2);
    v1v2 = v1.dot(v2);

    double base = 0.5/(v1v1*v2v2-v1v2*v1v2);
    double k1 = base*v2v2*(v1v1-v1v2);
    double k2 = base*v1v1*(v2v2-v1v2);

    //! Center of arc.
    Eigen::Vector3d pc = p0 + v1*k1 + v2*k2;
    // std::cout<<"center x:"<<pc.x()<<" y:"<<pc.y()<<" z:"<<pc.z()<<std::endl;

    //! Arc angle.
    Eigen::Vector3d va=(p0-pc).normalized();
    Eigen::Vector3d vb=(p1-pc).normalized();


    //! Arc direction, in arc plane between p1,p3 or v1,v2, doesn't really matter.
    Eigen::Vector3d n=v1.cross(v2);
    double nl=n.norm();
    //! Axis to arc's origin.
    Eigen::Vector3d axis=n/sqrt(nl);

    //! Axis to arc's origin.
    Eigen::Vector3d arc_normal=axis.normalized();
    double angle=acos(va.dot(vb));

    Eigen::Vector3d vab=va.cross(vb);
    double dot=vab.dot(arc_normal);
    //! std::cout<<"sign of the angle <0 or >0:"<<dot<<std::endl;
    bool arcAngleNegative=false; //! Reset flag.
    if(dot<0){
        double diff=M_PI-angle;
        angle=M_PI+diff;
        arcAngleNegative=true; //! Set flag so user can see there is something going on.
    }
    double angle_deg_total=angle*RadtoDeg;
    // std::cout<<"angle total:"<<angle_deg_total<<std::endl;

    return angle_deg_total;
}

//! This function calculates arc angle for p0-pw-p1.
//! To check if poin p is on the arc circumfence it calculates 2 angles, p0-p and p-p1.
//! If sum off the 2 angles = arc angle, point is on arc circumfence.
bool pathlib::is_point_on_arc(point p0_, point pw_, point p1_, point p_){

    double eps=0.1;

    Eigen::Vector3d p0(p0_.x,p0_.y,p0_.z);
    Eigen::Vector3d pw(pw_.x,pw_.y,pw_.z);
    Eigen::Vector3d p1(p1_.x,p1_.y,p1_.z);
    Eigen::Vector3d p(p_.x,p_.y,p_.z);

    Eigen::Vector3d v1 = pw-p0;
    Eigen::Vector3d v2 = p1-p0;
    double v1v1, v2v2, v1v2;
    v1v1 = v1.dot(v1);
    v2v2 = v2.dot(v2);
    v1v2 = v1.dot(v2);

    double base = 0.5/(v1v1*v2v2-v1v2*v1v2);
    double k1 = base*v2v2*(v1v1-v1v2);
    double k2 = base*v1v1*(v2v2-v1v2);

    //! Center of arc.
    Eigen::Vector3d pc = p0 + v1*k1 + v2*k2;
    // std::cout<<"center x:"<<pc.x()<<" y:"<<pc.y()<<" z:"<<pc.z()<<std::endl;

    // Compute the radius of the arc
    double radius = (pc - p0).norm();
    // std::cout<<"radius:"<<radius<<std::endl;


    gp_Pnt p00={p0.x(),p0.y(),p0.z()};
    gp_Pnt pww={pw.x(),pw.y(),pw.z()};
    gp_Pnt p11={p1.x(),p1.y(),p1.z()};
    gp_Pnt pii={p.x(),p.y(),p.z()};

    gce_MakeCirc gce_circle(p00,pww,p11);
    gp_Circ myGpCircle=(gce_circle.Value());

    double u0=ElCLib().Parameter(myGpCircle,p00);
    double u1=ElCLib().Parameter(myGpCircle,p11);
    double up=ElCLib().Parameter(myGpCircle,pii);

    // std::cout<<"elclib Ustart:"<<u0*RadtoDeg<<std::endl;
    // std::cout<<"elclib Uend  :"<<u1*RadtoDeg<<std::endl;
    // std::cout<<"Upoint:"<<up*RadtoDeg<<std::endl;

    bool ok=0;
    if(up>=u0 && up<=u1){
        if(p_.distance({pc.x(),pc.y(),pc.z()})<=radius+eps &&
                p_.distance({pc.x(),pc.y(),pc.z()})>=radius-eps ){
            ok=1;
        }
    }

    if(ok){
        // std::cout<<"point is on arc"<<std::endl;
    } else {
        // std::cout<<"point is not on arc"<<std::endl;
    }
    return ok;
}

bool pathlib::get_circle_lenght_radius(point p1,
                                       point pc,
                                       double &radius,
                                       double &lenght){

    radius=p1.distance(pc);
    lenght=0;

    if(radius<1e-6){
        if(myDebug){
            std::cout<<"error, circle radius to small."<<std::endl;
        }
        return 0;
    }

    lenght=M_PI*(2*radius);
    return 1;
}

bool pathlib::intersect_plane_plane(plane pl1, plane pl2, point &p0, point &p1){

    // Define three points on each plane
    Eigen::Vector3d point1_1(pl1.p1.x,pl1.p1.y,pl1.p1.z);
    Eigen::Vector3d point2_1(pl1.p2.x,pl1.p2.y,pl1.p2.z);
    Eigen::Vector3d point3_1(pl1.p3.x,pl1.p3.y,pl1.p3.z);

    Eigen::Vector3d point1_2(pl2.p1.x,pl2.p1.y,pl2.p1.z);
    Eigen::Vector3d point2_2(pl2.p2.x,pl2.p2.y,pl2.p2.z);
    Eigen::Vector3d point3_2(pl2.p3.x,pl2.p3.y,pl2.p3.z);

    // Find normal vectors of each plane
    Eigen::Vector3d normal1 = (point2_1 - point1_1).cross(point3_1 - point1_1);
    Eigen::Vector3d normal2 = (point2_2 - point1_2).cross(point3_2 - point1_2);

    // Normalize the normal vectors
    normal1.normalize();
    normal2.normalize();

    // Find a point on each plane (using the first point for simplicity)
    Eigen::Vector3d pointOnPlane1 = point1_1;
    Eigen::Vector3d pointOnPlane2 = point1_2;

    // Find the direction vector of the intersection line
    Eigen::Vector3d direction = normal1.cross(normal2);

    // If the direction vector is zero, the planes are parallel or coincident
    if (direction.norm() == 0) {
        // if(myDebug){ std::cout << "Planes are parallel or coincident." << std::endl; }
        return 0;
    } else {
        // Normalize the direction vector
        direction.normalize();

        // Find a point on the intersection line
        Eigen::Vector3d pointOnLine = direction * ((pointOnPlane2 - pointOnPlane1).dot(normal2) / direction.dot(direction));

        p0.set_xyz(pointOnLine.x(),pointOnLine.y(),pointOnLine.z());
        p1.set_xyz(pointOnLine.x()+direction.x(),pointOnLine.y()+direction.y(),pointOnLine.z()+direction.z());

        if(myDebug){
            // p0.print("plane-plane intersection point 1:");
            // p1.print("plane-plane intersection point 2:");
        }
    }
    return 1;
}

//! Calculate the intersection of a ray and a sphere
//! The line segment is defined from p1 to p2
//! The sphere is of radius r and centered at sc
//! There are potentially two points of intersection given by
//! p = p1 + mu1 (p2 - p1)
//! p = p1 + mu2 (p2 - p1)
//! Return FALSE if the ray doesn't intersect the sphere.
float square( float f ) { return (f*f) ;};
std::pair<bool,bool> pathlib::intersect_ray_sphere(point pl0, point pl1, point p0, point pw, point p1, point &pi0, point &pi1){

    point pc;
    double radius,lenght;
    get_arc_lenght_center_radius(p0,pw,p1,pc,radius,lenght);

    float a, b, c, mu, i;

    a = square(pl1.x - pl0.x) + square(pl1.y - pl0.y) + square(pl1.z - pl0.z);
    b = 2 * ((pl1.x - pl0.x) * (pl0.x - pc.x) +
             (pl1.y - pl0.y) * (pl0.y - pc.y) +
             (pl1.z - pl0.z) * (pl0.z - pc.z));
    c = square(pc.x) + square(pc.y) +
            square(pc.z) + square(pl0.x) +
            square(pl0.y) + square(pl0.z) -
            2 * (pc.x * pl0.x + pc.y * pl0.y + pc.z * pl0.z) - square(radius);
    i = b * b - 4 * a * c;

    if (i < 0.0) {
        // no intersection
        return {0,0};
    }
    if (i == 0.0) {
        mu = -b / (2 * a);
        pi0.set_xyz(pl0.x + mu * (pl1.x - pl0.x), pl0.y + mu * (pl1.y - pl0.y), pl0.z + mu * (pl1.z - pl0.z));
        return {1,0};
    }
    if (i > 0.0) {
        mu = (-b + sqrt(square(b) - 4 * a * c)) / (2 * a);
        pi0.set_xyz(pl0.x + mu * (pl1.x - pl0.x), pl0.y + mu * (pl1.y - pl0.y), pl0.z + mu * (pl1.z - pl0.z));

        mu = (-b - sqrt(square(b) - 4 * a * c)) / (2 * a);
        pi1.set_xyz(pl0.x + mu * (pl1.x - pl0.x), pl0.y + mu * (pl1.y - pl0.y), pl0.z + mu * (pl1.z - pl0.z));
        return {1,1};
    }
    return {0,0};
}

std::pair<bool,bool> pathlib::intersect_plane_arc(plane pl1, point p0, point pw, point p1, point &pi0, point &pi1){

    std::pair<bool,bool> ok={0,0};
    point lp0,lp1;
    plane pl2(p0,pw,p1);
    if(!intersect_plane_plane(pl1,pl2,lp0,lp1)){
        return {0,0};
    }

    ok=intersect_ray_sphere(lp0,lp1,p0,pw,p1,pi0,pi1);
    if(myDebug){
        if(ok.first==1){
            // pi0.print("first arc plane intersection:");
        }
        if(ok.second==1){
            // pi1.print("second arc plane intersection:");
        }
    }
    return ok;
}

//! The axis with biggest change in lenght is master.
int pathlib::get_leading_axis(double dx, double dy, double dz){
    if(dx>=dy && dx>=dz){
        if(myDebug){ std::cout<<"x_axis is leading."<<std::endl; }
        return 0;
    }
    if(dy>=dx && dy>=dz){
        if(myDebug){ std::cout<<"y_axis is leading."<<std::endl; }
        return 1;
    }
    if(dz>=dx && dz>=dy){
        if(myDebug){ std::cout<<"z_axis is leading."<<std::endl; }
        return 2;
    }
    if(myDebug){ std::cout<<"error, no axis is leading."<<std::endl; }
    return -1;
}

//! Get the velocity of axis with the most displacement, set the velocity to the axis configuration,
//! limit to path maxvel.
double pathlib::get_velocity_max_leading_axis(int leading_axis, double vm, double vmx, double vmy, double vmz){
    if(leading_axis==0){
        std::min(vmx,vm);
        return vmx;
    }
    if(leading_axis==1){
        std::min(vmy,vm);
        return vmy;
    }
    std::min(vmz,vm);
    return vmz;
}

//! Calculate the velocity vector.
void pathlib::get_line_vel_vector(segment &seg){

    if(seg.type==line){
        double dx=fabs(seg.p1.x-seg.p0.x);
        double dy=fabs(seg.p1.y-seg.p0.y);
        double dz=fabs(seg.p1.z-seg.p0.z);

        double x_ratio=dx/seg.lenght;
        double y_ratio=dy/seg.lenght;
        double z_ratio=dz/seg.lenght;

        seg.vx=x_ratio*seg.v;
        seg.vy=y_ratio*seg.v;
        seg.vz=z_ratio*seg.v;
    }
}

void pathlib::get_vel_start_end_vector(segment &seg, segment &next_seg){

    if(seg.type==line && next_seg.type==line){
        if(is_colinear(seg.p0,seg.p1,next_seg.p1)){
            seg.vex=seg.vx;
            seg.vey=seg.vy;
            seg.vez=seg.vz;
            next_seg.vox=seg.vex;
            next_seg.voy=seg.vey;
            next_seg.voz=seg.vez;
        }
    }
}

void pathlib::get_vel_start_end_vector(std::vector<segment> &seg_vec){

    for(uint i=0; i<seg_vec.size()-1; i++){
        get_vel_start_end_vector(seg_vec.at(i),seg_vec.at(i+1));
    }
}

void pathlib::get_vel_start_end_vector(){

    for(uint i=0; i<segment_vec.size()-1; i++){
        get_vel_start_end_vector(segment_vec.at(i),segment_vec.at(i+1));
    }
}












