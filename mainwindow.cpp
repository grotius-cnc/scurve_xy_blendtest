#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "ruckig/ruckig_format.h"
#include "ruckig/ruckig_interface.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    occ = new OcctQtViewer();

    //! Add gridlayout on top of the occ widget.
    //QGridLayout *layout=new QGridLayout(occ);


    ui->gridLayout->addWidget(occ);

    occ->set_view_top();
    occ->set_orthographic();
    //    Handle(AIS_Shape) lineshape_0=draw_primitives().draw_3d_line({0,0,0},{0,80,0});
    //    occ->add_shapevec(lineshape_0);
    //    Handle(AIS_Shape) lineshape_1=draw_primitives().draw_3d_line({20,100,0},{100,100,0});
    //    occ->add_shapevec(lineshape_1);

    //    Handle(AIS_Shape) arcshape_0=draw_primitives().draw_3d_pc_arc({0,80,0},{20,100,0},{20,80,0},{0,0,-1});
    //    occ->add_shapevec(arcshape_0);
    //    //! To update mouse moves.
    //    occ->redraw();

    //    // Set debug output.
    //    plib->set_debug(1);

    //    // Initialize motion limits.
    //    plib->set_vm_config(100);
    //    plib->set_a_config(2);
    //    plib->set_jm_config(10);

    //    plib->add_line({0,20,0},{0,80,0},100);
    //  plib->add_arc({0,80,0},{20,80,0},{20,100,0},1);

    //    plib->get_vel_start_end_vector();

    //    plib->print_segment_vec();

    double cycle=0.01;
    double am=2;
    double vm=6;
    double jm=2;

    ruckig_result resy;
    resy.curacc=0;
    resy.curpos=0;
    resy.curvel=0;
    resy.period=cycle;
    resy.durationdiscretizationtype=durationdiscretization::Continuous;
    resy.interfacetype=interface::position;
    resy.enable=1;
    resy.maxjerk=jm;
    resy.maxacc=am;
    resy.maxvel=vm;
    resy.taracc=0;
    resy.tarpos=100;
    resy.tarvel=0;

    ruckig_result resx;
    resx.curacc=0;
    resx.curpos=0;
    resx.curvel=0;
    resx.period=cycle;
    resx.durationdiscretizationtype=durationdiscretization::Continuous;
    resx.interfacetype=interface::position;
    resx.enable=1;
    resx.maxjerk=jm;
    resx.maxacc=am;
    resx.maxvel=vm;
    resx.taracc=0;
    resx.tarpos=100;
    resx.tarvel=0;

    ruckig_interface *iface=new ruckig_interface();
    resy=iface->dofs(resy);
    resx=iface->dofs(resx);

    double ty=resy.duration;
    double tx=resx.duration;

    double lastposy=0,lastposx=0;
    double lastvely=0,lastvelx=0;
    double lastaccy=0,lastaccx=0;
    double lastty=0, lasttx=0;

    double scale=5;
    bool trigger=0;

    Handle(AIS_Shape) arc=draw_primitives().draw_3d_pc_arc({0,80,0},{20,100,0},{20,80,0},{0,0,-1});
    arc=draw_primitives().colorize(arc,Quantity_NOC_ALICEBLUE,0.5);
    occ->add_shapevec(arc);

    double t=0;
    for(t=0; t<ty; t+=cycle){

        resy=iface->dofs(resy);
        std::cout<<"time y:"<<t<<std::endl;
        std::cout<<"position y:"<<resy.curpos<<std::endl;
        std::cout<<"velocity y:"<<resy.curvel<<std::endl;
        std::cout<<"acceleration y:"<<resy.curacc<<std::endl;

        Handle(AIS_Shape) lineshape_0=draw_primitives().draw_3d_line({0,lastposy,0},{0,resy.curpos,0});
        lastposy=resy.curpos;
        occ->add_shapevec(lineshape_0);

        Handle(AIS_Shape) v_graph=draw_primitives().draw_3d_line({lastty*scale,lastvely*scale,0},{t*scale,resy.curvel*scale,0});
        lastvely=resy.curvel;
        v_graph=draw_primitives().colorize(v_graph,Quantity_NOC_ALICEBLUE,0);
        occ->add_shapevec(v_graph);

        Handle(AIS_Shape) a_graph=draw_primitives().draw_3d_line({lastty*scale,lastaccy*scale,0},{t*scale,resy.curacc*scale,0});
        lastaccy=resy.curacc;
        a_graph=draw_primitives().colorize(a_graph,Quantity_NOC_YELLOW,0);
        occ->add_shapevec(a_graph);

        lastty=t;

        if(resy.curpos>=80.0){ // Start the x axis. motion at this point.
            break;
        }
    }

    std::cout<<""<<std::endl;
    std::cout<<"second motion start."<<std::endl;
    std::cout<<""<<std::endl;


    Handle(AIS_Shape) dot_0=draw_primitives().draw_3d_point({lastty*scale,lastvely*scale,0});
    occ->add_shapevec(dot_0);
    Handle(AIS_Shape) dot_1=draw_primitives().draw_3d_point({lastty*scale,lastaccy*scale,0});
    occ->add_shapevec(dot_1);

    for(double t1=0; t1<tx; t1+=cycle){

        resy=iface->dofs(resy);
        std::cout<<"time y:"<<ty+cycle<<std::endl;
        std::cout<<"position y:"<<resy.curpos<<std::endl;
        std::cout<<"velocity y:"<<resy.curvel<<std::endl;
        std::cout<<"acceleration y:"<<resy.curacc<<std::endl;

        //Handle(AIS_Shape) lineshape_0=draw_primitives().draw_3d_line({0,lastposy,0},{0,resy.curpos,0});

        // occ->add_shapevec(lineshape_0);

        resx=iface->dofs(resx);
        std::cout<<"time x:"<<t<<std::endl;
        std::cout<<"position x:"<<resx.curpos<<std::endl;
        std::cout<<"velocity x:"<<resx.curvel<<std::endl;
        std::cout<<"acceleration x:"<<resx.curacc<<std::endl;

        // XY Path.
        Handle(AIS_Shape) lineshape_1=draw_primitives().draw_3d_line({lastposx,lastposy,0},{resx.curpos,resy.curpos,0});
        lastposx=resx.curpos;
        lastposy=resy.curpos;
        occ->add_shapevec(lineshape_1);
        std::cout<<""<<std::endl;

        if(resx.curpos>20 && !trigger){
            Handle(AIS_Shape) dot_0=draw_primitives().draw_3d_point({lastty*scale,lastvely*scale,0});
            occ->add_shapevec(dot_0);
            Handle(AIS_Shape) dot_1=draw_primitives().draw_3d_point({lastty*scale,lastaccy*scale,0});
            occ->add_shapevec(dot_1);
            trigger=1;
        }

        // Y axis vel,acc.
        Handle(AIS_Shape) v_graph=draw_primitives().draw_3d_line({lastty*scale,lastvely*scale,0},{(t1+t)*scale,resy.curvel*scale,0});
        lastvely=resy.curvel;
        v_graph=draw_primitives().colorize(v_graph,Quantity_NOC_ALICEBLUE,0);
        occ->add_shapevec(v_graph);

        Handle(AIS_Shape) a_graph=draw_primitives().draw_3d_line({lastty*scale,lastaccy*scale,0},{(t1+t)*scale,resy.curacc*scale,0});
        lastaccy=resy.curacc;
        a_graph=draw_primitives().colorize(a_graph,Quantity_NOC_YELLOW,0);
        occ->add_shapevec(a_graph);

        lastty=t1+t;

        if(resy.curpos>80 && resy.curpos<80.1){
            Handle(AIS_Shape) dot_0=draw_primitives().draw_3d_point({resx.curpos*scale,resy.curpos*scale,0});
            occ->add_shapevec(dot_0);

        }

        // X axis vel,acc.
        Handle(AIS_Shape) vx_graph=draw_primitives().draw_3d_line({lasttx*scale,lastvelx*scale,0},{(t1+t)*scale,resx.curvel*scale,0});
        lastvelx=resx.curvel;
        vx_graph=draw_primitives().colorize(vx_graph,Quantity_NOC_GREEN,0);
        occ->add_shapevec(vx_graph);

        Handle(AIS_Shape) ax_graph=draw_primitives().draw_3d_line({lasttx*scale,lastaccx*scale,0},{(t1+t)*scale,resx.curacc*scale,0});
        lastaccx=resx.curacc;
        ax_graph=draw_primitives().colorize(ax_graph,Quantity_NOC_ORANGE,0);
        occ->add_shapevec(ax_graph);

        lasttx=t1+t;
    }

        int h=5;

    std::string text;
    text.append("maxvel:");
    text.append(std::to_string(vm));
    Handle(AIS_Shape) text_shape=draw_primitives().draw_2d_text(text,h,{50,50,0},0);
    occ->add_shapevec(text_shape);

    text.clear();
    text.append("maxacc:");
    text.append(std::to_string(am));
    Handle(AIS_Shape) text_shape1=draw_primitives().draw_2d_text(text,h,{50,40,0},0);
    occ->add_shapevec(text_shape1);

    text.clear();
    text.append("maxjerk:");
    text.append(std::to_string(jm));
    Handle(AIS_Shape) text_shape2=draw_primitives().draw_2d_text(text,h,{50,30,0},0);
    occ->add_shapevec(text_shape2);



    text.clear();
    text.append(". 0,0");
    Handle(AIS_Shape) text_shape3=draw_primitives().draw_2d_text(text,h,{0,0,0},0);
    occ->add_shapevec(text_shape3);

    text.clear();
    text.append(". 0,80");
    Handle(AIS_Shape) text_shape4=draw_primitives().draw_2d_text(text,h,{0,80,0},0);
    occ->add_shapevec(text_shape4);

    text.clear();
    text.append(". 0,100");
    Handle(AIS_Shape) text_shape5=draw_primitives().draw_2d_text(text,h,{0,100,0},0);
    occ->add_shapevec(text_shape5);

    text.clear();
    text.append(". 20,100");
    Handle(AIS_Shape) text_shape6=draw_primitives().draw_2d_text(text,h,{20,100,0},0);
    occ->add_shapevec(text_shape6);

    text.clear();
    text.append(". 100,100");
    Handle(AIS_Shape) text_shape7=draw_primitives().draw_2d_text(text,h,{100,100,0},0);
    occ->add_shapevec(text_shape7);

    occ->redraw();



    /*

    // Test line vectors in each quadrant.
    plib->print("test quadrant upper right.");
    plib->create_sample_one_line_path({0,0,0},{100,100,0});
    plib->get_velocity_end_vec();

    plib->print("test quadrant upper left.");
    plib->create_sample_one_line_path({0,0,0},{-100,100,10});
    plib->get_velocity_end_vec();

    plib->print("test quadrant lower left.");
    plib->create_sample_one_line_path({0,0,0},{-100,-100,10});
    plib->get_velocity_end_vec();

    plib->print("test quadrant lower right.");
    plib->create_sample_one_line_path({0,0,0},{100,-100,10});
    plib->get_velocity_end_vec();

    plib->print("test quadrant lower left to upper right.");
    plib->create_sample_one_line_path({-10,-10,0},{100,100,10});
    plib->get_velocity_end_vec();

    plib->print("test horizontal line vector.");
    plib->create_sample_one_line_path({-100,0,0},{100,0,0});
    plib->get_velocity_end_vec();

    plib->print("test vertical line vector.");
    plib->create_sample_one_line_path({0,0,0},{0,100,0});
    plib->get_velocity_end_vec();

    plib->print("test normal line vector.");
    plib->create_sample_one_line_path({0,0,0},{0,0,100});
    plib->get_velocity_end_vec();

    // Test arc lenghts.
    plib->print("test arc lenght.");
    plib->create_sample_one_arc_path({0,0,0},{50,50,0},{100,0,0});
    plib->get_velocity_end_vec();

    plib->print("test arc lenght.");
    plib->create_sample_one_arc_path({0,0,0},{100,0,0},{50,-50,0});
    plib->get_velocity_end_vec();

    // Test circle lenght.
    plib->print("test circle lenght.");
    plib->create_sample_one_circle_path({-50,0,0},{0,0,0},{0,0,1});
    plib->get_velocity_end_vec();

    plib->print("test quadrant.");
    plib->get_arc_quadrant({-10,0,0},{0,0,0});

    plib->print("test quadrant.");
    plib->get_arc_quadrant({10,0,0},{0,0,0});

    plib->print("test quadrant.");
    plib->get_arc_quadrant({-10,10,0},{0,0,0});

    plib->print("test quadrant.");
    plib->get_arc_quadrant({10,10,0},{0,0,0});

    plib->print("test quadrant.");
    plib->get_arc_quadrant({-10,10,-10},{0,0,0});

    plib->print("test quadrant.");
    plib->get_arc_quadrant({10,10,-10},{0,0,0});

    plib->print("test quadrant.");
    plib->get_arc_quadrant({10,-10,-10},{0,0,0});

    plib->print("test quadrant.");
    plib->get_arc_quadrant({-10,-10,-10},{0,0,0});

    plib->print("test intersection of 2 planes.");

    point p0,p1,pw;
    plane pl1({0,0,0},{100,0,0},{0,100,0}); // top plane.
    plane pl2({0,0,0},{0,0,100},{100,0,0}); // front plane.
    plib->intersect_plane_plane(pl1,pl2,p0,p1);
    p0.print("p0:");
    p1.print("p1:");

    plib->print("test intersection of 2 planes.");
    pl1.set_points({0,0,0},{100,0,0},{0,100,0}); // top plane.
    pl2.set_points({0,0,0},{0,100,00},{0,0,100}); // left plane.
    plib->intersect_plane_plane(pl1,pl2,p0,p1);
    p0.print("p0:");
    p1.print("p1:");

    plib->print("test point on arc.");
    p0={0,0,0};
    pw={0,50,50};
    p1={0,0,100};
    p0.print("p0:");
    pw.print("pw:");
    p1.print("p1:");
    plib->is_point_on_arc(p0,pw,p1,{0,50,50});

    plib->print("");
    plib->print("test plane and arc intersection.");
    point pi0,pi1;
    pl1.set_xz_plane();
    p0={-50,0,0};
    pw={0,50,0};
    p1={0,-50,0};
    plib->print("test xz plane intersections.");
    int intersections_x=0;
    int intersections_y=0;
    int intersections_z=0;
    std::pair<bool,bool> ok=plib->intersect_plane_arc(pl1,p0,pw,p1,pi0,pi1); // ok
    if(ok.first==1){
        plib->is_point_on_arc(p0,pw,p1,pi0);
        intersections_x++;
    }
    if(ok.second==1){
        if(plib->is_point_on_arc(p0,pw,p1,pi1)){
            intersections_x++;
        }
    }

    plib->print("test yz plane intersections.");
    pl1.set_yz_plane();
    ok=plib->intersect_plane_arc(pl1,p0,pw,p1,pi0,pi1); // ok
    if(ok.first==1){
        if(plib->is_point_on_arc(p0,pw,p1,pi0)){
            intersections_y++;
        }

    }
    if(ok.second==1){
        if(plib->is_point_on_arc(p0,pw,p1,pi1)){
            intersections_y++;
        }
    }

    plib->print("test xy plane intersections.");
    pl1.set_xy_plane();
    ok=plib->intersect_plane_arc(pl1,p0,pw,p1,pi0,pi1); // ok
    if(ok.first==1){
        if(plib->is_point_on_arc(p0,pw,p1,pi0)){
            intersections_z++;
        }
    }
    if(ok.second==1){
        if(plib->is_point_on_arc(p0,pw,p1,pi1)){
            intersections_z++;
        }
    }

    std::cout<<"intersections_x:"<<intersections_x<<std::endl;
    std::cout<<"intersections_y:"<<intersections_y<<std::endl;
    std::cout<<"intersections_z:"<<intersections_z<<std::endl;
    */
}

MainWindow::~MainWindow()
{
    delete ui;
}
























