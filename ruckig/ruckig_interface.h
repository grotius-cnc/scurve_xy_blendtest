#ifndef CPP_INTERFACE_H
#define CPP_INTERFACE_H

#include <iostream>
#include <vector>
#include <fstream>

//! https://github.com/pantor/ruckig
#include "ruckig.hpp"
#include "ruckig_format.h"

/*
int i=1;
ruckig::Ruckig<ruckig::DynamicDOFs> otg {static_cast<size_t>(i), 0.001};
ruckig::InputParameter<ruckig::DynamicDOFs> in {static_cast<size_t>(i)};
ruckig::OutputParameter<ruckig::DynamicDOFs> out {static_cast<size_t>(i)};
std::array<double, 3> pos, vel, acc;
*/

class ruckig_interface
{
public:

public:
    ruckig_interface();
    //! Calculate a normal cycle.
    ruckig_result dofs(ruckig_result input);

    //! Get the stop lenght.
    double dofs_stop_lenght(ruckig_result input);



private:

};

#endif