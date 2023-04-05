/**
 *  @file   open_loop_controller.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Open loop controller class source.
 *
 *  This file implements the open loop controller class.
 */

/*
 *  Project headers.
 */
#include "utility/math.h"
#include "controller/open_loop_controller.h"
#include "common/parameter.h"

/*
 *  Biped namespace.
 */
namespace biped
{
OpenLoopController::OpenLoopController() : gain_(0), reference_(0)
{
}

double
OpenLoopController::getReference() const
{
    /*
     *  Return the open loop controller reference (R).
     */
    // TODO LAB 7 YOUR CODE HERE.
}

void
OpenLoopController::setGain(const double& gain)
{
    /*
     *  Set the open loop controller gain.
     */
    // TODO LAB 7 YOUR CODE HERE.
}

void
OpenLoopController::setSaturation(const ControllerSaturation& saturation)
{
    /*
     *  Set the open loop controller saturation.
     */
    // TODO LAB 7 YOUR CODE HERE.
}

void
OpenLoopController::setReference(const double& reference)
{
    /*
     *  Set the open loop controller reference (R).
     */
    // TODO LAB 7 YOUR CODE HERE.
}

double
OpenLoopController::control()
{
    /*
     *  Calculate the open loop controller output.
     *
     *  The open loop controller output is the product between
     *  the open loop controller gain and the clamped
     *  open loop controller reference (R) between the input
     *  saturation upper and lower bounds.
     *
     *  Open loop controller:
     *  https://en.wikipedia.org/wiki/Open-loop_controller
     */
    // TODO LAB 7 YOUR CODE HERE.

    /*
     *  Return the clamped output between the output
     *  saturation upper and lower bounds as the final
     *  output of the open loop controller.
     */
    // TODO LAB 7 YOUR CODE HERE.
}
}   // namespace biped
