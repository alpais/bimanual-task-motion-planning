#include "bimanual_action_server.h"

// ACTION TYPE 4: Go to Targets in Cartesian Space
bool BimanualActionServer::bimanual_goto_cart_execution(tf::Transform task_frame, tf::Transform right_att, tf::Transform left_att){

    // Convert attractors to world frame (right robot base frame)
    tf::Transform  right_final_target, left_final_target;
    right_final_target.mult(task_frame, right_att);
    left_final_target.mult(task_frame, left_att);

    // Setting Initial conditions
    if (initial_config == true){
        r_curr_ee_pose = r_ee_pose;
        l_curr_ee_pose = l_ee_pose;
    }

    //        * how to use

    //           CDDynamics *testDyn;
    //           testDyn = new CDDynamics(dim, dt, wn);

    //           testDyn->SetVelocityLimits(velLimits);
    //           testDyn->SetState(initial);
    //           testDyn->SetTarget(target);


    //           start loop
    //               // if new target is set
    //               testDyn->SetTarget(target);

    //               // update dynamics
    //               testDyn->Update();

    //               // get state
    //               testDyn->GetState(state);
    //           end loop


}
