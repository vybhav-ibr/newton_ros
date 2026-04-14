import newton

def rigid_simulate_impl(bridge):
    for _ in range(bridge.sim_substeps):
        bridge.state_0.clear_forces()
        bridge.state_1.clear_forces()
        bridge.viewer.apply_forces(bridge.state_0)

        newton.eval_fk(
            bridge.model,
            bridge.state_0.joint_q,
            bridge.state_0.joint_qd,
            bridge.state_0,
        )
        bridge.model.collide(bridge.state_0, bridge.contacts)

        bridge.rigid_solver.step(
            state_in=bridge.state_0,
            state_out=bridge.state_1,
            control=bridge.control,
            contacts=bridge.contacts,
            dt=bridge.sim_dt,
        )

        bridge.state_0, bridge.state_1 = bridge.state_1, bridge.state_0
        bridge.sim_time += bridge.sim_dt

def soft_simulate_impl(bridge):
    bridge.soft_solver.rebuild_bvh(bridge.state_0)
    for _ in range(bridge.sim_substeps):
        bridge.state_0.clear_forces()
        bridge.state_1.clear_forces()
        bridge.viewer.apply_forces(bridge.state_0)

        newton.eval_fk(
            bridge.model,
            bridge.state_0.joint_q,
            bridge.state_0.joint_qd,
            bridge.state_0,
        )

        bridge.collider.collide(bridge.state_0, bridge.contacts)

        bridge.soft_solver.step(
            state_in=bridge.state_0,
            state_out=bridge.state_1,
            control=bridge.control,
            contacts=bridge.contacts,
            dt=bridge.sim_dt,
        )

        bridge.state_0, bridge.state_1 = bridge.state_1, bridge.state_0
        bridge.sim_time += bridge.sim_dt

def hybrid_simulate_impl(bridge):  
    bridge.soft_solver.rebuild_bvh(bridge.state_0)  
    for _ in range(bridge.sim_substeps):  
        bridge.state_0.clear_forces()  
        bridge.state_1.clear_forces()  
        bridge.viewer.apply_forces(bridge.state_0)  
  
        newton.eval_fk(  
            bridge.model,  
            bridge.state_0.joint_q,  
            bridge.state_0.joint_qd,  
            bridge.state_0,  
        )  
  
        # Step 1: rigid solver writes rigid poses to state_1  
        bridge.rigid_solver.step(  
            bridge.state_0,  
            bridge.state_1,  
            bridge.control,  
            bridge.contacts,  
            bridge.sim_dt,  
        )  
  
        # Step 2: collision detection (uses state_0 particle positions)  
        bridge.collider.collide(bridge.state_0, bridge.contacts)  
  
        # Step 3: VBD solver handles particles only;  
        # reads rigid poses from state_1.body_q (written by rigid solver above)  
        bridge.soft_solver.step(  
            bridge.state_0,  
            bridge.state_1,  
            bridge.control,  
            bridge.contacts,  
            bridge.sim_dt,  
        )  
  
        bridge.state_0, bridge.state_1 = bridge.state_1, bridge.state_0  
        bridge.sim_time += bridge.sim_dt