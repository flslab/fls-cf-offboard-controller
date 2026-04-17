import logging
from .vicon_scanner import ViconScanner, mock_scan, sort_and_match, ViconConnectionError, InsufficientMarkersError
from .ui import show_ui

logger = logging.getLogger(__name__)

import multiprocessing as mp

def _run_ui_process(assignments, outliers, mission_data, queue):
    from .ui import show_ui
    import signal
    # Ignore SIGINT in the child to let the parent handle KeyboardInterrupt safely
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    final_assgn, extra_params = show_ui(assignments, outliers, mission_data)
    queue.put((final_assgn, extra_params))

def run_dispatch(manifest_drones, mission_data, vicon_host="192.168.1.39", mock=False):
    """
    Scans Vicon (or mocks), matches sorted points to sorted manifest positions,
    shows the UI for confirmation/editing, and returns the updated drone manifest array 
    where 'init_pos' equals their physical Vicon coordinate.
    """
    n_drones = len(manifest_drones)
    
    if mock:
        logger.info(f"Using mock scanner for {n_drones} drones.")
        vicon_points = mock_scan(manifest_drones)
    else:
        logger.info(f"Connecting to Vicon scanner at {vicon_host} required {n_drones} markers...")
        scanner = ViconScanner(vicon_host)
        try:
            # We use 5 frames so that it averages and confirms stability
            vicon_points = scanner.scan(n_frames=5, n_drones=n_drones)
        except (ViconConnectionError, InsufficientMarkersError) as e:
            logger.error(str(e))
            logger.error("Falling back to MOCK scanner due to Vicon failure.")
            vicon_points = mock_scan(manifest_drones)

    # Generate initial sorted assignments
    assignments, outliers = sort_and_match(vicon_points, manifest_drones)
    
    # Run UI in an isolated process to protect Orchestrator signal handlers and ensure closure
    logger.info("Opening Dispatcher UI for confirmation...")
    ctx = mp.get_context('spawn')
    queue = ctx.Queue()
    p = ctx.Process(target=_run_ui_process, args=(assignments, outliers, mission_data, queue))
    p.start()
    
    try:
        # Wait until UI completes or user interrupts
        p.join()
    except KeyboardInterrupt:
        p.terminate()
        p.join()
        raise  # Re-raise to trigger orchestrator.py emergency handlers
    
    if not queue.empty():
        final_assignments, extra_params = queue.get()
    else:
        final_assignments, extra_params = None, None

    if not final_assignments:
        logger.warning("Dispatcher UI aborted or skipped. No Vicon coordinates will be updated.")
        return manifest_drones
    
    # Update manifest drones with their confirmed real coordinates
    # We will build a dictionary to preserve the exact properties of the original drones
    drones_by_id = {d['id']: d for d in manifest_drones}
    updated_drones = []
    
    for a in final_assignments:
        d_id = a['id']
        drone = drones_by_id[d_id]
        # Replace init_pos with real matched coordinate
        drone['init_pos'] = a['vicon_pos']
        if extra_params:
            if extra_params.get('viewpoint'):
                vp = extra_params['viewpoint']
                offset_x, offset_y, offset_z = -0.025, 0.0, 0.005
                drone['viewpoint'] = [vp[0] + offset_x, vp[1] + offset_y, vp[2] + offset_z]
            if extra_params.get('anchor'):
                drone['anchor'] = extra_params['anchor']
        updated_drones.append(drone)
        
    return updated_drones
