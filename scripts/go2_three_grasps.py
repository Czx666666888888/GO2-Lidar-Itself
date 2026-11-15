"""Three completed grasp/StandUp cycles; no hardware access on import."""


def run_three_grasps(run_round, save_progress):
    completed = []
    save_progress(completed)
    for number in range(1, 4):
        result = run_round(number, [r['target_map_xy'] for r in completed])
        if result.get('task_motion_complete') is not True:
            raise RuntimeError('Round did not complete grasp and StandUp; do not count it')
        completed.append(result)
        save_progress(completed)
    return completed
